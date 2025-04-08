#include <stdio.h>
#include <string.h>

#include <nrf_modem.h>
#include <nrf_modem_at.h>
#include <nrf_softsim.h>

#include <pm_config.h>

#include <modem/lte_lc.h>
#include <modem/nrf_modem_lib.h>
#include <modem/modem_info.h>

#include <zephyr/kernel.h>
#include <zephyr/net/socket.h>
// #include <zephyr/drivers/clock_control.h>
// #include <zephyr/drivers/clock_control/nrf_clock_control.h>
#include <zephyr/device.h>
// #include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/pm/pm.h>
// header for GNSS interface
#include <nrf_modem_gnss.h>

LOG_MODULE_REGISTER(softsim_sample, LOG_LEVEL_INF);

// semaphores
K_SEM_DEFINE(lte_connected, 0, 1);	// semaphore to signal that the LTE connection is established
K_SEM_DEFINE(gnss_fix, 0, 1); // Semaphore for GNSS fix

static void lte_handler(const struct lte_lc_evt *const evt);
static int server_connect(void);

static int client_fd;
static struct sockaddr_storage host_addr;
static struct k_work_delayable server_transmission_work;

// Add with your other global variables
static bool gnss_running = false;
static double latitude = 0.0;
static double longitude = 0.0;
static float accuracy = 0.0;
static bool has_fix = false;

// Signal quality tracking
static int16_t current_rsrp = 0;
static int16_t current_rsrq = 0;

static void request_signal_quality(void)
{
    int err;
    
    // Request neighbor cell measurements (includes signal strength)
    struct lte_lc_ncellmeas_params params = {0};
    err = lte_lc_neighbor_cell_measurement(&params);
    if (err) {
        LOG_WRN("Failed to request neighbor cell measurements: %d", err);

    } else {
        LOG_DBG("Neighbor cell measurement requested");
    }
}

static int request_gnss_fix(bool wait_for_fix)
{
    int err;
    
    // If GNSS is already running, do nothing
    if (gnss_running) {
        LOG_INF("GNSS is already running");
        return 0;
    }
    
    LOG_INF("Starting GNSS fix");
    
    // Clear the semaphore before starting
    k_sem_reset(&gnss_fix);
    
    // Start GNSS
    err = nrf_modem_gnss_start();
    if (err) {
        LOG_ERR("Failed to start GNSS: %d", err);
        return err;
    }
    
    gnss_running = true;
    
    if (wait_for_fix) {
        LOG_INF("Waiting for GNSS fix (timeout: 60s)");
        
        // Wait for a fix or timeout
        if (k_sem_take(&gnss_fix, K_SECONDS(60)) != 0) {
            LOG_WRN("GNSS fix timed out");
            
            // Stop GNSS to save power
            err = nrf_modem_gnss_stop();
            if (err) {
                LOG_ERR("Failed to stop GNSS: %d", err);
            } else {
                gnss_running = false;
            }
            
            return -ETIMEDOUT;
        }
    }
    
    return 0;
}

static void server_transmission_work_fn(struct k_work *work)
{
  // Get fresh signal quality before sending
    request_signal_quality();
    
    // Give a little time for signal quality to update
    k_sleep(K_MSEC(100));

    // Request GNSS location before sending
    LOG_INF("Requesting GNSS location fix");
    int gnss_err = request_gnss_fix(true); // true = wait for fix
    if (gnss_err) {
        LOG_WRN("Could not get GPS fix, sending last known position or zeros");
    }
    
    // Create JSON with signal quality
    char buffer[256];
    snprintf(buffer, sizeof(buffer), 
             "{\"message\":\"rsrp\":%d,\"rsrq\":%d,\"lat\":%.6f,\"lon\":%.6f,\"accuracy\":%.1f,\"has_fix\":%s}", 
             current_rsrp, current_rsrq, latitude, longitude, accuracy, has_fix ? "true" : "false");

    int err = send(client_fd, buffer, strlen(buffer), 0);

    if (err < 0) {
        LOG_ERR("Failed to transmit UDP packet, %d", errno);
        k_work_schedule(&server_transmission_work, K_SECONDS(2));
        return;
    }

    k_work_schedule(&server_transmission_work, K_SECONDS(180)); // Schedule next transmission in 3 minutes
}

static void work_init(void)
{
  k_work_init_delayable(&server_transmission_work, server_transmission_work_fn);
}

static void gnss_event_handler(int event)
{
	int err, num_satellites;
  struct nrf_modem_gnss_pvt_data_frame pvt_data;

	switch (event) {
	case NRF_MODEM_GNSS_EVT_PVT:
		num_satellites = 0;
		for (int i = 0; i < 12 ; i++) {
			if (pvt_data.sv[i].signal != 0) {
				num_satellites++;
			}
		}
		LOG_INF("Searching. Current satellites: %d", num_satellites);
		err = nrf_modem_gnss_read(&pvt_data, sizeof(pvt_data), NRF_MODEM_GNSS_DATA_PVT);
		if (err) {
			LOG_ERR("nrf_modem_gnss_read failed, err %d", err);
			return;
		}
		if (pvt_data.flags & NRF_MODEM_GNSS_PVT_FLAG_FIX_VALID) {

      // We have a valid fix
      latitude = pvt_data.latitude;
      longitude = pvt_data.longitude;
      accuracy = pvt_data.accuracy;
      has_fix = true;
			LOG_INF("GNSS Fix: Lat: %.6f, Lon: %.6f, Accuracy: %.1fm", 
                      latitude, longitude, accuracy);
                
      // Signal that we have a fix
      k_sem_give(&gnss_fix);
			// After getting a fix, we can stop GNSS to save power
      err = nrf_modem_gnss_stop();
      if (err) {
          LOG_ERR("Failed to stop GNSS: %d", err);
      } else {
          gnss_running = false;
          LOG_INF("GNSS stopped after fix");
      }
		}
		/* Check for the flags indicating GNSS is blocked */
		if (pvt_data.flags & NRF_MODEM_GNSS_PVT_FLAG_DEADLINE_MISSED) {
			LOG_INF("GNSS blocked by LTE activity");
		} else if (pvt_data.flags & NRF_MODEM_GNSS_PVT_FLAG_NOT_ENOUGH_WINDOW_TIME) {
			LOG_INF("Insufficient GNSS time window");
		}
		break;

	case NRF_MODEM_GNSS_EVT_PERIODIC_WAKEUP:
		LOG_INF("GNSS has woken up");
		break;
	case NRF_MODEM_GNSS_EVT_SLEEP_AFTER_FIX:
		LOG_INF("GNSS enter sleep after fix");
		break;
	default:
		break;
	}
}

static int gnss_init(void)
{
    int err;
    
    // Register GNSS event handler
    err = nrf_modem_gnss_event_handler_set(gnss_event_handler);
    if (err) {
        LOG_ERR("Failed to set GNSS event handler: %d", err);
        return err;
    }

    // Set fix interval to single fix (0 = single, >0 = periodic interval in seconds)
    uint16_t fix_interval = 0; // Single fix
    err = nrf_modem_gnss_fix_interval_set(fix_interval);
    if (err) {
        LOG_ERR("Failed to set fix interval: %d", err);
    }
    
    // Set fix timeout to 60 seconds
    uint16_t fix_timeout = 60;
    err = nrf_modem_gnss_fix_retry_set(fix_timeout);
    if (err) {
        LOG_ERR("Failed to set fix timeout: %d", err);
    }
    
    LOG_INF("GNSS initialized successfully");
    return 0;
}

static void lte_handler(const struct lte_lc_evt *const evt)
{
    switch (evt->type) {
        case LTE_LC_EVT_NW_REG_STATUS:
            if ((evt->nw_reg_status != LTE_LC_NW_REG_REGISTERED_HOME) &&
                (evt->nw_reg_status != LTE_LC_NW_REG_REGISTERED_ROAMING)) {
                break;
            }

            LOG_INF("Network registration status: %s",
                  evt->nw_reg_status == LTE_LC_NW_REG_REGISTERED_HOME ? 
                  "Connected - home network" : "Connected - roaming");
                  
            k_sem_give(&lte_connected);
            break;
        
        case LTE_LC_EVT_PSM_UPDATE:
            LOG_INF("PSM parameter update: TAU: %d, Active time: %d", evt->psm_cfg.tau, evt->psm_cfg.active_time);
            if (evt->psm_cfg.active_time == -1){
              LOG_ERR("Network rejected PSM parameters. Failed to enable PSM");
            }
            break;
        case LTE_LC_EVT_EDRX_UPDATE: {
            char log_buf[60];
            ssize_t len;

            len = snprintf(log_buf, sizeof(log_buf), "eDRX parameter update: eDRX: %f, PTW: %f",
                           (double)evt->edrx_cfg.edrx, (double)evt->edrx_cfg.ptw);
            if (len > 0) {
                LOG_INF("%s\n", log_buf);
            }
            break;
        }
        case LTE_LC_EVT_RRC_UPDATE:
            LOG_INF("RRC mode: %s\n", evt->rrc_mode == LTE_LC_RRC_MODE_CONNECTED ? "Connected" : "Idle");
            break;
        case LTE_LC_EVT_CELL_UPDATE:
            LOG_INF("LTE cell changed: Cell ID: %d, Tracking area: %d", evt->cell.id, evt->cell.tac);
            break;

        // Add signal strength event handler
        case LTE_LC_EVT_NEIGHBOR_CELL_MEAS:
        LOG_INF("Neighbor cell measurement event received");
            
        if (evt->cells_info.current_cell.id != LTE_LC_CELL_EUTRAN_ID_INVALID) {
            // Current cell information is available
            current_rsrp = RSRP_IDX_TO_DBM(evt->cells_info.current_cell.rsrp);
            current_rsrq = RSRQ_IDX_TO_DB(evt->cells_info.current_cell.rsrq);
            
            LOG_INF("Current cell: ID=%d, RSRP=%d dBm, RSRQ=%d dB", 
                   evt->cells_info.current_cell.id,
                   current_rsrp,
                   current_rsrq);
        }
        
        if (evt->cells_info.ncells_count > 0) {
            // Neighboring cells information is available
            LOG_INF("Found %d neighbor cells", evt->cells_info.ncells_count);
            
            for (int i = 0; i < evt->cells_info.ncells_count; i++) {
                LOG_INF("Neighbor cell %d: PCI=%d, RSRP=%.2f dBm, RSRQ=%.2f dB",
                       i+1,
                       evt->cells_info.neighbor_cells[i].phys_cell_id,
                       RSRP_IDX_TO_DBM(evt->cells_info.neighbor_cells[i].rsrp),
                       RSRQ_IDX_TO_DB(evt->cells_info.neighbor_cells[i].rsrq));
            }
        }
            break;
            
        default:
            break;
    }
}

static void modem_connect(void)
{
  int err = lte_lc_connect_async(lte_handler);
    if (err) {
        LOG_ERR("Connecting to LTE network failed, error: %d", err);
        return;
    }
}

static void server_disconnect(void)
{
  (void)close(client_fd);
}

static int server_init(void)
{
  struct sockaddr_in *server4 = ((struct sockaddr_in *)&host_addr);

  server4->sin_family = AF_INET;
  server4->sin_port = htons(4321);

  inet_pton(AF_INET, "1.2.3.4", &server4->sin_addr);

  return 0;
}

static int server_connect(void)
{
  int err;

  client_fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (client_fd < 0) {
    LOG_ERR("Failed to create UDP socket: %d\n", errno);
    err = -errno;
    goto error;
  }

  err = connect(client_fd, (struct sockaddr *)&host_addr, sizeof(struct sockaddr_in));
  if (err < 0) {
    LOG_ERR("Connect failed : %d\n", errno);
    goto error;
  }

  return 0;

error:
  server_disconnect();
  return err;
}

int main(void)
{
  LOG_INF("SoftSIM sample started.");

  int32_t err = nrf_modem_lib_init();
  if (err) {
    LOG_ERR("Failed to initialize modem library, error: %d\n", err);
  }

  work_init();

  modem_connect();

  LOG_INF("Waiting for LTE connect event.\n");
  do {
  } while (k_sem_take(&lte_connected, K_SECONDS(10)));

  LOG_INF("LTE connected!\n");

  // Initialize GNSS
  err = gnss_init();
  if (err) {
      LOG_ERR("Failed to initialize GNSS: %d", err);
      // Continue anyway - we'll send without GPS data
  }
  // Request initial signal quality check
  request_signal_quality();

  err = server_init();
  if (err) {
    LOG_ERR("Not able to initialize UDP server connection\n");
    return -1;
  }

  err = server_connect();
  if (err) {
    LOG_ERR("Not able to connect to UDP server\n");
    return -1;
  }

  k_work_schedule(&server_transmission_work, K_NO_WAIT);
}