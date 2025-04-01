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
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/pm/pm.h>

LOG_MODULE_REGISTER(softsim_sample, LOG_LEVEL_INF);

// semaphores
K_SEM_DEFINE(lte_connected, 0, 1);	// semaphore to signal that the LTE connection is established

static void lte_handler(const struct lte_lc_evt *const evt);
static int server_connect(void);

static int client_fd;
static struct sockaddr_storage host_addr;
static struct k_work_delayable server_transmission_work;

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

static void server_transmission_work_fn(struct k_work *work)
{
    // Get fresh signal quality before sending
    request_signal_quality();
    
    // Give a little time for signal quality to update
    k_sleep(K_MSEC(100));
    
    // Create JSON with signal quality
    char buffer[128];
    snprintf(buffer, sizeof(buffer), 
             "{\"message\":\"Hello from Onomondo!\",\"rsrp\":%d,\"rsrq\":%d}", 
             current_rsrp, current_rsrq);

    int err = send(client_fd, buffer, strlen(buffer), 0);

    if (err < 0) {
        LOG_ERR("Failed to transmit UDP packet, %d", errno);
        k_work_schedule(&server_transmission_work, K_SECONDS(2));
        return;
    }

    k_work_schedule(&server_transmission_work, K_SECONDS(15));
}

static void work_init(void)
{
  k_work_init_delayable(&server_transmission_work, server_transmission_work_fn);
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
                LOG_INF("Neighbor cell %d: PCI=%d, RSRP=%d dBm, RSRQ=%d dB",
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