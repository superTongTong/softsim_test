cmake_minimum_required(VERSION 3.20.0)

list(APPEND OVERLAY_CONFIG "$ENV{ZEPHYR_BASE}/../modules/lib/onomondo-softsim/overlay-softsim.conf")

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(softsim_test)

target_sources(app PRIVATE src/main.c)
