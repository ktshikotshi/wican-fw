# cherylink: CheryLink drive/charge history, efficiency and battery info
# for the Chery Q (fork component; see README.md). Listens to autopid's
# value sink only, never transmits. cl_engine.c is pure C (host_test/).
idf_component_register(
    SRCS "cherylink.c" "cl_engine.c" "cl_ble.c" "cl_ble_core.c"
    INCLUDE_DIRS "include"
    EMBED_FILES "web/cherylink.html"
    PRIV_REQUIRES autopid flash_jobs http_server_manager rtc_manager
                  esp_http_server esp_timer ble_manager
                  data_destinations esp_app_format
)
