#include "nimble/ble.h"
#include "host/ble_hs.h"
#include "esp_nimble_hci.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
//#include <ble_svc_gap.h>
//#include "services/gap/ble_svc_gap.h"

// BLE Variables
static uint8_t ble_address[6];
//static const char *device_name = "ESP32_S3";

// Callback for BLE connection events
static int ble_gap_event(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status == 0) {
                printf("Connected to device\n");
            } else {
                printf("Connection failed\n");
            }
            break;

        case BLE_GAP_EVENT_DISCONNECT:
            printf("Disconnected from device\n");
            break;

        default:
            break;
    }
    return 0;
}

void ble_host_task(void *param) {
    nimble_port_run(); // This runs the NimBLE stack
    nimble_port_freertos_deinit();
}

void ble_hs_reset(int reason) {
    printf("BLE stack reset, reason: %d\n", reason);
}

void ble_hs_sync() {
    printf("BLE stack synchronized and ready\n");

    // Start advertising after synchronization
    ble_advertise();
}

// Initialize BLE
void ble_init() {
    nimble_port_freertos_init(ble_host_task);
    esp_nimble_hci_init();
    ble_hs_cfg.reset_cb = ble_hs_reset;
    ble_hs_cfg.sync_cb = ble_hs_sync;

    // Start advertising
    ble_advertise();
}

// Start Advertising
void ble_advertise() {
    struct ble_gap_adv_params adv_params = {0};
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                      &adv_params, ble_gap_event, NULL);
    printf("Advertising started\n");
}
