#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdbool.h>
#include "gattlib.h"

// Define BT_SEC_LOW if not defined already.
// You can adjust the value if your environment requires a different security level.
#ifndef BT_SEC_LOW
#define BT_SEC_LOW 0
#endif

// Define the target device name and the 16-bit characteristic UUID to subscribe.
#define TARGET_DEVICE_NAME "Ping"
#define CHARACTERISTIC_UUID "1234"

// Global variable to store the BLE device address once found.
static char g_target_address[18] = {0};

// Updated callback: Note the first parameter (adapter) is required.
void device_discovered(gattlib_adapter_t *adapter, const char* addr, const char* name, void* user_data) {
    (void)adapter;    // Unused parameter.
    (void)user_data;  // Unused parameter.
    if (name && strcmp(name, TARGET_DEVICE_NAME) == 0) {
        strncpy(g_target_address, addr, sizeof(g_target_address) - 1);
        printf("Found device: %s [%s]\n", name, addr);
    }
}

// Notification handler for BLE notifications.
void notification_handler(const uuid_t* uuid, const uint8_t* data, size_t data_length, void* user_data) {
    (void)user_data; // Unused parameter.
    char uuid_str[MAX_LEN_UUID_STR + 1] = {0};
    gattlib_uuid_to_string(uuid, uuid_str, sizeof(uuid_str));
    printf("Notification from %s: ", uuid_str);
    for (size_t i = 0; i < data_length; i++) {
        printf("%02x ", data[i]);
    }
    printf("\n");
}

int main() {
    int ret;
    gattlib_adapter_t* adapter = NULL;

    // Open the default BLE adapter.
    ret = gattlib_adapter_open(NULL, &adapter);
    if (ret) {
        fprintf(stderr, "Failed to open BLE adapter.\n");
        return 1;
    }

    // Start scanning for the target device.
    // Note that the updated API requires a fourth user_data parameter.
    printf("Scanning for device with name: %s...\n", TARGET_DEVICE_NAME);
    ret = gattlib_adapter_scan_enable(adapter, device_discovered, 10, NULL);
    if (ret) {
        fprintf(stderr, "Failed to scan for BLE devices (error: %d).\n", ret);
        gattlib_adapter_close(adapter);
        return 1;
    }

    // Check if our target device was found.
    if (strlen(g_target_address) == 0) {
        printf("Device with name '%s' not found.\n", TARGET_DEVICE_NAME);
        gattlib_adapter_close(adapter);
        return 0;
    }

    // Stop scanning since we found our device.
    ret = gattlib_adapter_scan_disable(adapter);
    if (ret) {
        fprintf(stderr, "Failed to disable scanning (error: %d).\n", ret);
    }

    printf("Connecting to device %s...\n", g_target_address);
    /*
       Updated connect signature:
         gattlib_connection_t* gattlib_connect(gattlib_adapter_t* adapter,
                                              const char *dst,
                                              int options,
                                              int sec_level,
                                              int psm);
       Here we use GATTLIB_CONNECTION_OPTIONS_LEGACY_DEFAULT for options,
       BT_SEC_LOW (defined above) for the security level, and 0 for psm.
    */
    gattlib_connection_t* connection = gattlib_connect(adapter,
                                                       g_target_address,
                                                       GATTLIB_CONNECTION_OPTIONS_LEGACY_DEFAULT,
                                                       BT_SEC_LOW,
                                                       0);
    if (connection == NULL) {
        fprintf(stderr, "Failed to connect to the BLE device.\n");
        gattlib_adapter_close(adapter);
        return 1;
    }
    printf("Connected to device %s.\n", g_target_address);

    // Register the notification handler.
    ret = gattlib_register_notification(connection, notification_handler, NULL);
    if (ret) {
        fprintf(stderr, "Failed to register notification handler (error: %d).\n", ret);
        gattlib_disconnect(connection, false);
        gattlib_adapter_close(adapter);
        return 1;
    }

    // Convert the 16-bit characteristic UUID to a full 128-bit UUID string.
    char char_uuid_str[37];
    snprintf(char_uuid_str, sizeof(char_uuid_str), "0000%s-0000-1000-8000-00805F9B34FB", CHARACTERISTIC_UUID);
    uuid_t char_uuid;
    ret = gattlib_string_to_uuid(char_uuid_str, strlen(char_uuid_str) + 1, &char_uuid);
    if (ret) {
        fprintf(stderr, "Failed to convert UUID string (%s) to uuid_t (error: %d).\n", char_uuid_str, ret);
        gattlib_disconnect(connection, false);
        gattlib_adapter_close(adapter);
        return 1;
    }

    // Start notifications on the characteristic.
    ret = gattlib_notification_start(connection, &char_uuid);
    if (ret) {
        fprintf(stderr, "Failed to start notification on characteristic %s (error: %d).\n", char_uuid_str, ret);
        gattlib_disconnect(connection, false);
        gattlib_adapter_close(adapter);
        return 1;
    }
    printf("Notifications enabled on characteristic %s.\n", char_uuid_str);

    // Wait for notifications for 30 seconds.
    sleep(30);

    // Stop notifications.
    ret = gattlib_notification_stop(connection, &char_uuid);
    if (ret) {
        fprintf(stderr, "Failed to stop notification on characteristic %s (error: %d).\n", char_uuid_str, ret);
    }
    printf("Stopping notifications and disconnecting...\n");

    // Disconnect from the device.
    gattlib_disconnect(connection, false);
    gattlib_adapter_close(adapter);
    return 0;
}
