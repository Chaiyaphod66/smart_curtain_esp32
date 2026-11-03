#pragma once

// Copy this file to include/secrets.h and replace both values.
// include/secrets.h is ignored by Git and must not be committed.
// Keep WIFI_USE_STATIC_IPV4 false for DHCP, or replace the example network
// values with addresses that belong to the target network.
constexpr bool WIFI_USE_STATIC_IPV4 = false;
constexpr uint8_t WIFI_STATIC_IPV4[4] = {172, 20, 10, 2};
constexpr uint8_t WIFI_GATEWAY_IPV4[4] = {172, 20, 10, 1};
constexpr uint8_t WIFI_SUBNET_MASK_IPV4[4] = {255, 255, 255, 240};
constexpr uint8_t WIFI_PRIMARY_DNS_IPV4[4] = {1, 1, 1, 1};
constexpr uint8_t WIFI_SECONDARY_DNS_IPV4[4] = {8, 8, 8, 8};

// Optional Cloud bridge. Keep disabled until a TLS MQTT broker and credentials
// are configured. The browser never receives these device credentials.
constexpr bool MQTT_ENABLED = false;
constexpr char MQTT_HOST[] = "YOUR_CLUSTER_HOST";
constexpr uint16_t MQTT_PORT = 8883;
constexpr char MQTT_USERNAME[] = "YOUR_DEVICE_MQTT_USERNAME";
constexpr char MQTT_PASSWORD[] = "YOUR_DEVICE_MQTT_PASSWORD";
constexpr char MQTT_DEVICE_ID[] = "smart-curtain-esp32";
constexpr char MQTT_TOPIC_PREFIX[] = "smart-curtain";
constexpr bool MQTT_TLS_INSECURE = false;
constexpr char MQTT_ROOT_CA_PEM[] = R"CERT(
-----BEGIN CERTIFICATE-----
PASTE_THE_BROKER_ROOT_CA_HERE
-----END CERTIFICATE-----
)CERT";

constexpr char WIFI_SSID[] = "YOUR_WIFI_NAME";
constexpr char WIFI_PASSWORD[] = "YOUR_WIFI_PASSWORD";
