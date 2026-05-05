# Lap Timer Board
Goal is to have all lap timer be stateless
- they have a esp_mac that can be used to identify them

They will all have a tf-luna plugged in to trigger packet send

Only a single board will have timer and button plugged in

so our loop would look like:

0. if reset detected, 
- drive reset_disp gpio high and return

1. if lap detected (buttton for now but will use tf-luna data later) or if gpio button is pressed
- drive count_disp high
- broadcast lora packet of (id) and crc




Pseudocode:




#include <esp_mac.h>

uint32_t deviceId;

void setup() {
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  // Use last 3 bytes — the unique part. Pack into a uint32_t.
  deviceId = ((uint32_t)mac[3] << 16) | ((uint32_t)mac[4] << 8) | mac[5];
  Serial.printf("Device ID: %06X\n", deviceId);
}
