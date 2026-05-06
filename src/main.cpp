#include <Arduino.h>
#include <RadioLib.h>
#include <SPI.h>
#include <esp_mac.h>

// function definitions:
void sendLapPacket();

/*~~~~~Pin Mapping~~~~~*/

// TF Luna 
#define TF_RX_TX 6 // SDA/RXD
#define TF_TX_RX 7 // SCL/TXD
#define TF_CFG 3
#define TF_MODE 2

// Buttons (input)
#define RESET_BTN 42
#define COUNT_UP_BTN 41 // <-- use this for triggering laps for now

// Display outputs
#define RESET_SIG 4
#define COUNT_SIG 5


/*~~~~~Hardware Definitions~~~~~*/

// These are hardware specific to the Heltec WiFi LoRa 32 V4
// Cite: https://resource.heltec.cn/download/WiFi_LoRa_32_V4/Schematic/HTIT-WB32LAF_V4.3.pdf
#define PRG_BUTTON 0
#define LORA_NSS_PIN 8
#define LORA_SCK_PIN 9
#define LORA_MOSI_PIN 10
#define LORA_MISO_PIN 11
#define LORA_RST_PIN 12
#define LORA_BUSY_PIN 13
#define LORA_DIO1_PIN 14


/*~~~~~Radio Configuration~~~~~*/

// Initialize SX1262 radio
// Make a custom SPI device because *of course* Heltec didn't use the default SPI pins
SPIClass spi(FSPI);
SPISettings spiSettings(2000000, MSBFIRST, SPI_MODE0); // Defaults, works fine
SX1262 radio = new Module(LORA_NSS_PIN, LORA_DIO1_PIN, LORA_RST_PIN, LORA_BUSY_PIN, spi, spiSettings);

// make sure basestation is on the same channel
#define LORA_FREQ 915.0
#define LORA_BW 125.0
#define LORA_SF 9

/*~~~~~Global Variables~~~~~*/

uint32_t deviceId = 0;
volatile bool countFlag = false;

// button debounce
unsigned long lastPressTime = 0;
const unsigned long debounceDelay = 50;

/*~~~~~Interrupts~~~~~*/

// This function should be called when button is pressed 
//  It is placed in RAM to avoid Flash usage errors
void IRAM_ATTR countISR(void)
{
  countFlag = true;
}

/*~~~~~Helper Functions~~~~~*/

void error_message(const char *message, int16_t state)
{
  Serial.printf("ERROR!!! %s with error code %d\n", message, state);
  while (true)
    ; // loop forever
}


void setup() {
  Serial.begin(115200);
  delay(2000);
  // button flag setup
  pinMode(COUNT_UP_BTN, INPUT_PULLUP); // wired to pullup for easier testing
  attachInterrupt(digitalPinToInterrupt(COUNT_UP_BTN), countISR, FALLING);

  // Other pins (just initialize, don't use yet)
  pinMode(RESET_BTN, INPUT_PULLUP);

  pinMode(RESET_SIG, OUTPUT);
  pinMode(COUNT_SIG, OUTPUT);

  pinMode(TF_RX_TX, INPUT);
  pinMode(TF_TX_RX, INPUT);
  pinMode(TF_CFG, OUTPUT);
  pinMode(TF_MODE, OUTPUT);

  // get device id 
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  deviceId = ((uint32_t)mac[3] << 16) |
             ((uint32_t)mac[4] << 8) |
             mac[5];

  Serial.printf("Device ID: %06X\n", deviceId);



  /* Initialize Lora */
  // Set up SPI with our specific pins
  spi.begin(LORA_SCK_PIN, LORA_MISO_PIN, LORA_MOSI_PIN, LORA_NSS_PIN);

  Serial.print("Initializing radio...");
  int16_t state = radio.begin(LORA_FREQ, LORA_BW, LORA_SF, 5, 0x12, 15, 8);
  if (state != RADIOLIB_ERR_NONE)
  {
    error_message("Radio initialization failed", state);
  }

  state = radio.setCurrentLimit(140.0);
  if (state != RADIOLIB_ERR_NONE)
  {
    error_message("Current limit initialization failed", state);
  }

  state = radio.setDio2AsRfSwitch(true);
  if (state != RADIOLIB_ERR_NONE)
  {
    error_message("DIO2 RF switch initialization failed", state);
  }

  state = radio.explicitHeader();
  if (state != RADIOLIB_ERR_NONE)
  {
    error_message("Explicit header initialization failed", state);
  }

  state = radio.setCRC(2);
  if (state != RADIOLIB_ERR_NONE)
  {
    error_message("CRC initialization failed", state);
  }

  Serial.println("Complete!");
  Serial.println("Ready. Press COUNT_UP_BTN to send packet.");
}

void loop() {
  if (countFlag)
  {
    countFlag = false;

    unsigned long now = millis();

    // check for debounce
    if (now - lastPressTime > debounceDelay)
    {
      lastPressTime = now;

      sendLapPacket();
    }
  }
}


void sendLapPacket() {
  uint8_t data[3];

  data[0] = (deviceId >> 16) & 0xFF;
  data[1] = (deviceId >> 8) & 0xFF;
  data[2] = deviceId & 0xFF;

  int16_t state = radio.transmit(data, 3);

  if (state == RADIOLIB_ERR_NONE)
  {
    Serial.println("Packet sent!");
  }
  else
  {
    Serial.printf("Transmit failed, code %d\n", state);
  }
}