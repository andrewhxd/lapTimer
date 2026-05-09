#include <Arduino.h>
#include <RadioLib.h>
#include <SPI.h>
#include <esp_mac.h>
#include <U8g2lib.h>
#include <TFMPlus.h>

// function definitions:
void sendLapPacket();

/*~~~~~Pin Mapping~~~~~*/

// TF Luna 
#define TF_SDA_RX 6 // SDA/RXD
#define TF_SCL_TX 7 // SCL/TXD
// #define TF_CFG 3
// #define TF_MODE 2 <- GPIO Pin 2 Conflicts with GC1109 FEM

// Buttons (input)
#define RESET_BTN 42
#define COUNT_UP_BTN 41 // <-- use this for triggering laps for now

// Gym Timer Display outputs
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

// GC1109 front-end enable (CSD)
#define FEM_EN 2

/*~~~~~TF Luna Setup~~~~~*/

TFMPlus tfmP;
HardwareSerial TFSerial(1);

#define LAP_TRIGGER_CM 50
#define LAP_HYSTERSIS_CM 80 // be ready to trigger once cart exceed 80cm away

static bool armed = true; // waiting for car, set to false while car is passing by

int16_t dist = 0;
int16_t flux = 0;
int16_t temp = 0;


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


/*~~~~~Screen Configuration~~~~~*/

#define VEXT_CTRL 36
#define OLED_RESET 21
#define OLED_SDA 17
#define OLED_SCL 18
U8G2_SSD1306_128X64_NONAME_F_SW_I2C display(U8G2_R0, /* clock=*/OLED_SCL, /* data=*/OLED_SDA, /* reset=*/OLED_RESET); // All Boards without Reset of the Display

// Screen drawing locations
#define X_MAX 128
#define Y_MAX 64
static uint8_t iteration_count = 0;
static uint32_t x_coor = 0;
static uint32_t y_coor = 10;
static int8_t x_rate = 4;
static int8_t y_rate = 4;

// String to draw on screen
static char display_str[80] = {0};

/*~~~~~Global Variables~~~~~*/

uint32_t deviceId = 0;
volatile bool countFlag = false;

static uint32_t laps_sent = 0;

// button debounce
unsigned long lastPressTime = 0;
const unsigned long debounceDelay = 300;

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

// Draw text horizontally centered on the screen at the given baseline y.
// Uses whatever font is currently set on `display`.
void draw_centered(int16_t y, const char *text)
{
  int16_t w = display.getStrWidth(text);
  int16_t x = (X_MAX - w) / 2;
  if (x < 0)
    x = 0; // string wider than screen — pin to left
  display.drawStr(x, y, text);
}

void logo()
{
  snprintf(display_str, sizeof(display_str), "Initializing");
  display.clearBuffer();
  draw_centered(25, display_str);

  snprintf(display_str, sizeof(display_str), "Detector");
  draw_centered(50, display_str);

  display.sendBuffer();
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

  // TF Luna in UART mode, tx and rx are configured elsewhere
  // pinMode(TF_MODE, OUTPUT);
  // pinMode(TF_CFG, OUTPUT);
  

  // get device id from factory-burned eFuse base MAC (3 bytes)
  // IEEE 802 format is first 3 OUI, last 3 vendor unique
  uint8_t mac[6];
  esp_efuse_mac_get_default(mac);
  deviceId = ((uint32_t)mac[3] << 16) |
             ((uint32_t)mac[4] << 8) |
             mac[5];

  Serial.printf("Device ID: %06X\n", deviceId);


  /* Initialize TF Luna */
  TFSerial.begin(115200, SERIAL_8N1, TF_SDA_RX, TF_SCL_TX);
  delay(200);

  tfmP.begin(&TFSerial);
  Serial.println("TFMPlus initialized");

  /* Initialize Lora */
  // Set up SPI with our specific pins
  pinMode(FEM_EN, OUTPUT);
  digitalWrite(FEM_EN, HIGH); // GC1109 Permanently Enabled
  spi.begin(LORA_SCK_PIN, LORA_MISO_PIN, LORA_MOSI_PIN, LORA_NSS_PIN);

  Serial.print("Initializing radio...");
  int16_t state = radio.begin(LORA_FREQ, LORA_BW, LORA_SF, 5, 0x12, 22, 8);
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

  // Initialize the display
  pinMode(VEXT_CTRL, OUTPUT);
  digitalWrite(VEXT_CTRL, LOW);

  display.begin();
  display.setContrast(200);
  display.setFont(u8g2_font_ncenB10_tr);

  // draw startup logo
  logo();
  delay(3000);
  display.clear();
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

  tfmP.getData(dist, flux, temp);
  // check if armed and dist > 0 as 0 is default when too far away
  if (armed && dist > 0 && dist <= LAP_TRIGGER_CM)
  {
    // car is passing by
    armed = false;
    Serial.printf("Lap! dist=%d\n", dist);
    sendLapPacket();
  } 
  else if (!armed && dist >= LAP_HYSTERSIS_CM) // once car passes by and distance goes above hystersis threshold
  {
    armed = true;
    Serial.println("Re-armed");
  }

  /* Live display update: previous lap + current elapsed */
  static uint32_t last_draw_ms = 0;
  uint32_t now = millis();
  if (now - last_draw_ms >= 100) // ~10 Hz refresh
  {
    last_draw_ms = now;

    display.clearBuffer();

    // print alive message
    snprintf(display_str, sizeof(display_str), "Lap Timer");
    draw_centered(25, display_str);

    snprintf(display_str, sizeof(display_str), "Sent: %u", laps_sent);
    draw_centered(50, display_str);
    display.sendBuffer();
  }
}

void sendLapPacket() {
  laps_sent++;

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