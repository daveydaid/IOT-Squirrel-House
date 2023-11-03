/*#####################################################################################################
#
# Title:    ESP32_SquirrelCam
# Version:  0.9
# Auth:     DMcD
# Date:     03/11/2023
#
# Future:
#   - Consider use of mqtt.subscribe() and mqtt.update() to trigger callbacks and react to incoming msgs
#   - Consider ringLight controllable via MQTT?
#####################################################################################################*/
/*#####################################################################################################
# PROJECT INCLUDES
#####################################################################################################*/
// #include <math.h>
#include "esp_camera.h"
#include <WiFi.h>
#include <MQTTPubSubClient.h>
#include <esp_wifi.h>
#include "sd_read_write.h"
#include <FastLED.h>
#include <driver/rtc_io.h>
#include <Wire.h>
#include "Adafruit_LC709203F.h"

/*#####################################################################################################
# PREPROCESSOR DIRECTIVES
#####################################################################################################*/
/* App GATE flags for debug: */
#define ENABLE_RINGLIGHT 1
#define ENABLE_CAMERA 1
#define ENABLE_WIFI 1
#define ENABLE_MQTT 1
#define ENABLE_SDCARD 0
#define ENABLE_BATTREAD 1
#define ENABLE_DEEPSLEEPMODE 0 // 0 for debug and test

#define LED_BUILTIN  2

#define CAMERA_MODEL_ESP32S3_EYE // Has PSRAM
#define BUTTON_PIN_BITMASK 0x4000 // 2^14 (since using EXT0 this is not currently needed since this is for EXT1)
#define INPUT_BUTTON 47 // test for toggling deep-sleep
#define PIR_SENSE 14  // Also used for EXT0

#include "camera_pins.h"

#define NUM_LEDS 12 // Single AZDelivery board
#define WS2812_ONBOARD_PIN 48
#define MAX_BRIGHTNESS 175  // (0 is brightest, 255 is off)

#define I2C_SDA 1  // Blue
#define I2C_SCL 21  // Purple

TwoWire I2CBAT = TwoWire(0); /* Note: Camera is using TwoWire(1)? */

CRGB leds[NUM_LEDS];

const char* ssid     = "********************";
const char* password = "********************";

WiFiClient client;
MQTTPubSubClient mqtt;

Adafruit_LC709203F lipo;

RTC_DATA_ATTR int bootCount = 0;

int sleepButtonState = 0;

void fadeOnRingLight(void);
uint8_t checkTime(void);
uint8_t checkBattTime(void);
void print_wakeup_reason(void);
void cameraInit(void);
void startCameraServer();

/*#####################################################################################################
#####################################        SETUP        ############################################
#####################################################################################################*/
void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println("");
  Serial.println("Serial ON and Setup starting...");

  //Print the wakeup reason for ESP32
  print_wakeup_reason();

#if (ENABLE_BATTREAD)
  I2CBAT.begin(I2C_SDA, I2C_SCL, 100000);

  Serial.print("Connecting to Adafruit LC709203F Battery Monitor...");
  while(!lipo.begin(&I2CBAT)) { //default address is 0x36
    delay(500);
    Serial.print(".");
  }
  Serial.println(F("Found LC709203F"));
  Serial.print("Version: 0x"); Serial.println(lipo.getICversion(), HEX);

  lipo.setThermistorB(3950);
  Serial.print("Thermistor B = "); Serial.println(lipo.getThermistorB());

  lipo.setPackSize(LC709203F_APA_3000MAH); // Approx. battery size
#endif

  // Configure WS2812 output
  FastLED.addLeds<WS2812, WS2812_ONBOARD_PIN, GRB>(leds, NUM_LEDS);

  // Set EXT0 wakeup pin source:
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_14,1); 
  /* TODD: (***** CONNECT THE PIR DATA LINE TO PIN 14 WHEN READY TO TEST *****) */

  // Initialise the pins:
  pinMode(INPUT_BUTTON, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);

#if (ENABLE_CAMERA)
  cameraInit();
#endif
#if (ENABLE_SDCARD)
  sdmmcInit();
  removeDir(SD_MMC, "/video");
  createDir(SD_MMC, "/video");
#endif

#if (ENABLE_WIFI)  
  WiFi.begin(ssid, password);

  Serial.print("Connecting to home WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" connected!");
#endif

#if (ENABLE_MQTT)
 Serial.print("Connecting to HA host...");
    while (!client.connect("192.168.0.51", 1883)) {
        Serial.print(".");
        delay(1000);
    }
    Serial.println(" connected!");

    // initialize mqtt client
    mqtt.begin(client);

    Serial.print("Connecting to MQTT broker...");
    while (!mqtt.connect("squirrelCam", "********************", "********************")) {
        Serial.print(".");
        delay(1000);
    }
    Serial.println(" connected!");

    // Don't care about this section atm - don't want to RX any msgs
    // // subscribe topic and callback which is called when any message has come
    // mqtt.subscribe("/outside/squirrel/power", [](const String& payload, const size_t size) {
    //     Serial.print("/outside/squirrel/power ");
    //     Serial.println(payload);
    // });

    // Send "device is on" message to HA
    mqtt.publish("/outside/squirrel/power", "on");
    delay(100);

    // Send battery level on startup
    mqtt.publish("/outside/squirrel/battlevel", String(int(lipo.cellPercent() + 0.5)));
    delay(100);    
#endif

#if (ENABLE_CAMERA)
  startCameraServer();

  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");
#endif

#if (ENABLE_RINGLIGHT)
  fadeOnRingLight();
#endif

  Serial.println("End of setup()");
}

/*#####################################################################################################
###################################        MAIN LOOP        ##########################################
#####################################################################################################*/
void loop() {
  // Just toggle the LED and RGBLED to let me know when device is alive...
  //static int i = 0;
  // FastLED.clear();
  // FastLED.show();
  // leds[i] = CRGB(255, 255, 255);        //IO48_RGB WHITE
  // FastLED.delay(100);
  // i++;
  // if (i >= 12)
  // {
  //   i = 0;
  // }

  // int val = 0;                  // TEMP test only for PIR
  // val = digitalRead(PIR_SENSE); // TEMP test only for PIR
  // Serial.print("PIR value: ");  // TEMP test only for PIR
  // Serial.println(val);          // TEMP test only for PIR
  // delay(100);                   // TEMP test only for PIR

#if (ENABLE_BATTREAD)
  Serial.print("Batt_Voltage: ");
  Serial.print(lipo.cellVoltage(), 3);
  Serial.print("\t");
  Serial.print("Batt_Percent: ");
  Serial.println(lipo.cellPercent(), 1);

  if (checkBattTime() == 1)
  {  
    // Send current battery level
    mqtt.publish("/outside/squirrel/battlevel", String(int(lipo.cellPercent() + 0.5)));
    delay(100);
  }

  //Battery rounded down:
  // int rounded = int(lipo.cellPercent() + 0.5);
  // Serial.print("\t");
  // Serial.print("Batt_Percent Rounded: ");
  // Serial.println(rounded);

  // Serial.print("\t");                              // No temp resistor added yet
  // Serial.print("Batt_Temp:");
  // Serial.println(lipo.getCellTemperature(), 1);
  delay(2000);  // dont query too often!
#endif

#if (ENABLE_DEEPSLEEPMODE)
  if ((checkTime() == 1) && (digitalRead(PIR_SENSE) == LOW)) //... aka no movement detected
  {
    //Go to sleep now...
    Serial.println("Conditions met to enter DEEP SLEEP!");
    delay(100);

    // Send "device is turning off" message to HA
    mqtt.publish("/outside/squirrel/power", "off");
    delay(100);

    // Ensure all WiFI comms are stopped (https://esp32.com/viewtopic.php?t=2512)
    esp_wifi_disconnect();
    delay(100);
    esp_wifi_stop();
    delay(100);
    esp_wifi_deinit();
    delay(100);

    // Ensure all pins to default state (see this post: https://electronics.stackexchange.com/questions/530151/esp32-wroom32-consuming-77-%C2%B5a-much-too-high-in-deep-sleep)
    gpio_reset_pin(GPIO_NUM_0);
    gpio_reset_pin(GPIO_NUM_1);
    gpio_reset_pin(GPIO_NUM_2);
    gpio_reset_pin(GPIO_NUM_3);
    gpio_reset_pin(GPIO_NUM_4);
    gpio_reset_pin(GPIO_NUM_5);
    gpio_reset_pin(GPIO_NUM_6);
    gpio_reset_pin(GPIO_NUM_7);
    gpio_reset_pin(GPIO_NUM_8);
    gpio_reset_pin(GPIO_NUM_9);
    gpio_reset_pin(GPIO_NUM_10);
    gpio_reset_pin(GPIO_NUM_11);
    gpio_reset_pin(GPIO_NUM_12);
    gpio_reset_pin(GPIO_NUM_13);
    // 14 needed for the EXT0
    gpio_reset_pin(GPIO_NUM_15);
    gpio_reset_pin(GPIO_NUM_16);
    gpio_reset_pin(GPIO_NUM_17);
    gpio_reset_pin(GPIO_NUM_18);
    gpio_reset_pin(GPIO_NUM_19);
    gpio_reset_pin(GPIO_NUM_20);
    gpio_reset_pin(GPIO_NUM_21);

    //rtc_gpio_isolate(GPIO_NUM_48);
    // gpio_reset_pin(GPIO_NUM_48);
    // clears all roof LEDs to off
    FastLED.clear();
    FastLED.show();

    esp_deep_sleep_start();
    Serial.println("This will never be printed...");
  }
#endif
}

/*#####################################################################################################
# Function:     fadeOnRingLight
#
# Inputs:       n/a
#
# Retuns:       n/a
#####################################################################################################*/
void fadeOnRingLight(){

  Serial.println("Enabling ring light...");
  FastLED.clear();

  for (int i=255; i>=MAX_BRIGHTNESS; i--)
  {
    // Serial.print("Current brightness: ");
    // Serial.println(i);
    fill_solid(leds, NUM_LEDS, CRGB::White);
    fadeLightBy(leds, NUM_LEDS, i);
    FastLED.show();
    delay(100);
  }
}

/*#####################################################################################################
# Function:     checkTime
#
# Inputs:       n/a
#
# Retuns:       n/a
#####################################################################################################*/
uint8_t checkTime(){
  long allowedTime = 120000; // 120 secs
//  long allowedTime = 30000; // 30 secs
  static unsigned long last_millis = 0;
 
  if((long)(millis() - last_millis) >= allowedTime)
  {
    Serial.println("***** Timer elapsed! *****");
    last_millis = millis();
    return 1;
  }
  else
  {
    //Serial.println("Timer on going...");
    return 0;
  }
}

/*#####################################################################################################
# Function:     checkBattTime
#
# Inputs:       n/a
#
# Retuns:       n/a
#####################################################################################################*/
uint8_t checkBattTime(){
  long allowedTime = 60000; // 1 min
  static unsigned long last_millis = 0;
 
  if((long)(millis() - last_millis) >= allowedTime)
  {
    Serial.println("***** Battery Check Time! *****");
    last_millis = millis();
    return 1;
  }
  else
  {
    //Serial.println("Timer on going...");
    return 0;
  }
}

/*#####################################################################################################
# Function:     print_wakeup_reason
#
# Inputs:       n/a
#
# Retuns:       n/a
#####################################################################################################*/
void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }
}

/*#####################################################################################################
# Function:     cameraInit
#
# Inputs:       n/a
#
# Retuns:       n/a
#####################################################################################################*/
void cameraInit(void){
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  // config.xclk_freq_hz = 20000000;
  config.xclk_freq_hz = 8000000;      // TEMP fix for the frame rate drop (shielding issue)
  config.frame_size = FRAMESIZE_UXGA;
  config.pixel_format = PIXFORMAT_JPEG; // for streaming
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 1;
  
  // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
  // for larger pre-allocated frame buffer.
  if(psramFound()){
    Serial.println("PSRAM detected");
    config.jpeg_quality = 10;
    config.fb_count = 2;
    config.grab_mode = CAMERA_GRAB_LATEST;
  } else {
    Serial.println("PSRAM not detected...");
    // Limit the frame size when PSRAM is not available
    config.frame_size = FRAMESIZE_SVGA;
    config.fb_location = CAMERA_FB_IN_DRAM;
  }

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  // initial sensors are flipped vertically and colors are a bit saturated
  s->set_vflip(s, 1); // flip it back
  s->set_brightness(s, 1); // up the brightness just a bit
  s->set_saturation(s, 0); // lower the saturation
}
