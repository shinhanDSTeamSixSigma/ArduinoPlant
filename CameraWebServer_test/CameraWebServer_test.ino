#include "esp_camera.h"
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include "soc/soc.h" //disable brownour problems
#include "soc/rtc_cntl_reg.h"  //disable brownour problems
#include <HardwareSerial.h>
//
// WARNING!!! PSRAM IC required for UXGA resolution and high JPEG quality
//            Ensure ESP32 Wrover Module or other board with PSRAM is selected
//            Partial images will be transmitted if image exceeds buffer size
//
//            You must select partition scheme from the board menu that has at least 3MB APP space.
//            Face Recognition is DISABLED for ESP32 and ESP32-S2, because it takes up from 15 
//            seconds to process single frame. Face Detection is ENABLED if PSRAM is enabled as well

// ===================
// Select camera model
// ===================
//#define CAMERA_MODEL_WROVER_KIT // Has PSRAM
//#define CAMERA_MODEL_ESP_EYE // Has PSRAM
//#define CAMERA_MODEL_ESP32S3_EYE // Has PSRAM
//#define CAMERA_MODEL_M5STACK_PSRAM // Has PSRAM
//#define CAMERA_MODEL_M5STACK_V2_PSRAM // M5Camera version B Has PSRAM
//#define CAMERA_MODEL_M5STACK_WIDE // Has PSRAM
//#define CAMERA_MODEL_M5STACK_ESP32CAM // No PSRAM
//#define CAMERA_MODEL_M5STACK_UNITCAM // No PSRAM
#define CAMERA_MODEL_AI_THINKER // Has PSRAM
//#define CAMERA_MODEL_TTGO_T_JOURNAL // No PSRAM
//#define CAMERA_MODEL_XIAO_ESP32S3 // Has PSRAM
// ** Espressif Internal Boards **
//#define CAMERA_MODEL_ESP32_CAM_BOARD
//#define CAMERA_MODEL_ESP32S2_CAM_BOARD
//#define CAMERA_MODEL_ESP32S3_CAM_LCD
//#define CAMERA_MODEL_DFRobot_FireBeetle2_ESP32S3 // Has PSRAM
//#define CAMERA_MODEL_DFRobot_Romeo_ESP32S3 // Has PSRAM
#include "camera_pins.h"

// ===========================
// Enter your WiFi credentials
// ===========================
const char* ssid = "3 강의장_2G";
const char* password = "fz202303";
// REPLACE with your Domain name and URL path or IP address with path
const char* serverName = "http://192.168.0.135:8090/fromArduino/sensor";
char jsonOutput[128];
void startCameraServer();
void setupLedFlash(int pin);
int solid_humid;
int lumen;
int humidity;
int thomer;
HardwareSerial mySerial(2); //3개의 시리얼 중 2번 채널을 사용
bool isWatered = false;
void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();
  mySerial.begin(115200, SERIAL_8N1, 12, 13); //추가로 사용할 시리얼. RX:12 / TX:13번 핀 사용

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
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  //config.xclk_freq_hz = 20000000; //20 fps
  config.xclk_freq_hz = 5000000; //5 fps
  config.frame_size = FRAMESIZE_UXGA;
  config.pixel_format = PIXFORMAT_JPEG; // for streaming
  //config.pixel_format = PIXFORMAT_RGB565; // for face detection/recognition
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 1;
  
  // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
  //                      for larger pre-allocated frame buffer.
  if(config.pixel_format == PIXFORMAT_JPEG){
    if(psramFound()){
      config.jpeg_quality = 10;
      config.fb_count = 2;
      config.grab_mode = CAMERA_GRAB_LATEST;
    } else {
      // Limit the frame size when PSRAM is not available
      config.frame_size = FRAMESIZE_SVGA;
      config.fb_location = CAMERA_FB_IN_DRAM;
    }
  } else {
    // Best option for face detection/recognition
    config.frame_size = FRAMESIZE_240X240;
#if CONFIG_IDF_TARGET_ESP32S3
    config.fb_count = 2;
#endif
  }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  // initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1); // flip it back
    s->set_brightness(s, 1); // up the brightness just a bit
    s->set_saturation(s, -2); // lower the saturation
  }
  // drop down frame size for higher initial frame rate
  if(config.pixel_format == PIXFORMAT_JPEG){
    s->set_framesize(s, FRAMESIZE_QVGA);
  }

#if defined(CAMERA_MODEL_M5STACK_WIDE) || defined(CAMERA_MODEL_M5STACK_ESP32CAM)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif

#if defined(CAMERA_MODEL_ESP32S3_EYE)
  s->set_vflip(s, 1);
#endif

// Setup LED FLash if LED pin is defined in camera_pins.h
#if defined(LED_GPIO_NUM)
  setupLedFlash(LED_GPIO_NUM);
#endif

  //set wifi static ip
  IPAddress local_ip(192,168,0,200);
  IPAddress gateway(192,168,0,1);
  IPAddress subnet(255,255,255,0);
  if(!WiFi.config(local_ip,gateway,subnet)){
    Serial.println("STA Failed to config");
  }

  WiFi.begin(ssid, password);
  WiFi.setSleep(false);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");

  startCameraServer();

  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.print("  ");
  Serial.print(WiFi.gatewayIP());
  Serial.print("  ");
  Serial.print(WiFi.subnetMask());
  Serial.println("' to connect");
  solid_humid = 0;
  lumen = 0;
  humidity = 0;
  thomer = 0;
}
void watering(){
  if(isWatered){
    isWatered = false;
    if(mySerial.available() > 0){
      mySerial.write(1);
      mySerial.println(".");
    }
  }
}

void loop() {
  if(WiFi.status() == WL_CONNECTED){
    WiFiClientSecure *client = new WiFiClientSecure;
    client -> setInsecure(); //don't use SSL
    HTTPClient https;
    https.begin(serverName);
    //https.addHeader("Content-Type",  "application/x-www-form-urlencoded");   //Specify content-type header,  Json형식의 타입이다.
    https.addHeader("Content-Type",  "application/json");
    const size_t CAPACITY = JSON_OBJECT_SIZE(1);
    StaticJsonDocument<CAPACITY> doc;

    JsonObject object = doc.to<JsonObject>();

    
    solid_humid = 0;
    lumen = 0;
    humidity = 0;
    thomer = 0;
    if(mySerial.available() > 0){
      /*String command = mySerial.readStringUntil('.');
      thomer = command.toInt();
      command = mySerial.readStringUntil('.');
      humidity = command.toInt();
      command = mySerial.readStringUntil('.');
      lumen = command.toInt();
      command = mySerial.readStringUntil('_');
      solid_humid = command.toInt();*/
      thomer = mySerial.parseInt();
      humidity = mySerial.parseInt();
      lumen = mySerial.parseInt();
      solid_humid = mySerial.parseInt();

      watering();
    }

    object["crop_no"] = "2";
    object["thomer"] = String(thomer);
    object["humidity"] = String(humidity);
    object["lumen"] = String(lumen);
    object["solid_humid"] = String(solid_humid);
    serializeJson(doc,jsonOutput);

    //String httpRequestData = "crop_no="+String(1)+"&thomer="+String(50);
    Serial.println(String(jsonOutput));
    int httpResponseCode = https.POST(String(jsonOutput));
    if(httpResponseCode >0){
      String response = https.getString();
      Serial.println(httpResponseCode);
      Serial.println(response);

    }else{
      Serial.print("Error on sending Post: ");
      Serial.println(httpResponseCode);
    }
    https.end();
    

  }
  //1시간 단위로
  //delay(3600000);
  //테스트용 1분 단위
  delay(60000);
  
}
