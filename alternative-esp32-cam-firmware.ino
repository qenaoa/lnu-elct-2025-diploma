#include <WiFi.h>
#include <PubSubClient.h>
#include "esp_camera.h"

// ==== Wi-Fi ====
const char* ssid = "SSID";
const char* password = "PASSWORD";

// ==== MQTT ====
const char* mqtt_server = "IP";
const int mqtt_port = 1883;
const char* mqtt_user = "USER";
const char* mqtt_pass = "PASSWORD";

WiFiClient espClient;
PubSubClient client(espClient);

// ==== Піни ESP32-CAM ====
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

#define RED_LED_GPIO      33
#define FLASH_LED_GPIO     4

#define DEVICE_NAME "espcam"
#define UPTIME_TOPIC "espcam/sensor/uptime/state"
#define RSSI_TOPIC   "espcam/sensor/wifi_signal/state"
#define RED_LED_TOPIC "espcam/switch/red_led/set"
#define FLASH_TOPIC   "espcam/light/flash_led/set"

unsigned long lastMsg = 0;
unsigned long startTime;

// ==== MJPEG сервер ====
#include <WebServer.h>
WebServer server(8080);

// ==== Функція стріму ====
void startCameraServer() {
  server.on("/", HTTP_GET, []() {
    server.send(200, "text/plain", "ESP32-CAM is up");
  });

  server.on("/stream", HTTP_GET, []() {
    WiFiClient client = server.client();

    String response = "HTTP/1.1 200 OK\r\n";
    response += "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n";
    server.sendContent(response);

    while (client.connected()) {
      camera_fb_t * fb = esp_camera_fb_get();
      if (!fb) continue;

      response = "--frame\r\n";
      response += "Content-Type: image/jpeg\r\n\r\n";
      server.sendContent(response);
      server.sendContent((const char *)fb->buf, fb->len);
      server.sendContent("\r\n");
      esp_camera_fb_return(fb);
      if (!client.connected()) break;
    }
  });

  server.begin();
}

// ==== Callbacks ====
void callback(char* topic, byte* payload, unsigned int length) {
  String command;
  for (int i = 0; i < length; i++) command += (char)payload[i];

  command.trim();
  command.toLowerCase();

  if (String(topic) == RED_LED_TOPIC) {
    digitalWrite(RED_LED_GPIO, command == "on" ? HIGH : LOW);
  }

  if (String(topic) == FLASH_TOPIC) {
    digitalWrite(FLASH_LED_GPIO, command == "on" ? HIGH : LOW);
  }
}

// ==== Підключення MQTT ====
void reconnect() {
  while (!client.connected()) {
    Serial.print("Connecting to MQTT... ");
    if (client.connect(DEVICE_NAME, mqtt_user, mqtt_pass)) {
      Serial.println("connected");
      client.subscribe(RED_LED_TOPIC);
      client.subscribe(FLASH_TOPIC);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" retry in 5 sec");
      delay(5000);
    }
  }
}

// ==== Камера + Wi-Fi ====
void setup() {
  Serial.begin(115200);

  pinMode(RED_LED_GPIO, OUTPUT);
  pinMode(FLASH_LED_GPIO, OUTPUT);
  digitalWrite(RED_LED_GPIO, HIGH);  // on_boot

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
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
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  if(psramFound()){
    config.frame_size = FRAMESIZE_SVGA;  // 800x600
    config.jpeg_quality = 20;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_VGA;
    config.jpeg_quality = 20;
    config.fb_count = 1;
  }

  // Камера init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed 0x%x", err);
    return;
  }

  // Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500); Serial.print(".");
  }
  Serial.println("\nWiFi connected");

  startCameraServer();

  // MQTT
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  startTime = millis();
}

// ==== Основний цикл ====
void loop() {
  server.handleClient();

  if (!client.connected()) reconnect();
  client.loop();

  unsigned long now = millis();
  if (now - lastMsg > 10000) { // кожні 10 сек
    lastMsg = now;

    long uptime = (now - startTime) / 1000;
    int rssi = WiFi.RSSI();

    client.publish(UPTIME_TOPIC, String(uptime).c_str(), true);
    client.publish(RSSI_TOPIC, String(rssi).c_str(), true);
  }
}
