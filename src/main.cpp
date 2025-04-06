#include <Adafruit_Sensor.h>
#include <WiFi.h>
#include <Arduino_MQTT_Client.h>
#include <ThingsBoard.h>
#include <DHT20.h> // Use the DHT20 library
#include <Wire.h>  // I2C library for DHT20
#include <ArduinoOTA.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <DHT.h>
// Configuration
#define LED_PIN 5
#define DHTPIN 4
#define DHTTYPE DHT22
constexpr char WIFI_SSID[] = "Wifi11";
constexpr char WIFI_PASSWORD[] = "123456789";
constexpr char TOKEN[] = "69o42qcjtwnwdwnsgomy";
constexpr char THINGSBOARD_SERVER[] = "app.coreiot.io";
constexpr uint16_t THINGSBOARD_PORT = 1883U;
constexpr uint32_t MAX_MESSAGE_SIZE = 1024U;
constexpr uint32_t SERIAL_DEBUG_BAUD = 115200U;
constexpr int16_t telemetrySendInterval = 5000U; // 10 seconds
uint32_t previousDataSend;
// Wi-Fi and ThingsBoard clients
WiFiClient wifiClient;
Arduino_MQTT_Client mqttClient(wifiClient);
ThingsBoard tb(mqttClient, MAX_MESSAGE_SIZE);
// DHT20 sensor
DHT20 dht20; // Create an instance of the DHT20 sensor
DHT dht(DHTPIN, DHTTYPE);  
// Shared attributes
constexpr char BLINKING_INTERVAL_ATTR[] = "blinkingInterval";
constexpr char LED_STATE_ATTR[] = "ledState";
volatile bool attributesChanged = false;
volatile bool ledState = false;
constexpr std::array<const char *, 2U> SHARED_ATTRIBUTES_LIST = {
  LED_STATE_ATTR,
  BLINKING_INTERVAL_ATTR};


// RPC Callback for LED control
RPC_Response getLedSwitchState(const RPC_Data &data)
{
  Serial.println("Received Switch state");
  bool newState = data;
  Serial.println(newState);
  return RPC_Response("getLedSwitchState", newState);
}
RPC_Response setLedSwitchState(const RPC_Data &data)
{
  Serial.println("Received Switch state");
  bool newState = data;
  Serial.print("Switch state change: ");
  Serial.println(newState);
  digitalWrite(LED_PIN, newState);
  attributesChanged = true;
  return RPC_Response("setLedSwitchValue", newState);
}
RPC_Response validationSuccess(const RPC_Data &data) {
  Serial.println("Received RPC message:");
  bool isValid = data["isValid"];
  Serial.print("Validation Success :");
  Serial.println(isValid);
  return RPC_Response("validationSuccess", true);
}// Register both RPC callbacks
RPC_Response validationFailure(const RPC_Data &data) {
  Serial.println("Received RPC message:");
  bool isValid = data["isValid"];
  Serial.print("Validation Failure :");
  Serial.println(isValid);
  return RPC_Response("validationFailure", false);
}
const std::array<RPC_Callback, 4U> callbacks = {
    RPC_Callback{"getLedSwitchValue", setLedSwitchState},
    RPC_Callback{"setLedSwitchValue", setLedSwitchState},
    RPC_Callback{"validationSuccess", validationSuccess},
    RPC_Callback{"validationFailure", validationFailure}
};

// Shared attribute callback
RPC_Response processSharedAttributes(const Shared_Attribute_Data &data)
{
  for (auto it = data.begin(); it != data.end(); ++it)
  {
    if (strcmp(it->key().c_str(), BLINKING_INTERVAL_ATTR) == 0)
    {
      const uint16_t new_interval = it->value().as<uint16_t>();
      if (new_interval >= 10U && new_interval <= 60000U)
      {
        Serial.print("Blinking interval is set to: ");
        Serial.println(new_interval);
      }
    }
    else if (strcmp(it->key().c_str(), LED_STATE_ATTR) == 0)
    {
      ledState = it->value().as<bool>();
      digitalWrite(LED_PIN, ledState);
      Serial.print("LED state is set to: ");
      Serial.println(ledState);
      return RPC_Response("getLedSwitchValue", ledState);
    }
  }
  attributesChanged = true;
}

const Shared_Attribute_Callback attributes_callback(&processSharedAttributes, SHARED_ATTRIBUTES_LIST.cbegin(), SHARED_ATTRIBUTES_LIST.cend());
const Attribute_Request_Callback attributes_shared_request_callback(&processSharedAttributes, SHARED_ATTRIBUTES_LIST.cbegin(), SHARED_ATTRIBUTES_LIST.cend());

// Wi-Fi initialization
void InitWifi()
{
  Serial.print("Connecting to AP ... ");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to AP");
}

bool reconnect()
{
  if (WiFi.status() != WL_CONNECTED)
  {
    InitWifi();
  }
  return WiFi.status() == WL_CONNECTED;
}

// FreeRTOS Tasks
void wifiTask(void *pvParameters)
{
  while (1)
  {
    if (!reconnect())
    {
      vTaskDelay(pdMS_TO_TICKS(1000)); // Wait for 1 second before retrying
    }
    vTaskDelay(pdMS_TO_TICKS(1000)); // Check every second
  }
}

void thingsboardTask(void *pvParameters)
{
  while (1)
  {
    if (!tb.connected())
    {
      Serial.print("Connecting to: ");
      Serial.print(THINGSBOARD_SERVER);
      Serial.print(" with token ");
      Serial.println(TOKEN);
      if (!tb.connect(THINGSBOARD_SERVER, TOKEN, THINGSBOARD_PORT))
      {
        Serial.println("Failed to connect.");
        vTaskDelay(pdMS_TO_TICKS(5000)); // Wait for 5 seconds before retrying
        continue;
      }
      tb.sendAttributeData("macAddress", WiFi.macAddress().c_str());
      Serial.println("Subscribing for RPC...");
      if (!tb.RPC_Subscribe(callbacks.cbegin(), callbacks.end()))
      {
        Serial.println("Failed to subscribe for RPC");
        vTaskDelay(pdMS_TO_TICKS(5000)); // Wait for 5 seconds before retrying
        continue;
      }
      if (!tb.Shared_Attributes_Subscribe(attributes_callback))
      {
        Serial.println("Failed to subscribe for shared attribute updates");
        vTaskDelay(pdMS_TO_TICKS(5000)); // Wait for 5 seconds before retrying
        continue;
      }
      Serial.println("Subscribe done");
      if (!tb.Shared_Attributes_Request(attributes_shared_request_callback))
      {
        Serial.println("Failed to request for shared attributes");
        vTaskDelay(pdMS_TO_TICKS(5000)); // Wait for 5 seconds before retrying
        continue;
      }
    }
    tb.loop();
    vTaskDelay(pdMS_TO_TICKS(100)); // Run every 100ms
  }
}

void telemetryTask(void *pvParameters)
{
  while (1)
  {
    if (millis() - previousDataSend > telemetrySendInterval)
    {
      // previousDataSend = millis();
      // int status = dht20.read(); // Read data from DHT20
      // if (status == DHT20_OK)
      // {
      //   float temperature = dht20.getTemperature(); // Get temperature
      //   float humidity = dht20.getHumidity();       // Get humidity
      //   Serial.print("Temperature: ");
      //   Serial.print(temperature);
      //   Serial.print("°C  |  Humidity: ");
      //   Serial.print(humidity);
      //   Serial.println("%");
      //   tb.sendTelemetryData("temperature", temperature);
      //   tb.sendTelemetryData("humidity", humidity);
      // }
      // else
      // {
      //   Serial.println("Failed to read from DHT20 sensor!");
      // }
      previousDataSend = millis();
      
      float temperature = dht.readTemperature(); // Đọc nhiệt độ (°C)
      float humidity = dht.readHumidity();       // Đọc độ ẩm (%)

      if (!isnan(temperature) && !isnan(humidity)) // Kiểm tra dữ liệu hợp lệ
      {
        Serial.print("Temperature: ");
        Serial.print(temperature);
        Serial.print("°C  |  Humidity: ");
        Serial.print(humidity);
        Serial.println("%");

        StaticJsonDocument<256> jsonBuffer;
        jsonBuffer["temperature"] = temperature;
        jsonBuffer["humidity"] = humidity;

        // Send telemetry data
        tb.sendTelemetryJson(jsonBuffer, measureJson(jsonBuffer));
      }
      else
      {
        Serial.println("⚠️ Error: Failed to read from DHT22 sensor!");
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1000)); // Run every second
  }
}

void attributesTask(void *pvParameters)
{
  while (1)
  {
    if (attributesChanged)
    {
      attributesChanged = false;
      tb.sendAttributeData(LED_STATE_ATTR, digitalRead(LED_PIN));
    }
    tb.sendAttributeData("rssi", WiFi.RSSI());
    tb.sendAttributeData("channel", WiFi.channel());
    tb.sendAttributeData("bssid", WiFi.BSSIDstr().c_str());
    tb.sendAttributeData("localIp", WiFi.localIP().toString().c_str());
    tb.sendAttributeData("ssid", WiFi.SSID().c_str());
    vTaskDelay(pdMS_TO_TICKS(1000)); // Run every second
  }
}

void setup()
{
  Serial.begin(SERIAL_DEBUG_BAUD);
  pinMode(LED_PIN, OUTPUT);
  delay(1000);

  // Initialize I2C and DHT20 sensor
  Wire.begin();
  dht20.begin();
  dht.begin();
  // Create FreeRTOS tasks
  xTaskCreate(wifiTask, "WifiTask", 4096, NULL, 1, NULL);
  xTaskCreate(thingsboardTask, "ThingsBoardTask", 8192, NULL, 2, NULL);
  xTaskCreate(telemetryTask, "TelemetryTask", 4096, NULL, 3, NULL);
  xTaskCreate(attributesTask, "AttributesTask", 4096, NULL, 4, NULL);
}

void loop()
{
  // Empty loop as everything is handled by FreeRTOS tasks
}