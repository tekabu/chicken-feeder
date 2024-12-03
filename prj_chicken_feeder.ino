#include <DHT.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <WiFiClientSecure.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define DHT22_TYPE DHT22
#define DHT22_PIN 2

DHT dht22(DHT22_PIN, DHT22_TYPE);

const char *ssid = "StarLink324G";
const char *password = "P@ng@p@t";

const char *mqtt_broker = "broker.emqx.io";
const char *mqtt_topic_dht = "3Eq2oLUyxvTKUHQlIHwYjQw0SCQ3o0hDx/dht";
const char *mqtt_topic_servo = "3Eq2oLUyxvTKUHQlIHwYjQw0SCQ3o0hDx/servo";
const char *mqtt_username = "emqx";
const char *mqtt_password = "public";
const int mqtt_port = 1883;

WiFiClient espClient;
PubSubClient mqtt_client(espClient);
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN 150
#define SERVOMAX 600
#define PWM_FREQ 50
#define PREFIX "S3"

unsigned long prevtime;

void setup() {
  Serial.begin(9600);

  pwm.begin();
  pwm.setPWMFreq(PWM_FREQ);

  WiFi.begin(ssid, password);
  connectToWiFi();

  mqtt_client.setServer(mqtt_broker, mqtt_port);
  mqtt_client.setKeepAlive(300);
  mqtt_client.setCallback(mqttCallback);
  connectToMQTT();

  Serial.println("ready...");
}

void loop() {
  connectToWiFi();
  
  if (!mqtt_client.connected()) {
    connectToMQTT();
  }
  mqtt_client.loop();

  if (millis() - prevtime >= 60000) {
    readDHT();
    prevtime = millis();
  }
}

void readDHT() {
  float temp22 = dht22.readTemperature();
  float hum22 = dht22.readHumidity();

  if (isnan(temp22) || isnan(hum22)) {
    temp22 = 0;
    hum22 = 0;
  }

  String dat = "[";
  dat += String(temp22, 1);
  dat += ",";
  dat += String(hum22, 1);
  dat += "]";

  Serial.print(temp22, 1);
  Serial.print(", ");
  Serial.print(hum22, 1);
  Serial.println();

  mqtt_client.publish(mqtt_topic_dht, dat.c_str());
}

void connectToWiFi() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.println("...");
    }
    Serial.println("\nConnected to WiFi");
  }

}

void connectToMQTT() {
  while (!mqtt_client.connected()) {
    String client_id = "esp32-client-fd-" + String(WiFi.macAddress());
    Serial.printf("Connecting to MQTT Broker as %s...\n", client_id.c_str());
    if (mqtt_client.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
      Serial.println("Connected to MQTT broker");
      mqtt_client.subscribe(mqtt_topic_servo);
    } else {
      Serial.print("Failed to connect to MQTT broker, rc=");
      Serial.print(mqtt_client.state());
      Serial.println(" Retrying in 5 seconds.");
      delay(5000);
    }
  }
}

void mqttCallback(char *topic, byte *payload, unsigned int length) {
  Serial.print("Message received on topic: ");
  Serial.println(topic);
  Serial.print("Message: ");
  String msg;
  for (unsigned int i = 0; i < length; i++) {
    msg += String((char) payload[i]);
  }
  msg.trim();
  if (msg.startsWith(PREFIX)) {
    int pulseLength = map(30, 0, 180, SERVOMIN, SERVOMAX);
    pwm.setPWM(0, 0, pulseLength);
    delay(300);
    pulseLength = map(90, 0, 180, SERVOMIN, SERVOMAX);
    pwm.setPWM(0, 0, pulseLength);
  }
}

void processCommand(String command) {
  // Parse the command
  if (command.startsWith("S")) {
    int commaIndex = command.indexOf(',');
    if (commaIndex > 0) {
      int channel = command.substring(1, commaIndex).toInt();
      int angle = command.substring(commaIndex + 1).toInt();

      // Validate channel and angle
      if (channel >= 0 && channel < 16 && angle >= 0 && angle <= 180) {
        int pulseLength = map(angle, 0, 180, SERVOMIN, SERVOMAX);
        pwm.setPWM(channel, 0, pulseLength);
        Serial.print("Channel ");
        Serial.print(channel);
        Serial.print(" set to ");
        Serial.print(angle);
        Serial.println(" degrees.");
      } else {
        Serial.println("Error: Invalid channel or angle.");
      }
    } else {
      Serial.println("Error: Malformed command.");
    }
  } else {
    Serial.println("Error: Command must start with 'S'.");
  }
}
