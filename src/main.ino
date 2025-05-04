#include <ArduinoJson.h>
#include <base64.h>
#include <ESP32Servo.h>
#include <HX711.h>
#include <libb64/cdecode.h>
#include <models/event.h>
#include <mqttClient.h>
#include <secrets.h>
#include <UUID.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>


void handleDeltaCallBack(char* topic, byte* payload, unsigned int length);
String THING_NAME = "CAMERA_DEMO";

const int TOPIC_LENGTH = 2;
String deltaTopic = "$aws/things/" + THING_NAME + "/shadow/name/config/update/delta";
String getSubscribeTopic = "$aws/things/" + THING_NAME + "/shadow/name/config/get/accepted";
String getPublishTopic = "$aws/things/" + THING_NAME + "/shadow/name/config/get";

String subscribeTopics[TOPIC_LENGTH]= {
  deltaTopic,
  getSubscribeTopic
};

WiFiClientSecure net;
PubSubClient client(AWS_IOT_ENDPOINT, 8883, handleDeltaCallBack, net);
MqttClient mqttClient(client, THING_NAME);

Servo cameraServo;
HX711 pressureDetector;
UUID uuid;

const int LOOP_DELAY = 2000;
const int MQTT_RECONNECT_CD = 6000;
int mqttReconnectCountDown = 0;

bool isSubscribed = false;

const int LED_PIN = 16;
const int PIR_PIN = 15;
const int US_TRIGGER_PIN = 2;
const int US_ECHO_PIN = 0;
const int BUZZER_PIN = 17;
const int SERVO_PIN = 14;
const int LOAD_DT = 25;
const int LOAD_SCK = 33;

const int LED_COUNT_DOWN = 6000;
int ledCountDown = 0;

const int EVENT_DETECTION_COUNTDOWN = 3000;
int visitorDetectCountdown = 0;
int doorPressureDetectCountdown = 0;

int distance_threshold = 50;
int alarm_max_duration = 1500;
int pressure_threshold = 2100;

void setup()
{
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  pinMode(PIR_PIN, INPUT);
  pinMode(US_TRIGGER_PIN, OUTPUT);
  pinMode(US_ECHO_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  cameraServo.attach(SERVO_PIN, 500, 2400);
  pressureDetector.begin(LOAD_DT, LOAD_SCK);

  connectToInternet();

  if (WiFi.status() == WL_CONNECTED) {
    setCertificates(net);
    bool mqttStatus = mqttClient.connect(5000);
    Serial.print("MQTT connect status: ");
    Serial.println(mqttStatus);
  }
}

void loop()
{
  resetCountDown();
  mqttClient.keepAlive();
  if (WiFi.status() == WL_CONNECTED) {
    if (mqttClient.isConnected()) {
      if (!isSubscribed) {
        mqttClient.subsribeTopics(subscribeTopics, TOPIC_LENGTH);
        delay(1500);
        mqttClient.publishGetMessage(getPublishTopic);
        isSubscribed = true;
      }
    }
    else {
      Serial.println("MQTT connection failed.");
      isSubscribed = false;
      if (mqttReconnectCountDown <= 0) {
        mqttClient.connect();
        mqttReconnectCountDown = MQTT_RECONNECT_CD;
      }
      else {
        mqttReconnectCountDown -= LOOP_DELAY;
      }
    }
  }
  ProcessDetecting();
  mqttClient.keepAlive();
  delay(LOOP_DELAY);
}

void connectToInternet() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.println("Connecting to Wi-Fi...");
  WiFi.waitForConnectResult();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Connected to Wi-Fi");
  }
}

void connectAWS()
{
  bool isMqttConnected = mqttClient.isConnected();
  if (isMqttConnected) {
    mqttClient.keepAlive();
    return;
  }

  if (isMqttConnected && mqttReconnectCountDown > 0) {
    mqttReconnectCountDown -= 1;
    return;
  }

  Serial.println("AWS IoT Connection Failed! Retrying...");
  mqttClient.keepAlive();
  mqttReconnectCountDown = 5;
}

void handleDeltaCallBack(char* topic, byte* payload, unsigned int length) {
  JsonDocument doc;
  JsonDocument delta;
  deserializeJson(doc, payload, length);
  Serial.print("Receive message from topic: ");
  Serial.println(topic);
  Serial.print("Delta: ");

  if (deltaTopic.equals(topic)) {
    delta = doc["state"];
  }

  if (getSubscribeTopic.equals(topic)) {
    delta = doc["state"]["delta"];
  }
  
  serializeJson(delta, Serial);
  
  Serial.println();

  if (delta["light"]["turned_on"].is<bool>()) {
    digitalWrite(LED_PIN, delta["light"]["turned_on"]);
  }

  if (delta["camera"]["angle"].is<double>()) {
    cameraServo.write(delta["camera"]["angle"]);
  }

  if (delta["distance_threshold"].is<double>()) {
    distance_threshold = delta["distance_threshold"];
  }

  if (delta["alarm_max_duration"].is<int>()) {
    alarm_max_duration = delta["alarm_max_duration"];
  }

  if (delta["pressure_threshold"].is<double>()) {
    pressure_threshold = delta["pressure_threshold"];
  }
}

void resetCountDown()
{
  if (ledCountDown >= LED_COUNT_DOWN) {
    ledCountDown = 0;
  };

  if (doorPressureDetectCountdown >= EVENT_DETECTION_COUNTDOWN) {
    doorPressureDetectCountdown = 0;
  };

  if (visitorDetectCountdown >= EVENT_DETECTION_COUNTDOWN) {
    visitorDetectCountdown = 0;
  };
}

int readCameraAngle() {
  int cameraAngle = cameraServo.read();
  if (cameraAngle < 0) return 0;

  if (cameraAngle > 180) return 180;

  return cameraAngle;
}

double calculateDistance() {
  digitalWrite(US_TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(US_TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(US_TRIGGER_PIN, LOW);

  // Echo Pin goes Hight after trigger Pin send signals, when meet the sound wave or timeout, goes low
  // Measure HIGH time mean time for sound wave go and turn back in micro sec
  double duration = pulseIn(US_ECHO_PIN, HIGH);

  return duration * 0.034 / 2; // In cm
}

void ProcessDetecting()
{
  JsonDocument event;
  bool eventDetected = false;
  int pirState = digitalRead(PIR_PIN);
  // If detects motion, measure the distance
  if (pirState == HIGH)
  {
    double detectedDistance = calculateDistance();
    // if distance is withIn 50 cm, keep turn on the light
    if (detectedDistance <= distance_threshold) 
    {
      digitalWrite(LED_PIN, HIGH);
      ledCountDown = 0;
      event["event_type"] = (int)EventType::VISITOR_DETECTED;
      event["distance"] = detectedDistance;
      eventDetected = visitorDetectCountdown == 0 ? true : false;
    }
    else {
      Serial.print("Distance measured: ");
      Serial.println(detectedDistance);
    }
  }

  if (pressureDetector.wait_ready_timeout(500)) {
    double detectedPressure = pressureDetector.get_units() * 2.381; // in grams
    if (detectedPressure > pressure_threshold) {     
      digitalWrite(LED_PIN, HIGH);
      tone(BUZZER_PIN, 222, alarm_max_duration);
      Serial.print("Alarm fired. Pressure = ");
      Serial.println(detectedPressure);

      event["event_type"] = (int)EventType::DOOR_PRESSURE_DETECTED;
      event["pressure"] = detectedPressure;
      eventDetected = doorPressureDetectCountdown == 0 ? true : false;
    }
  }

  if (eventDetected) {
    uuid.generate();
    const char* eventImageName = uuid.toCharArray();
    // send event data to MQTT
    event["camera_angle"] = readCameraAngle();
    event["device_name"] = THING_NAME;
    event["light_turned_on"] = digitalRead(LED_PIN);
    event["image_prefix"] = "event-image/" + THING_NAME + "/" + (int)EventType::DOOR_PRESSURE_DETECTED + "/" + eventImageName + ".jpg";

    sendEventImage(eventImageName);
    mqttClient.publishEventData(event);
  }

  ledCountDown += LOOP_DELAY;
  doorPressureDetectCountdown += LOOP_DELAY;
  visitorDetectCountdown += LOOP_DELAY;
}

void sendEventImage(const char* imageName) {
  char * output = (char *) malloc(3833);
  int actualLength = base64_decode_chars(imageBase64, 5112, output);
  mqttClient.publishEventImage(imageName, (byte*)output, 3833);
  free(output);
}