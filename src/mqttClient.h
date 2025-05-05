#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <secrets.h>
#include <WiFiClientSecure.h>

class MqttClient {
    private:
        PubSubClient client;
        String thingName;

        void shadowMessageHandler(char* topic, byte* payload, unsigned int length) {
            Serial.print("Receive message from topic: ");
            Serial.println(topic);

            JsonDocument doc;
            deserializeJson(doc, payload, length);
        }
    public:
        MqttClient(PubSubClient& pubSubClient, String thingName) {
            pubSubClient.setBufferSize(std::numeric_limits<uint16_t>::max());
            pubSubClient.setKeepAlive(60);
            this->client = pubSubClient;
            this->thingName = thingName;
        };

        bool connect(int timeout = 2000) {
            Serial.print("Connecting to AWS IoT endpoint: ");
            Serial.print(SECRET AWS_IOT_ENDPOINT);
            Serial.println("...");
                
            client.connect(this->thingName.c_str());
            while(!client.connected() && timeout > 0) {
                timeout-= 200;
                delay(200);
            }
            return client.connected();
        };

        void keepAlive() {
            client.loop();
        };

        bool isConnected() {
            return client.connected();
        }

        void subsribeTopics(String* topics, int length) {

            Serial.println("Start subscribe to topics...");
            for (int i = 0; i < length; i++) {
                const char* topicName = topics[i].c_str();
                Serial.println(topicName);
                client.subscribe(topicName);
            }

            Serial.println("Complete subscribe to topics.");
        };

        void publishGetMessage(String getShadowTopic) {
            Serial.println("Sent message to sync device shadow.");
            client.publish(getShadowTopic.c_str(), "{}");
        }

        void syncDevideShadow(JsonDocument& delta, String shadowUpdateTopic) {
            String jsonData;
            serializeJson(delta, jsonData);
            client.publish(shadowUpdateTopic.c_str(), jsonData.c_str());
        }

        void publishEventData(JsonDocument& event) {
            String eventTopic = this->thingName + "/events/" + (event["event_type"].as<int>());
            String jsonData;
            serializeJson(event, jsonData);
            Serial.print("Send event data to topic: ");
            Serial.println(eventTopic);
            client.publish(eventTopic.c_str(), jsonData.c_str());
        }

        void publishEventImage(const char* imageName, const byte* payload, unsigned int payloadSize) {
            String eventTopic = this->thingName + "/event-image/" + (int)EventType::DOOR_PRESSURE_DETECTED + "/" + imageName;
            Serial.print("Send event data to topic: ");
            Serial.println(eventTopic);
            client.publish(eventTopic.c_str(), payload, payloadSize, false);
        }
};

static void setCertificates(WiFiClientSecure& net) {
    net.setCACert(AWS_CERT_CA);
    net.setCertificate(AWS_CERT_CRT);
    net.setPrivateKey(AWS_CERT_PRIVATE);
}