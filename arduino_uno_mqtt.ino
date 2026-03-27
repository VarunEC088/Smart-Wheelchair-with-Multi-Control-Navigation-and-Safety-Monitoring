#include <SPI.h>
#include <Ethernet.h>
#include <PubSubClient.h>

byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };

const char* broker   = "broker.hivemq.com";
const char* pubTopic = "arduino/uno/data";
const char* subTopic = "arduino/uno/data";

EthernetClient ethClient;
PubSubClient   mqtt(ethClient);

// ── Callback: just print the received message ──
void callback(char* topic, byte* payload, unsigned int length) {
  String msg = "";
  for (int i = 0; i < length; i++) msg += (char)payload[i];
  Serial.print("Received [");
  Serial.print(topic);
  Serial.print("]: ");
  Serial.println(msg);
}

void reconnect() {
  while (!mqtt.connected()) {
    Serial.println("Connecting to broker...");
    if (mqtt.connect("Arduino_things")) {
      Serial.println("Connected!");
      mqtt.subscribe(subTopic);
    } else {
      delay(3000);
    }
  }
}

void setup() {
  Serial.begin(9600);
  Ethernet.begin(mac);
  delay(1500);
  mqtt.setServer(broker, 1883);
  mqtt.setCallback(callback);
}

void loop() {
  if (!mqtt.connected()) reconnect();
  mqtt.loop();

  // Publish A0 reading every 5 seconds
  static unsigned long lastMsg = 0;
  if (millis() - lastMsg > 5000) {
    lastMsg = millis();
    char payload[32];
    snprintf(payload, sizeof(payload), "sensor:%d", analogRead(A0));
    mqtt.publish(pubTopic, payload);
    Serial.print("Published: ");
    Serial.println(payload);
  }
}
