#include <WiFi.h>
#include <PubSubClient.h>

// WiFi
char ssid[] = "hu";
char pass[] = "12345678";
WiFiClient espClient;
PubSubClient client(espClient);

// MQTT Broker
const char* mqtt_server = "test.mosquitto.org";
const int mqtt_port = 1883;
const char* mqtt_accX = "gyroscope/accX";
const char* mqtt_accY = "gyroscope/accY";
const char* mqtt_accZ = "gyroscope/accZ";

// String Parser
String value_accX;
String value_accY;
String value_accZ;
char input[30];
char* delimiter = ";";
char* output[4];
int outputSize;

void setup() {
  Serial.begin(115200);

  Serial.println("Connecting WiFi: " + String(ssid));
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi Connected");
  Serial.println(WiFi.localIP());

  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  reconnect();
}

void loop() {
  Serial.println(data);
  if (data.length() > 0) {
    data.toCharArray(input, sizeof(input));
    splitString(input, delimiter, output, &outputSize);
    for (int i = 0; i < outputSize; i++) {
      switch (i) {
        case 0:
          value_accX = output[0];
          break;
        case 1:
          value_accY = output[1];
          break;
        case 2:
          value_accZ = output[2];
          break;
        default:
          // Do Nothing
          break;
      }
    }

    char sendValue[15];

    // MPU6050 Processing
    value_accX.toCharArray(sendValue, sizeof(sendValue));
    client.publish(mqtt_accX, sendValue);
    memset(sendValue, 0, sizeof(sendValue));

    value_accY.toCharArray(sendValue, sizeof(sendValue));
    client.publish(mqtt_accY, sendValue);
    memset(sendValue, 0, sizeof(sendValue));

    value_accZ.toCharArray(sendValue, sizeof(sendValue));
    client.publish(mqtt_accZ, sendValue);
    memset(sendValue, 0, sizeof(sendValue));

    data = "";
  }

  // MQTT
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
}

void splitString(char* input, char* delimiter, char* output[], int* outputSize) {
  char* token = strtok(input, delimiter);
  int index = 0;

  while (token != NULL) {
    output[index] = token;
    index++;
    token = strtok(NULL, delimiter);
  }

  *outputSize = index;
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Pesan yang diterima [");
  Serial.print(topic);
  Serial.print("] ");

  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }

  Serial.println();
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Menghubungkan ke MQTT broker...");

    String clientId = "MQTT_Gyroscope";
    clientId += String(random(0xffff), HEX);

    if (client.connect(clientId.c_str())) {
      Serial.println("Terhubung");
    } else {
      Serial.print("Gagal, kode kesalahan = ");
      Serial.print(client.state());
      Serial.println(" Coba lagi dalam 5 detik");
      delay(5000);
    }
  }
}
