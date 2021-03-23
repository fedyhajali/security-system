#include "structure.h"


int TERMINALS[] = {13, 12, 14, 27};
int LED[] = {23, 22, 21, 19};
int BUZZER = 2;
int ALLARM_LED = 4; 

const char* ssid     = "Vodafone-A80665306";
const char* password = "Frosinone28";
const char* mqtt_server = "broker.hivemq.com";
const char* clientid = "DevESP32%MQTT_HOME%";
const char* topic_allarm = "KfZ91%%7BM@/ALLARM";
const char* topic_allarm_mode = "KfZ91%%7BM@/ALLARM/MODE";
const char* topic_allarm_sound = "KfZ91%%7BM@/ALLARM/SOUND";
const char* topic_allarm_received = "KfZ91%%7BM@/ALLARM/RECEIVED";


WiFiClient espClient;
PubSubClient client(mqtt_server, 1883, callback, espClient);
