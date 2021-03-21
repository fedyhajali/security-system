#include "structure.h"


int TERMINALS[] = {12, 14, 27};
int LED[] = {23, 22, 21};
int BUZZER = 2; 

const char* ssid     = "extender";
const char* password = "freeRTOS";
const char* mqtt_server = "broker.hivemq.com";
const char* clientid = "DevESP32%MQTT_HOME%";
const char* topic_allarm = "KfZ91%%7BM@/ALLARM";
const char* topic_allarm_on = "KfZ91%%7BM@/ALLARM/ON";
const char* topic_allarm_received = "KfZ91%%7BM@/ALLARM/RECEIVED";


WiFiClient espClient;
PubSubClient client(mqtt_server, 1883, callback, espClient);
