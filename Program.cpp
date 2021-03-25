#include "structure.h"


int TERMINALS[] = {13, 12, 14, 27};
int LED[] = {23, 22, 21, 19};
int BUZZER = 2;
int ALARM_LED = 4; 
int MOV_TRIG = 5;
int MOV_ECHO = 17;

const char* ssid     = "extender";
const char* password = "freeRTOS";
const char* mqtt_server = "broker.hivemq.com";
const char* clientid = "DevESP32%MQTT_HOME%";
char topic_id[30]; //copia del topic in arrivo in subscription
char topic_payload[60]; //copia del payload in arrivo in subscription
boolean ACK = 0;

const char* topic_alarm = "KfZ91%%7BM@/ALLARM";
const char* topic_alarm_mode_on = "KfZ91%%7BM@/ALLARM/MODE/ON";
const char* topic_alarm_mode_off = "KfZ91%%7BM@/ALLARM/MODE/OFF";
const char* topic_alarm_sound = "KfZ91%%7BM@/ALLARM/SOUND";
const char* topic_alarm_received = "KfZ91%%7BM@/ALLARM/RECEIVED";
const char* topic_open_slaves = "KfZ91%%7BM@/SLAVES/OPEN";

//unsigned int distance;

WiFiClient espClient;
PubSubClient client(mqtt_server, 1883, callback, espClient);
