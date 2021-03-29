#include "structure.h"

/*
int TERMINALS[] = {32, 35, 34, 39};
int LED[] = {13, 12, 14, 27};
//int LED[] = {25, 26, 21, 19};
int BUZZER = 2;
int ALARM_LED = 4; 
int MOV_TRIG = 16;
int MOV_ECHO = 17;
int DISPLAY1 = 22;
int DISPLAY2 = 23;
int DISPLAY3 = 5;
int DISPLAY4 = 18;
int DISPLAY5 = 19;
int DISPLAY6 = 21;
const char* ssid     = "extender";
const char* password = "freeRTOS";
const char* clientid = "DevESP32%MQTT_HOME%";


int TERMINALS[] = {23, 22, 21, 15};
int LED[] = {12, 32, 13, 14};
int BUZZER = 18;
int ALARM_LED = 19; 
int MOV_TRIG = 5;
int MOV_ECHO = 4;
const char* ssid     = "Vodafone-51555344";
const char* password = "uejst7ccxbkkivm";
const char* clientid = "DevESP32%MQTT_BUZZ%";
*/


const char* mqtt_server = "broker.hivemq.com";
char topic_id[30]; //copia del topic in arrivo in subscription
char topic_payload[60]; //copia del payload in arrivo in subscription
boolean ACK = 0;

const char* topic_alarm_sound = "KfZ91%%7BM@/ALLARM/SOUND";
const char* topic_alarm_mode_on = "KfZ91%%7BM@/ALLARM/MODE/ON";
const char* topic_alarm_mode_off = "KfZ91%%7BM@/ALLARM/MODE/OFF";
const char* topic_alarm_sound_text = "KfZ91%%7BM@/ALLARM/SOUND/TEXT";
const char* topic_alarm_received = "KfZ91%%7BM@/ALLARM/RECEIVED";
const char* topic_motion_detection_code = "KfZ91%%7BM@/PERSONAL/CODE";
const char* topic_open_slaves = "KfZ91%%7BM@/SLAVES/OPEN";

int curr_distance;
int last_distance;
bool first_read = false;

WiFiClient espClient;
PubSubClient client(mqtt_server, 1883, callback, espClient);
