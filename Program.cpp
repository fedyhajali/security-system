#include "structure.h"

/*
int TERMINALS[] = {35, 34, 39};
int LED[] = {12, 14, 27};
int BUZZER = 2;
int ALARMLED = 4;
int rgbREDLED = 13;
int rgbGREENLED = 26;
int rgBLED = 25;
int ALARMBUTTON = 32;
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

int TERMINALS[] = {37, 36, 33, 23};
int LED[] = {13, 7, 15, 12};
int BUZZER = 30;
int ALARMLED = 31; 
int MOV_TRIG = 29;
int MOV_ECHO = 26;
int DISPLAY1 = 5;
int DISPLAY2 = 6;
int DISPLAY3 = 11;
int DISPLAY4 = 8;
int DISPLAY5 = 25;
int DISPLAY6 = 24;
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

const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 0;
const int   daylightOffset_sec = 7200;

//int IR_Recv = 9;
//int IR_Recv = 15;

char real_password[PASS_SIZE] = "0000";
char try_password[PASS_SIZE];
bool insert_password = false;
bool exit_while = false;

WiFiClient espClient;
PubSubClient client(mqtt_server, 1883, callback, espClient);
