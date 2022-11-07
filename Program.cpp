#include "structure.h"

int TERMINALS[] = {35, 34};
int LED[] = {12, 14};
int BUZZER = 2;
int ALARMLED = 4;
int rgbREDLED = 13;
int rgbGREENLED = 26;
int rgBLED = 25;
int ALARMBUTTON = 32;
int MOV_TRIG = 16;
int MOV_ECHO = 17;

const char* ssid     = "Redmi 9C";
const char* password = "123456789F";
const char* clientid = "VSS%V01@esp32";
const char* mqtt_server = "broker.hivemq.com";

char topic_id[30]; 
char topic_payload[60];
boolean ACK = 0;

const char* basetopic = "KfZ91%%7BM@/";
const char* topic_mode_status = "KfZ91%%7BM@/status";
const char* topic_mode_on = "KfZ91%%7BM@/status/on";
const char* topic_mode_off = "KfZ91%%7BM@/status/off";
const char* topic_sound_status = "KfZ91%%7BM@/sound/status";
const char* topic_sound_off = "KfZ91%%7BM@/sound/disable";
const char* topic_notification = "KfZ91%%7BM@/notification";
const char* topic_open = "KfZ91%%7BM@/open";
const char* topic_code = "KfZ91%%7BM@/code";
const char* topic_general = "KfZ91%%7BM@/general";
const char* topic_distance = "KfZ91%%7BM@/distance";

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
bool insert_password = false;
bool give_fromTimer = false;

const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 0;
const int   daylightOffset_sec = 7200;

WiFiClient espClient;
PubSubClient client(mqtt_server, 1883, callback, espClient);
