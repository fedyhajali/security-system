#ifndef STRUCTURE_H
#define STRUCTURE_H

// MISRA C R. 20.1 OK
#include <WiFi.h>
#include <PubSubClient.h>
#include <LiquidCrystal.h>
#include <Ultrasonic.h>
#include "time.h"

// slaves terminal
#define SLAVES 3

// alarm mode
#define DISABLED 0
#define ENABLED 1

// alarm sound
#define OFF 0
#define ON 1

// slave state
#define CLOSE 0
#define OPEN 1

#define MAX_DISTANCE 5

#define PASS_SIZE 5

// MISRA C R. 8.11 OK
extern int TERMINALS[SLAVES]; // input array of button pins that simulate slaves
extern int LED[SLAVES];       // output array of LEDs indicating the status of the slaves (open / closed)
extern int BUZZER;            // alarm buzzer pin
extern int ALARMLED;          // alarm LED pin
extern int ALARMBUTTON;       // button that enables / disables alarm
extern int rgbREDLED;         // led RGB alarm mode
extern int rgbGREENLED;       // led RGB alarm mode
extern int rgBLED;            // led RGB alarm mode

// Network credentials
extern const char *ssid;        // Network Identifier
extern const char *password;    // Network Code
extern const char *mqtt_server; // MQTT Public Server
extern const char *clientid;    // MQTT client ID

// MQTT Topics
extern char topic_id[30];                       // copy of the incoming topic
extern char topic_payload[60];                  // copy of the incoming payload
extern boolean ACK;                             // true if there is an ACK message to send
extern const char *topic_alarm_mode_on;         // Alarm activation button
extern const char *topic_alarm_mode_off;        // Alarm deactivation button
extern const char *topic_alarm_sound;           // Alarm sound deactivation button
extern const char *topic_alarm_sound_text;      // Alarm sound status notification tab
extern const char *topic_alarm_received;        // Reception tab, ACK of commands sent by the Application
extern const char *topic_open_slaves;           // Open slave counter tab
extern const char *topic_motion_detection_code; // Button to enter code and disable timer

// DISPLAY-1602 pins
extern int DISPLAY1;
extern int DISPLAY2;
extern int DISPLAY3;
extern int DISPLAY4;
extern int DISPLAY5;
extern int DISPLAY6;

// HC-SR04 Motion Detection sensor
extern int MOV_TRIG;      // Output pin HC-SR04
extern int MOV_ECHO;      // Input pin HC-SR04
extern int curr_distance; // Current distance
extern int last_distance; // Last distance read
extern bool first_read;   // First read

// NTP - Network Time Protocol for Date and Time
extern const char *ntpServer;
extern const long gmtOffset_sec;
extern const int daylightOffset_sec;

// Remote control and Infrared Receiver
extern int IR_Recv;
extern bool insert_password;
extern char real_password[PASS_SIZE];
extern char try_password[PASS_SIZE];
extern bool exit_while;

// SemaphoreGive to Alarm from Timer Callback
extern bool give_fromTimer;

// Load Wi-Fi and PubSub library
extern void callback(char *topic, byte *payload, unsigned int length);
extern WiFiClient espClient;
extern PubSubClient client;

// Shared Resource
struct home_state
{
  bool slave_state[SLAVES]; // slave state: OPEN/CLOSE
  bool alarm_mode;          // mode: DISABLED/ENABLED
  bool alarm_sound;         // ON/OFF per far suonare l'allarme
  int open_slaves;          // counter 
};

#endif
