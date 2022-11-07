#ifndef STRUCTURE_H
#define STRUCTURE_H

// MISRA C R. 20.1 OK
#include <WiFi.h>
#include <PubSubClient.h>
#include <LiquidCrystal.h>
#include <Ultrasonic.h>
#include "time.h"

// slaves terminal
#define SLAVES 2

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
extern const char *basetopic;                   // Topics BaseName
extern const char *topic_mode_status;           // Alarm Mode
extern const char *topic_mode_on;               // Alarm Mode activation button
extern const char *topic_mode_off;              // Alarm Mode deactivation button
extern const char *topic_sound_status;          // Alarm Sound
extern const char *topic_sound_off;             // Alarm Sound deactivation button
extern const char *topic_notification;          // Alarm Notification
extern const char *topic_open;                  // Alarm Open terminals Counter 
extern const char *topic_code;                  // Alarm Code
extern const char *topic_general;               // Alarm General Informations
extern const char *topic_distance;              // Alarm Distance Informations

extern const char *topic_alarm_mode_on;         // Alarm activation button
extern const char *topic_alarm_mode_off;        // Alarm deactivation button
extern const char *topic_alarm_sound;           // Alarm sound deactivation button
extern const char *topic_alarm_sound_text;      // Alarm sound status notification tab
extern const char *topic_alarm_received;        // Reception tab, ACK of commands sent by the Application
extern const char *topic_open_slaves;           // Open slave counter tab
extern const char *topic_motion_detection_code; // Button to enter code and disable timer


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

// Mutex for password 
extern bool insert_password;

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
  bool alarm_sound;         // sound: ON/OFF
  int open_slaves;          // counter 
};

#endif
