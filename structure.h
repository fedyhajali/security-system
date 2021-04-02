#ifndef STRUCTURE_H
#define STRUCTURE_H

// slaves terminal
#define SLAVES 4

// alarm mode
#define DISABLED 0   
#define ENABLED 1

// alarm sound 
#define OFF 0   
#define ON 1

// slave mode => definisce se la finestra/porta (slave) è aperta o chiusa
#define CLOSE 0
#define OPEN 1

extern int TERMINALS[]; // input array dei pin dei pulsanti che simulano gli slaves
extern int LED[];   // output array dei led che indicano lo stato degli slaves (aperti/chiusi)
extern int BUZZER;  // pin buzzer di allarme
extern int ALARM_LED; // pin led di allarme

// Load Wi-Fi library
#include <WiFi.h>
#include <PubSubClient.h>

// Network credentials
extern const char* ssid;
extern const char* password;
extern const char* mqtt_server;
extern const char* clientid;
extern char topic_id[30];   // copia del topic in arrivo
extern char topic_payload[60];   // copia del payload in arrivo
extern boolean ACK;   // true se c'è un messaggio di ack da mandare


extern const char* topic_alarm_mode_on; // Pulsante attivazione allarme 
extern const char* topic_alarm_mode_off;  // Pulsante disattivazione allarme
extern const char* topic_alarm_sound;  // Pulsante disattivazione suono allarme
extern const char* topic_alarm_sound_text;  // Tab di notifica stato suono allarme 
extern const char* topic_alarm_received;  // Tab di ricezione, ACK dei comandi inviati da APP
extern const char* topic_open_slaves; // Tab contatore slave aperti
extern const char* topic_motion_detection_code; // Pulsante disabilitazione timer di inserimento codice

// DISPLAY-1602
#include <LiquidCrystal.h>
extern int DISPLAY1;
extern int DISPLAY2;
extern int DISPLAY3;
extern int DISPLAY4;
extern int DISPLAY5;
extern int DISPLAY6;

// HC-SR04 Distance sensor 
#include <Ultrasonic.h>
#define MAX_DISTANCE 5
extern int MOV_TRIG; // Output pin HC-SR04 sensore rilevamento movimenti
extern int MOV_ECHO; // Input pin HC-SR04 sensore rilevamento movimenti
extern int curr_distance;
extern int last_distance;
extern bool first_read;

#include "time.h"

extern const char* ntpServer;
extern const long  gmtOffset_sec;
extern const int   daylightOffset_sec;

//#include <IRremote.h>
#define PASS_SIZE 5
//extern int IR_Recv; //IR Receiver Pin
extern bool ins_password; 

extern void callback(char* topic, byte* payload, unsigned int length);

extern WiFiClient espClient;
extern PubSubClient client;

// Shared Resource
struct home_state {
  bool slave_state[SLAVES];  // slave state: OPEN/CLOSE
  bool alarm_mode;   // mode: DISABLED/ENABLED
  bool alarm_sound;    // ON/OFF per far suonare l'allarme 
  int open_slaves;    // contatore finestre aperte
};

#endif  
