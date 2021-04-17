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

// slave mode => definisce se la finestra/porta (slave) è aperta o chiusa
#define CLOSE 0
#define OPEN 1

#define MAX_DISTANCE 5

#define PASS_SIZE 5

// MISRA.C REGOLA 8.11 
extern int TERMINALS[SLAVES]; // input array dei pin dei pulsanti che simulano gli slaves
extern int LED[SLAVES];   // output array dei led che indicano lo stato degli slaves (aperti/chiusi)
extern int BUZZER;  // pin buzzer di allarme
extern int ALARMLED; // pin led di allarme
extern int ALARMBUTTON; // pulsante abilita/disabilita allarme
extern int rgbREDLED; // led RGB alarm mode
extern int rgbGREENLED; // led RGB alarm mode
extern int rgBLED; // led RGB alarm mode

// Load Wi-Fi library
// Network credentials
extern const char* ssid;
extern const char* password;
extern const char* mqtt_server;
extern const char* clientid;
extern char topic_id[30];   // copia del topic in arrivo
extern char topic_payload[60];   // copia del payload in arrivo
extern boolean ACK;   // true se c'è un messaggio di ack da mandare

// MQTT Topics
extern const char* topic_alarm_mode_on; // Pulsante attivazione allarme 
extern const char* topic_alarm_mode_off;  // Pulsante disattivazione allarme
extern const char* topic_alarm_sound;  // Pulsante disattivazione suono allarme
extern const char* topic_alarm_sound_text;  // Tab di notifica stato suono allarme 
extern const char* topic_alarm_received;  // Tab di ricezione, ACK dei comandi inviati da APP
extern const char* topic_open_slaves; // Tab contatore slave aperti
extern const char* topic_motion_detection_code; // Pulsante disabilitazione timer di inserimento codice

// DISPLAY-1602

extern int DISPLAY1;
extern int DISPLAY2;
extern int DISPLAY3;
extern int DISPLAY4;
extern int DISPLAY5;
extern int DISPLAY6;

// HC-SR04 Distance sensor 
extern int MOV_TRIG; // Output pin HC-SR04 sensore rilevamento movimenti
extern int MOV_ECHO; // Input pin HC-SR04 sensore rilevamento movimenti
extern int curr_distance;
extern int last_distance;
extern bool first_read;


extern const char* ntpServer;
extern const long  gmtOffset_sec;
extern const int   daylightOffset_sec;


extern int IR_Recv;
extern bool insert_password; 
extern char real_password[PASS_SIZE];
extern char try_password[PASS_SIZE];
extern bool exit_while;

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
