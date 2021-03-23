#ifndef STRUCTURE_H
#define STRUCTURE_H

// slaves terminal
#define SLAVES 4

// system mode  => definisce se il sistema di allarme è acceso o spento
#define DISABLED 0   
#define ENABLED 1

// allarm mode  => definisce se l'allarme è stato attivato (suona) o è disattivato
#define OFF 0   
#define ON 1

// slave mode => definisce se la finestra/porta (slave) è aperta o chiusa
#define CLOSE 0
#define OPEN 1

// Load Wi-Fi library
#include <WiFi.h>
#include <PubSubClient.h>

// Network credentials
extern const char* ssid;
extern const char* password;
extern const char* mqtt_server;
extern const char* clientid;
extern const char* topic_allarm;
extern const char* topic_allarm_mode;
extern const char* topic_allarm_sound;
extern const char* topic_allarm_received;
//extern const char topic_slaves[SLAVES];


extern void callback(char* topic, byte* payload, unsigned int length);

extern WiFiClient espClient;
extern PubSubClient client;

// Shared Resource
struct home_state {
  bool slave_state[SLAVES];  // slave state: OPEN/CLOSE
  bool allarm_mode;   // mode: DISABLED/ENABLED
  bool allarm_sound;    // ON/OFF per far suonare l'allarme 
  int open_slaves;    // contatore finestre aperte
};

// slaves pin
extern int TERMINALS[]; // input array dei pin dei pulsanti che simulano gli slaves
extern int LED[];   // output array dei led che indicano lo stato degli slaves (aperti/chiusi)
extern int BUZZER;  // pin del buzzer di allarme
extern int ALLARM_LED; 

#endif  
