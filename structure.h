#ifndef STRUCTURE_H
#define STRUCTURE_H

// slaves terminal
#define SLAVES 4

// system mode  => definisce se l'allarme è acceso o spento
#define DISABLED 0   
#define ENABLED 1

// slave mode => definisce se la finestra/porta (slave) è aperta o chiusa
#define CLOSE 0
#define OPEN 1
 
// slaves pin
extern int TERMINALS[]; // input array dei pin dei pulsanti che simulano gli slaves
extern int LED[];   // output array dei led che indicano lo stato degli slaves (aperti/chiusi)
extern int BUZZER;  // pin del buzzer di allarme

#endif  
