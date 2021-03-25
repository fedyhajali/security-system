#include "structure.h"

// TaskHandles
TaskHandle_t Handle_TaskSlave[SLAVES];
TaskHandle_t Handle_TaskAlarm;
TaskHandle_t Handle_TaskMain;
TaskHandle_t Handle_TaskConnection;
TaskHandle_t Handle_TaskMovement;

// Semaphores
SemaphoreHandle_t mutex_home;
SemaphoreHandle_t mutex_alarm;
SemaphoreHandle_t mutex_movement;

//Task Definitions
void TaskSlave(void* pvParameters);
void TaskAlarm(void* pvParameters);
void TaskMain(void* pvParameters);
void TaskConnection(void* pvParameters);
void TaskMovement(void* pvParameters);

// shared struct resource
struct home_state home;


// NewPing setup of pins and maximum distance
NewPing sonar(MOV_TRIG, MOV_ECHO, MAX_DISTANCE); 

void setup() {

  Serial.begin(115200);
  pinMode(BUZZER, OUTPUT);
  pinMode(ALARM_LED, OUTPUT);
  

  /* Inizializzazione struttura e semafori*/
  for (int i = 0; i < SLAVES; i++)
  {
    home.slave_state[i] = CLOSE;
    pinMode(TERMINALS[i], INPUT);
    pinMode(LED[i], OUTPUT);
  }
  home.alarm_mode = ENABLED;
  home.alarm_sound = OFF;
  home.open_slaves = 0;
  mutex_home = xSemaphoreCreateCounting(1, 1);
  mutex_alarm = xSemaphoreCreateCounting(1, 0);
  mutex_movement = xSemaphoreCreateCounting(1, 0);

  Serial.println("Creating slave tasks..");
  delay(200);   
  
    /* Creazione N task di slave */
  for (int i = 0; i < SLAVES; i++)
  { 
    char taskName[15];
    snprintf(taskName, sizeof(taskName), "Task Slave %d", i);
       
    xTaskCreate(
    TaskSlave,
    taskName,     
    2500,       
    (void *) i,        
    10,           
    &Handle_TaskSlave[i]
    );
    delay(500);
  }

    /* Creazione task di allarme */
  xTaskCreatePinnedToCore(
    TaskAlarm,   /* Task function. */
    "Alarm Task",     /* name of task. */
    2000,       /* Stack size of task */
    NULL,        /* parameter of the task */
    15,           /* priority of the task */
    &Handle_TaskAlarm,      /* Task handle to keep track of created task */
    0);
    delay(200);
        
    /* Creazione task di connessione */
  xTaskCreatePinnedToCore(
    TaskConnection,   /* Task function. */
    "Connection Task",     /* name of task. */
    2300,       /* Stack size of task */
    NULL,        /* parameter of the task */
    10,           /* priority of the task */
    &Handle_TaskConnection,      /* Task handle to keep track of created task */
    1);
    delay(200);
  
    /* Creazione task di gestione sistema */
  xTaskCreatePinnedToCore(
    TaskMain,   /* Task function. */
    "Main Task",     /* name of task. */
    1000,       /* Stack size of task */
    NULL,        /* parameter of the task */
    12,           /* priority of the task */
    &Handle_TaskMain,      /* Task handle to keep track of created task */
    1);
    delay(200);

    /* Creazione task di rilevamento movimenti */
  xTaskCreatePinnedToCore(
    TaskMovement,   /* Task function. */
    "Movement Detection Task",     /* name of task. */
    1000,       /* Stack size of task */
    NULL,        /* parameter of the task */
    10,           /* priority of the task */
    &Handle_TaskMovement,      /* Task handle to keep track of created task */
    1);
    delay(200);
  
}

void TaskConnection(void *pvParameters) {
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 500 / portTICK_PERIOD_MS;
  // Initialise the xLastWakeTime variable with the current time.
  xLastWakeTime = xTaskGetTickCount();
  
  Serial.print("Created Connection Task on core: ");
  Serial.println(xPortGetCoreID());
  
  /* Connessione Wifi */
  WifiConnection();

  /* Connessione MQTT */
  mqttConnection();
  
  while (1)
  {
    client.loop();
    
    if (!client.connected()) {
      reconnect();
    }
    
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
  }
  
}

void TaskAlarm(void *pvParameters) {
  
  Serial.print("Created Allarm Task on core: ");
  Serial.println(xPortGetCoreID());


  while (1)
  {
    /*  SOSPENSIONE ALLARME */
    xSemaphoreTake(mutex_alarm, 0xffffffff);

    /*  SEMAFORO SEZIONE CRITICA */
    xSemaphoreTake(mutex_home, 0xffffffff);
    home.alarm_sound = ON;
    xSemaphoreGive(mutex_home);

    client.publish(topic_alarm_sound, "ALARM ON");
    digitalWrite(ALARM_LED, HIGH);
    digitalWrite(BUZZER, HIGH);
    delay(200);
    Serial.println("ALARM ON");

    xSemaphoreGive(mutex_movement);
    
    /*  SOSPENSIONE ALLARME */
    xSemaphoreTake(mutex_alarm, 0xffffffff);

   /*  SEMAFORO SEZIONE CRITICA */
    xSemaphoreTake(mutex_home, 0xffffffff);
    home.alarm_sound = OFF;
    xSemaphoreGive(mutex_home);

    client.publish(topic_alarm_sound, "ALARM OFF");
    digitalWrite(BUZZER, LOW);
    digitalWrite(ALARM_LED, LOW);
    Serial.println("ALARM OFF");

    xSemaphoreGive(mutex_movement);

  }  
}

void TaskSlave(void *pvParameters) {
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 300 / portTICK_PERIOD_MS;
  // Initialise the xLastWakeTime variable with the current time.
  xLastWakeTime = xTaskGetTickCount();
  
  /* ID SLAVE */
  uint32_t id = (uint32_t)pvParameters;
    
  Serial.print("Created Task ");
  Serial.print(id);
  Serial.print(" on core: ");
  Serial.println(xPortGetCoreID());

  char topic_slaves[24]; // Salvo le stringhe dei topic di ogni slave
  snprintf(topic_slaves, 24, "KfZ91%%%%7BM@/Window/%d", id); 
  
  while (1)
  {  
     if(digitalRead(TERMINALS[id]) == HIGH)
      {
      
        xSemaphoreTake(mutex_home, 0xffffffff);
        
        home.slave_state[id] = !home.slave_state[id];
        if (home.slave_state[id]) {
          home.open_slaves++;
        } else {
          home.open_slaves--;
        }
        char value[5];
        snprintf(value, sizeof(value), "%d", home.open_slaves);
        client.publish(topic_open_slaves, value);             
        client.publish(topic_slaves, ((home.slave_state[id]) ? "OPEN" : "CLOSED"));
        digitalWrite(LED[id],home.slave_state[id]);
        Serial.print("Window ");
        Serial.print(id);
        Serial.print((home.slave_state[id]) ? " OPEN " : " CLOSED ");

        if (home.slave_state[id] && home.alarm_mode && !home.alarm_sound) {
          xSemaphoreGive(mutex_home);
          xSemaphoreGive(mutex_alarm);
        } else {
          xSemaphoreGive(mutex_home);    
        }  
       }

      vTaskDelayUntil( &xLastWakeTime, xFrequency );
  }
  
}

void TaskMain(void *pvParameters) {
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 500 / portTICK_PERIOD_MS;
  // Initialise the xLastWakeTime variable with the current time.
  xLastWakeTime = xTaskGetTickCount();
    
  Serial.print("Created Task Main on core: ");
  Serial.println(xPortGetCoreID());

  while (1)
  {  
    if (client.connected() && ACK) {
      if (strcmp(topic_id, topic_alarm) == 0)   /* Verifica disattivazione suono allarme */
      {
        xSemaphoreTake(mutex_home, 0xffffffff);
        if (home.alarm_mode && home.alarm_sound) {
          client.publish(topic_alarm_received, "sound disabled");
          xSemaphoreGive(mutex_home);
          xSemaphoreGive(mutex_alarm);
          Serial.println(topic_payload);
        } else {
          client.publish(topic_alarm_received, "sound is off or alarm is disabled");
          xSemaphoreGive(mutex_home);
        }
        
      } else if (strcmp(topic_id, topic_alarm_mode_on) == 0)  /* Verifica abilitazione allarme */
      {
        xSemaphoreTake(mutex_home, 0xffffffff);
        if (!home.alarm_mode && !home.alarm_sound && home.open_slaves == 0)
        {
          home.alarm_mode = ENABLED;
          client.publish(topic_alarm_received, topic_payload);
          xSemaphoreGive(mutex_home);
          Serial.println(topic_payload);
        } else
        {    
          client.publish(topic_alarm_received, "Check if slaves open or sound on or alarm already enabled");
          xSemaphoreGive(mutex_home);
          Serial.println("Check if slaves open or sound on or alarm already enabled");  
        }
        
      } else if (strcmp(topic_id, topic_alarm_mode_off)  == 0)  /* Verifica disabilitazione allarme */
      {
        xSemaphoreTake(mutex_home, 0xffffffff);
        home.alarm_mode = DISABLED;
        client.publish(topic_alarm_received, topic_payload);
        xSemaphoreGive(mutex_home);
        Serial.println(topic_payload);
      } else{
        Serial.println("Message with no topic");
      }
      memset(topic_id, 0, sizeof(topic_id));
      memset(topic_payload, 0, sizeof(topic_payload));
      ACK = 0;
    }

     vTaskDelayUntil( &xLastWakeTime, xFrequency);
  }
  
}

void TaskMovement(void *pvParameters) {
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 1000 / portTICK_PERIOD_MS;
  // Initialise the xLastWakeTime variable with the current time.
  xLastWakeTime = xTaskGetTickCount();
  
  Serial.print("Created movement detection Task on core: ");
  Serial.println(xPortGetCoreID());
  
   while (1)
  {
     /*  SOSPENSIONE TASK */
    xSemaphoreTake(mutex_movement, 0xffffffff);
    unsigned int distance;
    while (uxSemaphoreGetCount(mutex_movement) == 0)
    {
      distance = sonar.ping_cm();
      Serial.print("distance ");
      Serial.print(distance);
      Serial.println("cm");
      vTaskDelay(1000);
    }
     xSemaphoreTake(mutex_movement, 0xffffffff);
  }
  
}


void WifiConnection() {
  // Connect to Wi-Fi network with SSID and password
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(200);
    Serial.print(".");
  }
  // Print local IP address and start web server
  Serial.println();
  Serial.println("WiFi connected.");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void mqttConnection() {
    if (client.connect(clientid)) {
    Serial.println("Connected to MQTT");
    subscriptions();
  } else {
    Serial.print("Connection to MQTT FAILED - State: ");
    Serial.println(client.state());
  }
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    if (client.connect(clientid)) {
     Serial.println("Reconnected to MQTT");
     subscriptions();
    }
  }
}

void subscriptions() {
  client.subscribe(topic_alarm);
  client.subscribe(topic_alarm_mode_on);
  client.subscribe(topic_alarm_mode_off);  
}

void callback(char* topic, byte* payload, unsigned int length) {
  int ret;
  if ((int)strlen(topic) > 30) {
    ret = sprintf(topic_id, "%s", "Error");
    if (ret < 0){
      Serial.println("Errore callback");
    }
  }
  else {
    ret = sprintf(topic_id, "%s", topic);
    if (ret < 0){
      Serial.println("Errore callback");
    }
  }

  Serial.print("Message arrived [");
  Serial.print(topic_id);
  Serial.print("] ");

  for (unsigned int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
    topic_payload[i] = (char)payload[i];
  }
  topic_payload[length] = 0;
  ACK = 1;
  Serial.println();

  // whatever you want for this topic
}

void loop() {
  // put your main code here, to run repeatedly:

    /* DEBUG utilizzato per dimensione STACK dei task */

    // Serial.print(pcTaskGetTaskName(NULL));
    // Serial.print(" uxTaskGetStackHighWaterMark = ")
    // Serial.println(uxTaskGetStackHighWaterMark(NULL));
}
