#include "structure.h"

// TaskHandles
TaskHandle_t Handle_TaskSlave[SLAVES];
TaskHandle_t Handle_TaskAllarm;
TaskHandle_t Handle_TaskConnection;

// Semaphores
SemaphoreHandle_t mutex_slaves;
SemaphoreHandle_t mutex_allarm;
SemaphoreHandle_t mutex_allarm_priv;

//Task Definitions
void TaskSlave(void* pvParameters);
void TaskAllarm(void* pvParameters);
void TaskConnection(void* pvParameters);

// shared struct resource
struct home_state home;

void setup() {

  Serial.begin(115200);
  pinMode(BUZZER, OUTPUT);
  pinMode(13, INPUT);
  pinMode(4, OUTPUT);
  

  /* Inizializzazione struttura e semafori*/
  for (int i = 0; i < SLAVES; i++)
  {
    home.slave_state[i] = CLOSE;
    pinMode(TERMINALS[i], INPUT);
    pinMode(LED[i], OUTPUT);
  }
  home.allarm_mode = DISABLED;
  home.allarm_sound = OFF;
  home.open_slaves = 0;
  mutex_slaves = xSemaphoreCreateCounting(1, 1);
  mutex_allarm = xSemaphoreCreateCounting(1, 1);
  mutex_allarm_priv = xSemaphoreCreateCounting(1, 0);


  
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
    10000,       
    (void *) i,        
    10,           
    &Handle_TaskSlave[i]
    );
    delay(500);
  }

  /* Creazione task di allarme */
  xTaskCreate(
    TaskAllarm,   /* Task function. */
    "Task for the allarm",     /* name of task. */
    10000,       /* Stack size of task */
    NULL,        /* parameter of the task */
    15,           /* priority of the task */
    &Handle_TaskAllarm      /* Task handle to keep track of created task */
    );
    delay(200);
    
    /* Creazione task di connessione */
  xTaskCreate(
    TaskConnection,   /* Task function. */
    "Task for the connection to Wifi and MQTT",     /* name of task. */
    10000,       /* Stack size of task */
    NULL,        /* parameter of the task */
    10,           /* priority of the task */
    &Handle_TaskConnection      /* Task handle to keep track of created task */
    );
    delay(200);
  
}

void TaskAllarm(void *pvParameters) {
  // TickType_t xLastWakeTime;
  // const TickType_t xFrequency = 500 / portTICK_PERIOD_MS;
  // // Initialise the xLastWakeTime variable with the current time.
  // xLastWakeTime = xTaskGetTickCount();
  
  Serial.println("Created Allarm Task ");

  while (1)
  {
    /*  SOSPENSIONE ALLARME */
    xSemaphoreTake(mutex_allarm_priv, 0xffffffff);

    /*  SEMAFORO SEZIONE CRITICA */
    xSemaphoreTake(mutex_allarm, 0xffffffff);
    home.allarm_sound = ON;
    xSemaphoreGive(mutex_allarm);
    client.publish(topic_allarm_on, "ALLARM ON!!!");
    digitalWrite(4, HIGH);
    digitalWrite(BUZZER, HIGH);
    delay(200);
    Serial.println("ALLARM ON");
    
    /*  SOSPENSIONE ALLARME */
    xSemaphoreTake(mutex_allarm_priv, 0xffffffff);

   /*  SEMAFORO SEZIONE CRITICA */
    xSemaphoreTake(mutex_allarm, 0xffffffff);
    home.allarm_sound = OFF;
    xSemaphoreGive(mutex_allarm);
    
    digitalWrite(BUZZER, LOW);
    digitalWrite(4, LOW);
    Serial.println("ALLARM OFF");
    client.publish(topic_allarm_on, "ALLARM OFF");
    
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
  Serial.println(id);

  char topic_slaves[24]; // Salvo le stringhe dei topic di ogni slave
  snprintf(topic_slaves, 24, "KfZ91%%%%7BM@/Window/%d", id); 

  while (1)
  {  
     if(digitalRead(TERMINALS[id]) == HIGH)
      {
        /* TO-DO: mutex_slaves si o no ? I task effettivamente accedono a variabili diverse, private, 
          quindi non dovrebbero avere bisogno di mutua esclusione */
      
      //  xSemaphoreTake(mutex_slaves, 0xffffffff);
        home.slave_state[id] = !home.slave_state[id];             
        client.publish(topic_slaves, ((home.slave_state[id]) ? "OPEN" : "CLOSED"));
        digitalWrite(LED[id],home.slave_state[id]);
        Serial.print("Window ");
        Serial.print(id);
        Serial.println((home.slave_state[id]) ? " OPEN" : " CLOSED");

        /*  mutex_allarm e verifica condizioni per attivazione allarme*/
        xSemaphoreTake(mutex_allarm, 0xffffffff);
        if (home.slave_state[id] && home.allarm_mode && !home.allarm_sound) {
          xSemaphoreGive(mutex_allarm);
          xSemaphoreGive(mutex_allarm_priv);
        } else {
          xSemaphoreGive(mutex_allarm);    
        }  
       }

      vTaskDelayUntil( &xLastWakeTime, xFrequency );
  }
  
}

void TaskConnection(void *pvParameters) {
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 500 / portTICK_PERIOD_MS;
  // Initialise the xLastWakeTime variable with the current time.
  xLastWakeTime = xTaskGetTickCount();
  
  Serial.println("Created Connection Task ");
  
  /* Connessione Wifi */
  WifiConnection();

  /* Connessione MQTT */
  mqttConnection();
  
  while (1)
  {
    client.loop();
  }
  
   vTaskDelayUntil( &xLastWakeTime, xFrequency );
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
    subscription();
  } else {
    Serial.println("Connected to MQTT FAILED - State: ");
    Serial.println(client.state());
  }
}

void subscription() {
  client.subscribe(topic_allarm);  
}

void callback(char* topic, byte* payload, unsigned int length) {
   // Allocazione della quantità di memoria necessaria per la copia del payload
  byte* p = (byte*)malloc(length);
  memcpy(p,payload,length);
  // Ripubblicazione messaggio: ACK di conferma 
  client.publish(topic_allarm_received, p, length);
  
  /* Verifica per disattivazione allarme */
  xSemaphoreTake(mutex_allarm, 0xffffffff);
  if (home.allarm_sound) {
    xSemaphoreGive(mutex_allarm);
    xSemaphoreGive(mutex_allarm_priv);
  } else {
    xSemaphoreGive(mutex_allarm);
  }
  Serial.println("Allarm Deactivated");
  free(p);
}

void loop() {
  // put your main code here, to run repeatedly:
  
}
