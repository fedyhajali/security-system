#include "structure.h"

// TaskHandles
TaskHandle_t Handle_TaskSlave[SLAVES];
TaskHandle_t Handle_TaskAllarm;


//Task Definitions
void TaskSlave(void* pvParameters);
void TaskAllarm(void* pvParameters);

// Shared Resource
struct home_state {
  bool state[SLAVES];  // slave state: OPEN/CLOSE
  bool allarm_mode;   // mode: DISABLED/ENABLED
};

struct home_state home;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(BUZZER, OUTPUT);

  
  for (int i = 0; i < SLAVES; i++)
  {
    home.state[i] = CLOSE;
    home.allarm_mode = ENABLED;
    pinMode(TERMINALS[i], INPUT);
    pinMode(LED[i], OUTPUT);
  }

  /* Creo task di allarme*/
  xTaskCreate(
    TaskAllarm,   /* Task function. */
    "Task for the allarm",     /* name of task. */
    10000,       /* Stack size of task */
    NULL,        /* parameter of the task */
    15,           /* priority of the task */
    &Handle_TaskAllarm      /* Task handle to keep track of created task */
    );
  
  Serial.println("Creating slave tasks..");
  delay(200);   


  /* Creo N task di slave*/
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
  
}

void TaskSlave(void *pvParameters) {
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 200 / portTICK_PERIOD_MS;
  // Initialise the xLastWakeTime variable with the current time.
  xLastWakeTime = xTaskGetTickCount();
  
  uint32_t id = (uint32_t)pvParameters;
    
  Serial.print("Created Task ");
  Serial.println(id);
  bool printed = 0;
  
  while (1)
  {  
     if(digitalRead(TERMINALS[id]) == HIGH )
      {
        home.state[id] = !home.state[id];         
        digitalWrite(LED[id],home.state[id]);
        Serial.print("Window ");
        Serial.print(id);
        Serial.print(" = ");
        if(home.state[id]){ 
          Serial.println("OPEN");
        } else {
          Serial.println("CLOSED");
        }
      }
    
      vTaskDelayUntil( &xLastWakeTime, xFrequency );
  }
  
}

void TaskAllarm(void *pvParameters) {
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 500 / portTICK_PERIOD_MS;
  // Initialise the xLastWakeTime variable with the current time.
  xLastWakeTime = xTaskGetTickCount();
  
  Serial.print("Created Allarm Task ");

  while (1)
  {
    delay(2000);
  }
  
}

void loop() {
  // put your main code here, to run repeatedly:
}
