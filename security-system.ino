#include "structure.h"

// TaskHandles
TaskHandle_t Handle_TaskSlave[SLAVES];
TaskHandle_t Handle_TaskAlarm;
TaskHandle_t Handle_TaskMain;
TaskHandle_t Handle_TaskConnection;
TaskHandle_t Handle_TaskMovementDetection;
TaskHandle_t Handle_TaskDisplay;
TaskHandle_t Handle_TaskSetAlarmButton;

// Timers
TimerHandle_t Timer_Alarm;
TimerHandle_t Timer_MovementDetection;
TimerHandle_t Timer_Code;

// Semaphores
SemaphoreHandle_t mutex_home;
SemaphoreHandle_t mutex_alarm;
SemaphoreHandle_t mutex_movement;
SemaphoreHandle_t mutex_password;

// Task Definitions
void TaskSlave(void *pvParameters);
void TaskAlarm(void *pvParameters);
void TaskMain(void *pvParameters);
void TaskConnection(void *pvParameters);
void TaskMovementDetection(void *pvParameters);
void TaskDisplay(void *pvParameters);
void TaskSetAlarmButton(void *pvParameters);

// Shared resource
struct home_state home;

void setup()
{

  Serial.begin(115200);
  pinMode(ALARMBUTTON, INPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(ALARMLED, OUTPUT);
  pinMode(rgbREDLED, OUTPUT);
  pinMode(rgbGREENLED, OUTPUT);

  /* Time library Initialization */
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);


  /* Structure and semaphores Initialization */
  for (int i = 0; i < SLAVES; i++)
  {
    home.slave_state[i] = CLOSE;
    pinMode(TERMINALS[i], INPUT);
    pinMode(LED[i], OUTPUT);
  }
  home.alarm_mode = DISABLED;
  home.alarm_sound = OFF;
  home.open_slaves = 0;
  digitalWrite(rgbREDLED, HIGH);
  digitalWrite(rgbGREENLED, LOW);
  mutex_home = xSemaphoreCreateCounting(1, 1);
  mutex_alarm = xSemaphoreCreateCounting(1, 0);
  mutex_movement = xSemaphoreCreateCounting(1, 0);
  mutex_password = xSemaphoreCreateCounting(1, 1);

  // MISRA C R. 11.6
  /* Creation of Software Timers */
  Timer_MovementDetection = xTimerCreate("Movement Detection Timer", pdMS_TO_TICKS(2000), pdTRUE, NULL, timer_callback);
  Timer_Code = xTimerCreate("Alarm code Timer", pdMS_TO_TICKS(20000), pdFALSE, NULL, timer_callback);
  Timer_Alarm = xTimerCreate("Alarm sound Timer", pdMS_TO_TICKS(60000), pdFALSE, NULL, timer_callback);

  Serial.println("Creating slave tasks..");
  delay(200);

  // MISRA C Violazione R. 11.6
  /* Creation of slave tasks */
  for (int i = 0; i < SLAVES; i++)
  {
    char taskName[15];
    int ret = snprintf(taskName, sizeof(taskName), "Task Slave %d", i);
    if (ret < 0) // MISRA C R. 17.7 OK
    {
      Serial.println("Error snprintf");
    }

    xTaskCreate(
        TaskSlave,
        taskName,
        5000,
        (void *)i,
        10,
        &Handle_TaskSlave[i]);
    delay(200);
  }

  /* Creation of Alarm Task */
  xTaskCreatePinnedToCore(
      TaskAlarm,         /* Task function. */
      "Alarm Task",      /* name of task. */
      5000,              /* Stack size of task */
      NULL,              /* parameter of the task */
      15,                /* priority of the task */
      &Handle_TaskAlarm, /* Task handle to keep track of created task */
      0);                /* Task pinned to core */
  delay(200);

  /* Creation of Alarm Button Task */
  xTaskCreatePinnedToCore(
      TaskSetAlarmButton,
      "Alarm Button Task",
      2000,
      NULL,
      12,
      &Handle_TaskSetAlarmButton,
      0);
  delay(200);

  /* Creation of Connection Task */
  xTaskCreatePinnedToCore(
      TaskConnection,
      "Connection Task",
      5000,
      NULL,
      10,
      &Handle_TaskConnection,
      1);
  delay(200);

  /* Creation of Main Task */
  xTaskCreatePinnedToCore(
      TaskMain,
      "Main Task",
      10000,
      NULL,
      15,
      &Handle_TaskMain,
      1);
  delay(200);

  /* Creation of Movement Detection Task */
  xTaskCreatePinnedToCore(
      TaskMovementDetection,
      "Movement Detection Task",
      10000,
      NULL,
      10,
      &Handle_TaskMovementDetection,
      1);
  delay(200);

}

void TaskAlarm(void *pvParameters)
{

  Serial.print("Created Alarm Task on core: ");
  Serial.println(xPortGetCoreID());

  while (1)
  {
    /*  Alarm suspension */
    xSemaphoreTake(mutex_alarm, portMAX_DELAY);

    xSemaphoreTake(mutex_home, portMAX_DELAY);
    home.alarm_sound = ON;
    xSemaphoreGive(mutex_home);

    xTimerStart(Timer_Alarm, 0);
    digitalWrite(ALARMLED, HIGH);
    digitalWrite(BUZZER, HIGH);
    client.publish(topic_sound_status, "ON");
    client.publish(topic_notification, "*SOUND ON*");
    Serial.println("ALARM SOUND ON!");

    /*  Alarm suspension */
    xSemaphoreTake(mutex_alarm, portMAX_DELAY);

    if (give_fromTimer) // If the xSemaphoreGive comes from the callback timer (Timer_Alarm Expired), i have to disable the alarm
    {
      xSemaphoreTake(mutex_home, portMAX_DELAY);
      home.alarm_mode = DISABLED;
      xSemaphoreGive(mutex_home);
      digitalWrite(rgbREDLED, HIGH);
      digitalWrite(rgbGREENLED, LOW);
      client.publish(topic_notification, "Alarm Timer expired: sound and alarm disabled");
      Serial.println("Alarm Timer expired: sound and alarm disabled");
      xTimerStop(Timer_MovementDetection, 0);
      give_fromTimer = false;
    }

    xSemaphoreTake(mutex_home, portMAX_DELAY);
    home.alarm_sound = OFF;
    xSemaphoreGive(mutex_home);


    digitalWrite(BUZZER, LOW);
    digitalWrite(ALARMLED, LOW);
    client.publish(topic_sound_status, "OFF");
    client.publish(topic_notification, "*SOUND OFF*");
    Serial.println("ALARM SOUND OFF");
  }
}

void TaskSlave(void *pvParameters)
{
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 500 / portTICK_PERIOD_MS;
  xLastWakeTime = xTaskGetTickCount();

  // MISRA C R. 11.6 VIOLATION

  /* ID slave */
  uint32_t id = (uint32_t)pvParameters;

  Serial.print("Created slave task ");
  Serial.print(id);
  Serial.print(" on core: ");
  Serial.println(xPortGetCoreID());

  /* Save the topic strings of each slave */
  char topic_slaves[24];
  int ret = snprintf(topic_slaves, 24, *basetopic + "terminals/%d", id);
  if (ret < 0) // MISRA C R.17.7 OK
  {
    Serial.println("Error snprintf");
  }

  while (1)
  {
    if (digitalRead(TERMINALS[id]) == HIGH)
    {

      xSemaphoreTake(mutex_home, portMAX_DELAY);
      home.slave_state[id] = !home.slave_state[id];
      if (home.slave_state[id])
      {
        home.open_slaves++;
      }
      else
      {
        home.open_slaves--;
      }
      char value[4];
      int ret = snprintf(value, sizeof(value), "%d", home.open_slaves);
      if (ret < 0) // MISRA C R.17.7 OK
      {
        Serial.println("Error snprintf");
      }
      client.publish(topic_open_slaves, value);
      client.publish(topic_open, value);
      struct tm time = getDateTime();
      char message[40];
      if (home.slave_state[id])
      {
        int return_v = strftime(message, sizeof(message), "%H:%M \n OPEN", &time);
        if (return_v == 0) // MISRA C R.17.7 OK
        {
          Serial.println("Error strftime");
        }
      }
      else
      {
        int return_v = strftime(message, sizeof(message), "%H:%M \n CLOSE", &time);
        if (return_v == 0) // MISRA C R.17.7 OK
        {
          Serial.println("Error strftime");
        }
      }
      client.publish(topic_slaves, message);
      Serial.println(message);
      digitalWrite(LED[id], home.slave_state[id]);

      if (home.slave_state[id] && home.alarm_mode && !home.alarm_sound)
      {
        xSemaphoreGive(mutex_home);
        xSemaphoreGiveFromISR(mutex_alarm, 0);
      }
      else
      {
        xSemaphoreGive(mutex_home);
      }
    }

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void TaskSetAlarmButton(void *pvParameters)
{
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 500 / portTICK_PERIOD_MS;
  xLastWakeTime = xTaskGetTickCount();

  Serial.print("Created Task Alarm button on core: ");
  Serial.println(xPortGetCoreID());

  while (1)
  {
    if (digitalRead(ALARMBUTTON) == HIGH)
    {
      xSemaphoreTake(mutex_home, portMAX_DELAY);
      if (home.alarm_mode)
      {
        home.alarm_mode = DISABLED;
        digitalWrite(rgbREDLED, HIGH);
        digitalWrite(rgbGREENLED, LOW);
        client.publish(topic_notification, "Alarm Disabled");
        client.publish(topic_mode_status, "OFF");
      }
      else
      {
        home.alarm_mode = ENABLED;
        digitalWrite(rgbGREENLED, HIGH);
        digitalWrite(rgbREDLED, LOW);
        client.publish(topic_notification, "Alarm Enabled");
        client.publish(topic_mode_status, "ON");
      }
      xSemaphoreGive(mutex_home);
    }

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void TaskConnection(void *pvParameters)
{
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 1000 / portTICK_PERIOD_MS;
  xLastWakeTime = xTaskGetTickCount();

  Serial.print("Created Connection Task on core: ");
  Serial.println(xPortGetCoreID());

  WifiConnection();

  mqttConnection();

  while (1)
  {
    client.loop();

    if (!client.connected())
    {
      reconnect();
    }

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void TaskMain(void *pvParameters)
{
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 1000 / portTICK_PERIOD_MS;
  xLastWakeTime = xTaskGetTickCount();

  Serial.print("Created Task Main on core: ");
  Serial.println(xPortGetCoreID());

  while (1)
  {
    if (client.connected() && ACK) /* ACK => New message */
    {
      if (strcmp(topic_id, topic_sound_off) == 0) /* Verifiy deactivation of the alarm sound */
      {
        xSemaphoreTake(mutex_home, portMAX_DELAY);
        if (home.alarm_mode && home.alarm_sound)
        {
          home.alarm_mode = DISABLED;
          xSemaphoreGive(mutex_home);
          digitalWrite(rgbREDLED, HIGH);
          digitalWrite(rgbGREENLED, LOW);
          client.publish(topic_notification, "Sound and alarm disabled");
          client.publish(topic_sound_status, "OFF");
          xSemaphoreGiveFromISR(mutex_alarm, 0);
          xTimerStop(Timer_Alarm, 0);
          xTimerStop(Timer_MovementDetection, 0);
        }
        else if (!home.alarm_mode)
        {
          xSemaphoreGive(mutex_home);
          client.publish(topic_notification, "Alarm is disabled!");
        }
        else if (home.alarm_mode && !home.alarm_sound)
        {
          xSemaphoreGive(mutex_home);
          client.publish(topic_notification, "Sound is off!");
        }
      }
      else if (strcmp(topic_id, topic_mode_on) == 0) /* Verify alarm enabling */
      {
        xSemaphoreTake(mutex_home, portMAX_DELAY);
        // MISRA C R. 12.1 OK
        if (!home.alarm_mode && !home.alarm_sound && (home.open_slaves == 0))
        {
          home.alarm_mode = ENABLED;
          xSemaphoreGive(mutex_home);
          digitalWrite(rgbGREENLED, HIGH);
          digitalWrite(rgbREDLED, LOW);
          client.publish(topic_notification, topic_payload);
          if (xTimerIsTimerActive(Timer_MovementDetection) != pdFALSE)
          {
            /* xTimer is active, do something. */
            Serial.println("Timer_MovementDetection IS active");
          }
          else
          {
            /* xTimer is not active, do something else. */
            Serial.println("Timer_MovementDetection IS NOT active");
            // START
            xTimerStart(Timer_MovementDetection, 0);
          }

        }
        else if (home.open_slaves > 0) // MISRA C R. 14.4 OK
        {
          xSemaphoreGive(mutex_home);
          client.publish(topic_notification, "Error - Slaves open, it'impossible to enable alarm");
        }
        else if (home.alarm_mode)
        {
          xSemaphoreGive(mutex_home);
          client.publish(topic_notification, "Error - Alarm already enabled!");
        }
        else if (home.alarm_sound)
        {
          xSemaphoreGive(mutex_home);
          client.publish(topic_notification, "Error - Alarm sound is ON!");
        }
      }
      else if (strcmp(topic_id, topic_mode_off) == 0) /* Verify alarm disabling */
      {
        xSemaphoreTake(mutex_home, portMAX_DELAY);
        if (home.alarm_mode && !home.alarm_sound)
        {
          home.alarm_mode = DISABLED;
          xSemaphoreGive(mutex_home);
          digitalWrite(rgbGREENLED, LOW);
          digitalWrite(rgbREDLED, HIGH);
          client.publish(topic_notification, topic_payload);
          xTimerStop(Timer_MovementDetection, portMAX_DELAY);
          Serial.println(topic_payload);
        }
        else if (!home.alarm_mode)
        {
          xSemaphoreGive(mutex_home);
          client.publish(topic_notification, "Error - Alarm already disabled!");
        }
        else if (home.alarm_sound)
        {
          xSemaphoreGive(mutex_home);
          client.publish(topic_notification, "Error - Alarm sound is on!");
        }
      }
      else if (strcmp(topic_id, topic_code) == 0) /* Check disabling of alarm timer by code */
      {
        Serial.println(topic_payload);
        stopTimerAndDisableAlarm();
        xSemaphoreTake(mutex_password, portMAX_DELAY);
        insert_password = false;
        xSemaphoreGive(mutex_password);
        vTaskResume(Handle_TaskDisplay);
        client.publish(topic_notification, "Timer stopped and alarm disabled with App code");
      }
      else
      {
        Serial.println("Message with no topic");
      }
      memset(topic_id, 0, sizeof(topic_id));
      memset(topic_payload, 0, sizeof(topic_payload));
      ACK = 0;
    }

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void stopTimerAndDisableAlarm()
{
  TickType_t xRemainingTime;
  xRemainingTime = xTimerGetExpiryTime(Timer_Code) - xTaskGetTickCount();
  char message[40];
  snprintf(message, sizeof(message), "Remaining time to activate alarm: %s", xRemainingTime);
  Serial.println(message);
  client.publish(topic_notification, message);
  xTimerStop(Timer_Code, 0);
  xSemaphoreTake(mutex_home, portMAX_DELAY);
  home.alarm_mode = DISABLED;
  xSemaphoreGive(mutex_home);
  digitalWrite(rgbREDLED, HIGH);
  digitalWrite(rgbGREENLED, LOW);
  Serial.println("Timer stopped and Alarm Disabled");
  client.publish(topic_notification, "Timer stopped and Alarm Disabled");
}

void TaskMovementDetection(void *pvParameters)
{

  Serial.print("Created movement detection Task on core: ");
  Serial.println(xPortGetCoreID());

  Ultrasonic ultrasonic(MOV_TRIG, MOV_ECHO);

  xSemaphoreTake(mutex_home, portMAX_DELAY);
  if (home.alarm_mode)
  {
    xTimerStart(Timer_MovementDetection, 0);
  }
  xSemaphoreGive(mutex_home);
  int val;

  while (1)
  {
    xSemaphoreTake(mutex_movement, portMAX_DELAY);
    xSemaphoreTake(mutex_home, portMAX_DELAY);
    if (home.alarm_mode && !home.alarm_sound)
    {
      xSemaphoreGive(mutex_home);
      if (!first_read)
      {
        val = ultrasonic.distanceRead();
        Serial.print("distance: ");
        Serial.println(val);
        client.publish(topic_distance, (const char*)val);
        if (val != 0)
        {
          last_distance = val;
          curr_distance = val;
          first_read = true;
        }
      }
      else
      {
        last_distance = curr_distance;
        val = ultrasonic.distanceRead();
        Serial.print("distance: ");
        Serial.println(val);
        client.publish(topic_distance, (const char*)val);

        if (val != 0)
        {
          curr_distance = val;
        }
        // MISRA C R. 12.1 OK
        if (((curr_distance - last_distance) > MAX_DISTANCE) || ((curr_distance - last_distance) < -MAX_DISTANCE))
        {
          Serial.println("MOVEMENT DETECTED");
          client.publish(topic_notification, "MOVEMENT DETECTED");
          xSemaphoreTake(mutex_home, portMAX_DELAY);
          if (!home.alarm_sound)
          {
            xSemaphoreGive(mutex_home);
            xSemaphoreTake(mutex_password, portMAX_DELAY);
            insert_password = true;
            xSemaphoreGive(mutex_password);
          }
          else
          {
            xSemaphoreGive(mutex_home);
          }
        }
      }
    }
    else
    {
      xSemaphoreGive(mutex_home);
    }
  }
}

void TaskDisplay(void *pvParameters)
{
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 2000 / portTICK_PERIOD_MS;
  xLastWakeTime = xTaskGetTickCount();

  Serial.print("Created Task Display on core: ");
  Serial.println(xPortGetCoreID());

  while (1)
  {
    xSemaphoreTake(mutex_password, portMAX_DELAY);
    if (insert_password)
    {
      xSemaphoreGive(mutex_password);
      vTaskSuspend(NULL);
    }
    else
    {
      xSemaphoreGive(mutex_password);
    }

    xSemaphoreTake(mutex_home, portMAX_DELAY);

    if (home.alarm_mode && !home.alarm_sound)
    {
      char value[5];
      int ret = snprintf(value, sizeof(value), "%d", home.open_slaves);
      if (ret < 0) // MISRA C R.17.7
      {
        Serial.println("Error snprintf");
      }

      xSemaphoreGive(mutex_home);
      char message[32];
      snprintf(message, sizeof(message), "Alarm ON, Open slaves: %s", value);
      Serial.println(message);
      client.publish(topic_general, message);
    }
    else if (!home.alarm_mode && !home.alarm_sound)
    {
      char value[5];
      int ret = snprintf(value, sizeof(value), "%d", home.open_slaves);
      if (ret < 0) // MISRA C R.17.7
      {
        Serial.println("Error snprintf");
      }

      char message[32];
      snprintf(message, sizeof(message), "Alarm ON, Open slaves: %s", value);
      Serial.println(message);
      client.publish(topic_general, message);
    }
    else if (home.alarm_sound)
    {
      xSemaphoreGive(mutex_home);      
      char *message = "ATTENTION **, ALARM SOUND ON!";
      Serial.println(message);
      client.publish(topic_general, message);
    }

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void WifiConnection()
{
  // Connect to Wi-Fi network with SSID and password
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(200);
    Serial.print(".");
  }
  // Print local IP address and start web server
  Serial.println();
  Serial.println("WiFi connected.");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void mqttConnection()
{
  if (client.connect(clientid) == true) // MISRA C R. 14.4 OK
  {
    Serial.println("Connected to MQTT");
    subscriptions();
  }
  else
  {
    Serial.print("Connection to MQTT FAILED - State: ");
    Serial.println(client.state());
  }
}

void reconnect()
{
  // Loop until we're reconnected
  while (!client.connected())
  {
    if (client.connect(clientid) == true) // MISRA C R. 14.4 OK
    {
      Serial.println("Reconnected to MQTT");
      subscriptions();
    }
  }
}

void subscriptions()
{
  client.subscribe(topic_sound_off);
  client.subscribe(topic_mode_on);
  client.subscribe(topic_mode_off);
  client.subscribe(topic_code);
}

struct tm getDateTime()
{
  struct tm timeinfo;
  timeinfo = {};
  if (!getLocalTime(&timeinfo))
  {
    Serial.println("Failed to obtain time");
  }

  return timeinfo;
}

void callback(char *topic, byte *payload, unsigned int length)
{
  int ret;
  if ((int)strlen(topic) > 30)
  {
    ret = sprintf(topic_id, "%s", "Error");
    if (ret < 0)
    {
      Serial.println("Errore callback");
    }
  }
  else
  {
    ret = sprintf(topic_id, "%s", topic);
    if (ret < 0)
    {
      Serial.println("Errore callback");
    }
  }

  Serial.print("Message arrived [");
  Serial.print(topic_id);
  Serial.print("] ");

  for (unsigned int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
    topic_payload[i] = (char)payload[i];
  }
  topic_payload[length] = 0;
  ACK = 1;
  Serial.println();
}

void timer_callback(TimerHandle_t xTimer)
{
  if (xTimer == Timer_MovementDetection)
  {
    xSemaphoreGive(mutex_movement);
  }
  else if (xTimer == Timer_Code)
  {
    xSemaphoreTake(mutex_password, portMAX_DELAY);
    insert_password = false;
    xSemaphoreGive(mutex_password);
    vTaskResume(Handle_TaskDisplay);
    xSemaphoreGiveFromISR(mutex_alarm, 0);
  }
  else if (xTimer == Timer_Alarm)
  {
    give_fromTimer = true;
    xSemaphoreGiveFromISR(mutex_alarm, 0);
  }
  else // MISRA C R. 15.7 OK
  {
    Serial.println("Timer Handle not recognized");
  }
}


void loop()
{
  // put your main code here, to run repeatedly:
}
