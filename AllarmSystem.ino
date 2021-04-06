#include "structure.h"

// TaskHandles
TaskHandle_t Handle_TaskSlave[SLAVES];
TaskHandle_t Handle_TaskAlarm;
TaskHandle_t Handle_TaskMain;
TaskHandle_t Handle_TaskConnection;
TaskHandle_t Handle_TaskMovementDetection;
TaskHandle_t Handle_TaskDisplay;
TaskHandle_t Handle_TaskPassword;

// Timers
TimerHandle_t Timer_Alarm;
TimerHandle_t Timer_MovementDetection;
TimerHandle_t Timer_Code;

// Semaphores
SemaphoreHandle_t mutex_home;
SemaphoreHandle_t mutex_alarm;
SemaphoreHandle_t mutex_movement;
SemaphoreHandle_t mutex_password;
SemaphoreHandle_t mutex_lcd;

//Task Definitions
void TaskSlave(void *pvParameters);
void TaskAlarm(void *pvParameters);
void TaskMain(void *pvParameters);
void TaskConnection(void *pvParameters);
void TaskMovementDetection(void *pvParameters);
void TaskDisplay(void *pvParameters);
void TaskPassword(void *pvParameters);

// shared struct resource
struct home_state home;

/* Inizializzazione Diplay */
LiquidCrystal lcd(DISPLAY1, DISPLAY2, DISPLAY3, DISPLAY4, DISPLAY5, DISPLAY6);

#include <IRremote.h>

void setup()
{

  Serial.begin(115200);
  pinMode(BUZZER, OUTPUT);
  pinMode(ALARM_LED, OUTPUT);

  /* Inizializzazione Time Library */
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  lcd.begin(16, 2);

  /* Inizializzazione struttura e semafori*/
  for (int i = 0; i < SLAVES; i++)
  {
    home.slave_state[i] = CLOSE;
    pinMode(TERMINALS[i], INPUT);
    pinMode(LED[i], OUTPUT);
  }
  home.alarm_mode = DISABLED;
  home.alarm_sound = OFF;
  home.open_slaves = 0;
  mutex_home = xSemaphoreCreateCounting(1, 1);
  mutex_alarm = xSemaphoreCreateCounting(1, 0);
  mutex_movement = xSemaphoreCreateCounting(1, 0);
  mutex_password = xSemaphoreCreateCounting(1, 1);
  mutex_lcd = xSemaphoreCreateCounting(1, 1);

  /* Creazione Software Timers*/
  Timer_MovementDetection = xTimerCreate("Movement sensor Timer", pdMS_TO_TICKS(2000), pdTRUE, (void *)0, timer_callback);
  Timer_Code = xTimerCreate("Alarm code Timer", pdMS_TO_TICKS(20000), pdFALSE, (void *)1, timer_callback);
  Timer_Alarm = xTimerCreate("Alarm Timer", pdMS_TO_TICKS(60000), pdFALSE, (void *)2, timer_callback);

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
        (void *)i,
        10,
        &Handle_TaskSlave[i]);
    delay(200);
  }

  /* Creazione task di allarme */
  xTaskCreatePinnedToCore(
      TaskAlarm,         /* Task function. */
      "Alarm Task",      /* name of task. */
      2500,              /* Stack size of task */
      NULL,              /* parameter of the task */
      15,                /* priority of the task */
      &Handle_TaskAlarm, /* Task handle to keep track of created task */
      0);                /* Task pinned to core */
  delay(200);

  /* Creazione task di connessione */
  xTaskCreatePinnedToCore(
      TaskConnection,
      "Connection Task",
      2500,
      NULL,
      10,
      &Handle_TaskConnection,
      1);
  delay(200);

  /* Creazione task di gestione sistema */
  xTaskCreatePinnedToCore(
      TaskMain,
      "Main Task",
      2000,
      NULL,
      15,
      &Handle_TaskMain,
      1);
  delay(200);

  /* Creazione task di rilevamento movimenti */
  xTaskCreatePinnedToCore(
      TaskMovementDetection,
      "Movement Detection Task",
      1000,
      NULL,
      10,
      &Handle_TaskMovementDetection,
      1);
  delay(200);

  /* Creazione task Display */
  xTaskCreatePinnedToCore(
      TaskDisplay,
      "Display Task",
      1000,
      NULL,
      10,
      &Handle_TaskDisplay,
      1);
  delay(200);
}

void TaskAlarm(void *pvParameters)
{

  Serial.print("Created Allarm Task on core: ");
  Serial.println(xPortGetCoreID());

  while (1)
  {
    /*  SOSPENSIONE ALLARME */
    xSemaphoreTake(mutex_alarm, portMAX_DELAY);

    /*  SEMAFORO SEZIONE CRITICA */
    xSemaphoreTake(mutex_home, portMAX_DELAY);
    home.alarm_sound = ON;
    xSemaphoreGive(mutex_home);

    client.publish(topic_alarm_sound_text, "ALARM SOUND ACTIVE");
    xTimerStart(Timer_Alarm, 0);
    digitalWrite(ALARM_LED, HIGH);
    digitalWrite(BUZZER, HIGH);
    Serial.println("ALARM ON");

    /*  SOSPENSIONE ALLARME */
    xSemaphoreTake(mutex_alarm, portMAX_DELAY);

    /*  SEMAFORO SEZIONE CRITICA */
    xSemaphoreTake(mutex_home, portMAX_DELAY);
    home.alarm_sound = OFF;
    xSemaphoreGive(mutex_home);

    client.publish(topic_alarm_sound_text, "ALARM SOUND OFF");
    digitalWrite(BUZZER, LOW);
    digitalWrite(ALARM_LED, LOW);
    Serial.println("ALARM OFF");
  }
}

void TaskSlave(void *pvParameters)
{
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 300 / portTICK_PERIOD_MS;
  // Initialise the xLastWakeTime variable with the current time.
  xLastWakeTime = xTaskGetTickCount();

  /* ID SLAVE */
  uint32_t id = (uint32_t)pvParameters;

  Serial.print("Created slave task ");
  Serial.print(id);
  Serial.print(" on core: ");
  Serial.println(xPortGetCoreID());

  char topic_slaves[24]; // Salvo le stringhe dei topic di ogni slave
  snprintf(topic_slaves, 24, "KfZ91%%%%7BM@/Window/%d", id);

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
      snprintf(value, sizeof(value), "%d", home.open_slaves);
      client.publish(topic_open_slaves, value);
      struct tm time = getDateTime();
      char message[40];
      if (home.slave_state[id])
      {
        strftime(message, sizeof(message), "%H:%M \n OPEN", &time);
      }
      else
      {
        strftime(message, sizeof(message), "%H:%M \n CLOSE", &time);
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

void TaskConnection(void *pvParameters)
{
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 700 / portTICK_PERIOD_MS;
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
  const TickType_t xFrequency = 300 / portTICK_PERIOD_MS;
  // Initialise the xLastWakeTime variable with the current time.
  xLastWakeTime = xTaskGetTickCount();

  Serial.print("Created Task Main on core: ");
  Serial.println(xPortGetCoreID());

  while (1)
  {
    if (client.connected() && ACK) /* ACK => Nuovo messaggio */
    {
      if (strcmp(topic_id, topic_alarm_sound) == 0) /* Verifica disattivazione suono allarme */
      {
        xSemaphoreTake(mutex_home, portMAX_DELAY);
        if (home.alarm_mode && home.alarm_sound)
        {
          home.alarm_mode = DISABLED;
          xSemaphoreGive(mutex_home);
          client.publish(topic_alarm_received, "Sound and alarm disabled");
          xSemaphoreGiveFromISR(mutex_alarm, 0);
          xTimerStop(Timer_Alarm, 0);
          xTimerStop(Timer_MovementDetection, 0);
          xSemaphoreTake(mutex_lcd, portMAX_DELAY);
          lcd.setCursor(0, 0);
          lcd.clear();
          lcd.print("Sound and alarm");
          lcd.setCursor(0, 1);
          lcd.print("disabled");
          vTaskDelay(pdMS_TO_TICKS(1000));
          xSemaphoreGive(mutex_lcd);
        }
        else if (!home.alarm_mode)
        {
          xSemaphoreGive(mutex_home);
          client.publish(topic_alarm_received, "Alarm is disabled!");
          xSemaphoreTake(mutex_lcd, portMAX_DELAY);
          lcd.setCursor(0, 0);
          lcd.clear();
          lcd.print("Alarm already");
          lcd.setCursor(0, 1);
          lcd.print("disabled!");
          vTaskDelay(pdMS_TO_TICKS(1000));
          xSemaphoreGive(mutex_lcd);
        }
        else if (home.alarm_mode && !home.alarm_sound)
        {
          xSemaphoreGive(mutex_home);
          client.publish(topic_alarm_received, "Sound is off!");
          xSemaphoreTake(mutex_lcd, portMAX_DELAY);
          lcd.setCursor(0, 0);
          lcd.clear();
          lcd.print("Sound is off!");
          vTaskDelay(pdMS_TO_TICKS(1000));
          xSemaphoreGive(mutex_lcd);
        }
      }
      else if (strcmp(topic_id, topic_alarm_mode_on) == 0) /* Verifica abilitazione allarme */
      {
        xSemaphoreTake(mutex_home, portMAX_DELAY);
        if (!home.alarm_mode && !home.alarm_sound && home.open_slaves == 0)
        {
          home.alarm_mode = ENABLED;
          xSemaphoreGive(mutex_home);
          client.publish(topic_alarm_received, topic_payload);
          xTimerStart(Timer_MovementDetection, 0);
          xSemaphoreTake(mutex_lcd, portMAX_DELAY);
          lcd.setCursor(0, 0);
          lcd.clear();
          lcd.print("Alarm enabled");
          vTaskDelay(pdMS_TO_TICKS(1000));
          xSemaphoreGive(mutex_lcd);
        }
        else if (home.open_slaves)
        {
          xSemaphoreGive(mutex_home);
          client.publish(topic_alarm_received, "Slaves open, can't enable alarm");
          xSemaphoreTake(mutex_lcd, portMAX_DELAY);
          lcd.setCursor(0, 0);
          lcd.clear();
          lcd.print("Slaves open");
          lcd.setCursor(0, 1);
          lcd.print("can't enable alarm");
          vTaskDelay(pdMS_TO_TICKS(1000));
          xSemaphoreGive(mutex_lcd);
        }
        else if (home.alarm_mode)
        {
          xSemaphoreGive(mutex_home);
          client.publish(topic_alarm_received, "Alarm already enabled!");
          xSemaphoreTake(mutex_lcd, portMAX_DELAY);
          lcd.setCursor(0, 0);
          lcd.clear();
          lcd.print("Alarm already");
          lcd.setCursor(0, 1);
          lcd.print("enabled!");
          vTaskDelay(pdMS_TO_TICKS(1000));
          xSemaphoreGive(mutex_lcd);
        }
        else if (home.alarm_sound)
        {
          xSemaphoreGive(mutex_home);
          client.publish(topic_alarm_received, "Alarm sound is ON!");
          xSemaphoreTake(mutex_lcd, portMAX_DELAY);
          lcd.setCursor(0, 0);
          lcd.clear();
          lcd.print("Alarm sound");
          lcd.setCursor(0, 1);
          lcd.print("is ON!");
          vTaskDelay(pdMS_TO_TICKS(1000));
          xSemaphoreGive(mutex_lcd);
        }
      }
      else if (strcmp(topic_id, topic_alarm_mode_off) == 0) /* Verifica disabilitazione allarme */
      {
        xSemaphoreTake(mutex_home, portMAX_DELAY);
        if (home.alarm_mode && !home.alarm_sound)
        {
          home.alarm_mode = DISABLED;
          xSemaphoreGive(mutex_home);
          client.publish(topic_alarm_received, topic_payload);
          xTimerStop(Timer_MovementDetection, portMAX_DELAY);
          Serial.println(topic_payload);
          xSemaphoreTake(mutex_lcd, portMAX_DELAY);
          lcd.setCursor(0, 0);
          lcd.clear();
          lcd.print("Alarm disabled");
          vTaskDelay(pdMS_TO_TICKS(1000));
          xSemaphoreGive(mutex_lcd);
        }
        else if (!home.alarm_mode)
        {
          xSemaphoreGive(mutex_home);
          client.publish(topic_alarm_received, "Alarm already disabled!");
          xSemaphoreTake(mutex_lcd, portMAX_DELAY);
          lcd.setCursor(0, 0);
          lcd.clear();
          lcd.print("Alarm already");
          lcd.setCursor(0, 1);
          lcd.print("disabled!");
          vTaskDelay(pdMS_TO_TICKS(1000));
          xSemaphoreGive(mutex_lcd);
        }
        else if (home.alarm_sound)
        {
          xSemaphoreGive(mutex_home);
          client.publish(topic_alarm_received, "Alarm sound is on!");
          xSemaphoreTake(mutex_lcd, portMAX_DELAY);
          lcd.setCursor(0, 0);
          lcd.clear();
          lcd.print("Alarm sound");
          lcd.setCursor(0, 1);
          lcd.print("is on!");
          vTaskDelay(pdMS_TO_TICKS(1000));
          xSemaphoreGive(mutex_lcd);
        }
      }
      else if (strcmp(topic_id, topic_motion_detection_code) == 0) /* Verifica disabilitazione Timer allarme tramite Codice */
      {
        Serial.println(topic_payload);
        stopTimerAndDisableAlarm();
        xSemaphoreTake(mutex_lcd, portMAX_DELAY);
        lcd.setCursor(0, 0);
        lcd.clear();
        lcd.print("PASSWORD OK");
        vTaskDelay(pdMS_TO_TICKS(1000));
        xSemaphoreGive(mutex_lcd);
        xSemaphoreTake(mutex_password, portMAX_DELAY);
        insert_password = false;
        xSemaphoreGive(mutex_password);
        vTaskResume(Handle_TaskDisplay);
        vTaskDelete(Handle_TaskPassword);
        client.publish(topic_alarm_received, "Timer stopped and alarm disabled with App code");
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

void TaskMovementDetection(void *pvParameters)
{

  Serial.print("Created movement detection Task on core: ");
  Serial.println(xPortGetCoreID());

  Ultrasonic ultrasonic(MOV_TRIG, MOV_ECHO);

  xTimerStart(Timer_MovementDetection, 0);
  int val;

  while (1)
  {
    xSemaphoreTake(mutex_movement, portMAX_DELAY);
    if (home.alarm_mode && !home.alarm_sound)
    {
      xSemaphoreGive(mutex_home);
      if (!first_read)
      {
        val = ultrasonic.distanceRead();
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
        if (val != 0)
        {
          curr_distance = val;
        }
        if ((curr_distance - last_distance) > MAX_DISTANCE || (curr_distance - last_distance) < -MAX_DISTANCE)
        {
          Serial.println("MOVEMENT DETECTED");
          xSemaphoreTake(mutex_lcd, portMAX_DELAY);
          lcd.setCursor(0, 0);
          lcd.clear();
          lcd.print("** MOVEMENT **");
          lcd.setCursor(0, 1);
          lcd.print("** DETECTED **");
          vTaskDelay(pdMS_TO_TICKS(1000));
          xSemaphoreGive(mutex_lcd);
          xSemaphoreTake(mutex_home, portMAX_DELAY);
          if (!home.alarm_sound)
          {
            xSemaphoreGive(mutex_home);
            xSemaphoreTake(mutex_password, portMAX_DELAY);
            insert_password = true;
            xSemaphoreGive(mutex_password);
            /* Creazione task inserimento codice*/
            xTaskCreate(
                TaskPassword,
                "Password Task",
                3000,
                NULL,
                10,
                &Handle_TaskPassword);
            xTimerStart(Timer_Code, 0);
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
  // Initialise the xLastWakeTime variable with the current time.
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
      snprintf(value, sizeof(value), "%d", home.open_slaves);
      xSemaphoreGive(mutex_home);
      xSemaphoreTake(mutex_lcd, portMAX_DELAY);
      lcd.setCursor(0, 0);
      lcd.clear();
      lcd.print("ALARM: ON");
      lcd.setCursor(0, 1);
      lcd.print("OPEN SLAVES: ");
      lcd.print(value);
      xSemaphoreGive(mutex_lcd);
    }
    else if (!home.alarm_mode && !home.alarm_sound)
    {
      char value[5];
      snprintf(value, sizeof(value), "%d", home.open_slaves);
      xSemaphoreGive(mutex_home);
      xSemaphoreTake(mutex_lcd, portMAX_DELAY);
      lcd.setCursor(0, 0);
      lcd.clear();
      lcd.print("ALARM: OFF");
      lcd.setCursor(0, 1);
      lcd.print("OPEN SLAVES: ");
      lcd.print(value);
      xSemaphoreGive(mutex_lcd);
    }
    else if (home.alarm_sound)
    {
      xSemaphoreGive(mutex_home);
      xSemaphoreTake(mutex_lcd, portMAX_DELAY);
      lcd.setCursor(0, 0);
      lcd.print("** ATTENTION **");
      lcd.setCursor(0, 1);
      lcd.print("ALARM SOUND ON!!");
      xSemaphoreGive(mutex_lcd);
    }

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void TaskPassword(void *pvParameters)
{
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 700 / portTICK_PERIOD_MS;
  // Initialise the xLastWakeTime variable with the current time.
  xLastWakeTime = xTaskGetTickCount();

  Serial.print("Created Password Task on core: ");
  Serial.println(xPortGetCoreID());

  IrReceiver.begin(IR_Recv, DISABLE_LED_FEEDBACK);
  int i = 0;

  xSemaphoreTake(mutex_lcd, portMAX_DELAY);
  lcd.setCursor(0, 0);
  lcd.clear();
  lcd.print("INSERT CODE:");
  lcd.setCursor(0, 1);
  xSemaphoreGive(mutex_lcd);

  while (1)
  {
    //decodes the infrared input
    if (IrReceiver.decode())
    {
      //Serial.println(IrReceiver.decodedIRData.decodedRawData, HEX);
      if (i < PASS_SIZE - 1)
      {
        xSemaphoreTake(mutex_lcd, portMAX_DELAY);
        switch (IrReceiver.decodedIRData.decodedRawData)
        {
        case 0xE916FF00: // 0 button
          Serial.print("0");
          lcd.print("0");
          try_password[i] = '0';
          i++;
          break;
        case 0xF30CFF00: // 1 button
          Serial.print("1");
          lcd.print("1");
          try_password[i] = '1';
          i++;
          break;
        case 0xE718FF00: // 2 button
          Serial.print("2");
          lcd.print("2");
          try_password[i] = '2';
          i++;
          break;
        case 0xA15EFF00: // 3 button
          Serial.print("3");
          lcd.print("3");
          try_password[i] = '3';
          i++;
          break;
        case 0xF708FF00: // 4 button
          Serial.print("4");
          lcd.print("4");
          try_password[i] = '4';
          i++;
          break;
        case 0xE31CFF00: // 5 button
          Serial.print("5");
          lcd.print("5");
          try_password[i] = '5';
          i++;
          break;
        case 0xA55AFF00: // 6 button
          Serial.print("6");
          lcd.print("6");
          try_password[i] = '6';
          i++;
          break;
        case 0xBD42FF00: // 7 button
          Serial.print("7");
          lcd.print("7");
          try_password[i] = '7';
          i++;
          break;
        case 0xAD52FF00: // 8 button
          Serial.print("8");
          lcd.print("8");
          try_password[i] = '8';
          i++;
          break;
        case 0xB54AFF00: // 9 button
          Serial.print("9");
          lcd.print("9");
          try_password[i] = '9';
          i++;
          break;
        case 0xBC43FF00: // RESET button
          Serial.println();
          Serial.println("RESET");
          lcd.clear();
          lcd.print("RESET");
          vTaskDelay(pdMS_TO_TICKS(500));
          lcd.setCursor(0, 0);
          lcd.clear();
          lcd.print("INSERT CODE:");
          lcd.setCursor(0, 1);
          memset(try_password, 0, sizeof(try_password));
          i = 0;
          break;
        default:
          Serial.println("input not valid");
        }
        xSemaphoreGive(mutex_lcd);
      }
      else if (i == PASS_SIZE - 1)
      {
        xSemaphoreTake(mutex_lcd, portMAX_DELAY);
        switch (IrReceiver.decodedIRData.decodedRawData)
        {
        case 0xF609FF00: // OK button
          Serial.println();
          Serial.println(" - OK");
          exit_while = true;
          break;
        case 0xBC43FF00: // RESET button
          Serial.println();
          Serial.println("RESET");
          lcd.setCursor(0, 0);
          lcd.clear();
          lcd.print("RESET");
          vTaskDelay(pdMS_TO_TICKS(500));
          lcd.setCursor(0, 0);
          lcd.clear();
          lcd.print("INSERT CODE:");
          lcd.setCursor(0, 1);
          memset(try_password, 0, sizeof(try_password));
          i = 0;
          break;
        default:
          Serial.println();
          Serial.println("Error password size, please check or reset.");
          lcd.setCursor(0, 0);
          lcd.clear();
          lcd.print("Error size");
          vTaskDelay(pdMS_TO_TICKS(500));
          lcd.setCursor(0, 0);
          lcd.clear();
          lcd.print("INSERT CODE:");
          lcd.setCursor(0, 1);
        }
        xSemaphoreGive(mutex_lcd);
      }
      IrReceiver.resume(); // Receives the next value from the button you press
    }

    if (exit_while)
    {
      Serial.print("Password: ");
      Serial.println(try_password);
      Serial.println("Break dal while");
      break;
    }

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }

  Serial.println("Check password..");
  exit_while = false;

  /* CHECK PASSWORD MATCHING */
  if (strcmp(try_password, real_password) == 0)
  {
    Serial.println("PASSWORD OK!");
    xSemaphoreTake(mutex_lcd, portMAX_DELAY);
    lcd.setCursor(0, 0);
    lcd.clear();
    lcd.print("PASSWORD OK");
    vTaskDelay(pdMS_TO_TICKS(1000));
    xSemaphoreGive(mutex_lcd);
    stopTimerAndDisableAlarm();
    client.publish(topic_alarm_received, "Timer stopped and Alarm disabled");
    xSemaphoreTake(mutex_password, portMAX_DELAY);
    insert_password = false;
    xSemaphoreGive(mutex_password);
    vTaskResume(Handle_TaskDisplay);
    vTaskDelete(NULL);
  }
  else
  {
    Serial.println("PASSWORD FAIL!");
    xSemaphoreTake(mutex_lcd, portMAX_DELAY);
    lcd.setCursor(0, 0);
    lcd.clear();
    lcd.print("PASSWORD FAIL");
    xSemaphoreGive(mutex_lcd);
    vTaskSuspend(NULL);
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
  if (client.connect(clientid))
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
    if (client.connect(clientid))
    {
      Serial.println("Reconnected to MQTT");
      subscriptions();
    }
  }
}

void subscriptions()
{
  client.subscribe(topic_alarm_sound);
  client.subscribe(topic_alarm_mode_on);
  client.subscribe(topic_alarm_mode_off);
  client.subscribe(topic_motion_detection_code);
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

void stopTimerAndDisableAlarm()
{
  TickType_t xRemainingTime;
  xRemainingTime = xTimerGetExpiryTime(Timer_Code) - xTaskGetTickCount();
  Serial.print("Remaining time to activate alarm: ");
  Serial.println(xRemainingTime);
  xTimerStop(Timer_Code, 0);
  xTimerStop(Timer_MovementDetection, 0);
  xSemaphoreTake(mutex_home, portMAX_DELAY);
  home.alarm_mode = DISABLED;
  xSemaphoreGive(mutex_home);
  Serial.println("Timer stopped and Alarm Disabled");
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
    vTaskDelete(Handle_TaskPassword);
    xSemaphoreGiveFromISR(mutex_alarm, 0);
  }
  else if (xTimer == Timer_Alarm)
  {
    xSemaphoreGiveFromISR(mutex_alarm, 0);
  }
}

void loop()
{
  // put your main code here, to run repeatedly:
}

/* DEBUG dimensione STACK dei task */
// Serial.print(pcTaskGetTaskName(NULL));
// Serial.print(" uxTaskGetStackHighWaterMark = ")
// Serial.println(uxTaskGetStackHighWaterMark(NULL));
