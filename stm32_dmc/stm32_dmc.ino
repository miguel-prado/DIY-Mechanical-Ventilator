#include <MapleFreeRTOS900.h>
#include "HardwareTimer.h"
#include "pid.h"
#include "profile.h"

#define PPR   1024
#define LED_BUITLIN PC13
#define PWM_PIN PB8
#define DIR_PIN PB9
#define SAMPLES 1500

HardwareTimer counterTimer(2);
HardwareTimer TimerC(4);
unsigned long ints = 0;
long reference = 0;
long response = 0;
long despos = 0;
long desvel = 0;
PID_Controller pid(&reference, &response);
Profile profile;
unsigned int idx = SAMPLES;
long samples[SAMPLES];

/* Command parser variables */
String inputString = "";
String command = "";
String argument = "";

static void blinkTask(void *pvParameters);
static void mainTask(void *pvParameters);
static void commandTask(void *pvParameters);
void updateCounter(void);
void parseCommand(void);

void setup()
{
  Serial.begin(115200);

  inputString.reserve(64);
  command.reserve(32);
  argument.reserve(32);

  pinMode(PA0, INPUT_PULLUP);
  pinMode(PA1, INPUT_PULLUP);
  pinMode(LIM_PIN, INPUT_PULLDOWN);
    
  counterTimer.setMode(0, TIMER_ENCODER);
  counterTimer.pause();
  counterTimer.setPrescaleFactor(1);
  counterTimer.setOverflow(PPR);
  counterTimer.setCount(0);
  counterTimer.setEdgeCounting(TIMER_SMCR_SMS_ENCODER3);
  counterTimer.attachInterrupt(0, updateCounter);
  counterTimer.resume();

  TimerC.pause();
  TimerC.setPrescaleFactor(1);
  TimerC.setOverflow(3000);
  TimerC.resume();

  xTaskCreate(blinkTask, "Blink", 32, NULL, 1, NULL);
  xTaskCreate(mainTask, "Control", 512, NULL, 5, NULL);
  xTaskCreate(commandTask, "Interface", 1024, NULL, 1, NULL);
  vTaskStartScheduler();
}

void loop()
{  
}

static void mainTask(void *pvParameters)
{
  int limit_state = 0;
  pinMode(PWM_PIN, PWM);
  pinMode(DIR_PIN, OUTPUT);
  while(1)
  {
    response = ints * PPR + counterTimer.getCount();
    if (idx < SAMPLES) samples[idx++] = response;
    pid.compute();
    profile.execute();
    reference = (long)profile.position();
    int out = int(300.0 * pid.output());
    writeOutput(out);

    if (!limit_state && digitalRead(LIM_PIN))
    {
      writeOutput(0);
      pid.reset();
      profile.abort();
      profile.setStep(0);
      reference = 0;
      response = 0;
      counterTimer.setCount(0);
      ints = 0;
    }
    limit_state = digitalRead(LIM_PIN);
    vTaskDelay(1);
  }
}

static void commandTask(void *pvParameters)
{
  while(1)
  {
    while (Serial.available())
    {
      char inChar = (char)Serial.read();
      if (inChar == '\r')
        continue;
      if (inChar == '\n')
      {
        parseCommand();
        inputString = "";
      }
      else
        inputString += inChar;
      vTaskDelay(1);
    }
    vTaskDelay(10);
  }
}

static void blinkTask(void *pvParameters)
{
  pinMode(LED_BUITLIN, OUTPUT);
  while(1)
  {
    vTaskDelay(500);
    digitalWrite(LED_BUITLIN, HIGH);
    vTaskDelay(500);
    digitalWrite(LED_BUITLIN, LOW);
  }
}

void updateCounter(void)
{
  if (counterTimer.getDirection())
    ints--;
  else
    ints++;
}

void parseCommand(void)
{
  Serial.print(": ");
  if (inputString == "RS")
  {
    pid.reset();
    reference = 0;
    ints = 0;
    counterTimer.setCount(0);
    vTaskDelay(100);
  }
  if (inputString == "TP") /* Tell position */
  {
    Serial.println(response);
    return;
  }
  if (inputString == "MV")
  {
    Serial.println(profile.isMoving());
    return;
  }
  if (inputString == "TV") /* Tell velocity */
  {
    Serial.println(pid.velocity());
    return;
  }
  if (inputString == "TE") /* Tell error */
  {
    Serial.println(pid.error());
    return;
  }
  if (inputString == "TT") /* Tell torque */
  {
    Serial.println(pid.output());
    return;
  }
  if (inputString == "BG") /* Begin movement */
  {
    profile.startMotion(reference);
    idx = 0;
    Serial.println("OK");
    return;
  }
  if (inputString == "SR") /* Send response */
  {
    sendResponse();
    return;
  }
  vTaskDelay(10);
  /* Command contains data argument */
  int space = inputString.indexOf(' ');
  if (space < 0)
    return;
  command = inputString.substring(0, space);
  argument = inputString.substring(space + 1, inputString.length());
  if (command == "REF") /* Set point */
  {
    if (argument == "?")
    {
      Serial.println(reference);
      return;
    }
    else
    {
      profile.setStep(argument.toInt());
      idx = 0;
      Serial.println("OK");
      return;
    }
  }
  if (command == "KP")
  {
    if (argument == "?")
    {
      Serial.println(pid.Kp(), 6);
      return;
    }
    else
    {
      pid.setKp(argument.toFloat());
      Serial.println("OK");
      return;
    }
  }
  if (command == "TI")
  {
    if (argument == "?")
    {
      Serial.println(pid.Ti(), 6);
      return;
    }
    else
    {
      pid.setTi(argument.toFloat());
      Serial.println("OK");
      return;
    }
  }
  if (command == "TD")
  {
    if (argument == "?")
    {
      Serial.println(pid.Td(), 6);
      return;
    }
    else
    {
      pid.setTd(argument.toFloat());
      Serial.println("OK");
      return;
    }
  }
  if (command == "PR") /* Position relative */
  {
    profile.setPosition((long)argument.toInt());
    Serial.println("OK");
    return;
  }
  if (command == "SP") /* Speed */
  {
    profile.setVelocity((int)argument.toInt());
    Serial.println("OK");
    return;
  }
}

void sendResponse(void)
{
  for (int k = 0; k < SAMPLES; k++)
  {
    Serial.println(samples[k]);
    vTaskDelay(1);
  }
}

void writeOutput(int outlet)
{
  if (outlet >= 0)
    digitalWrite(DIR_PIN, LOW);
  else
    digitalWrite(DIR_PIN, HIGH);
  int absVal = abs(outlet);
  pwmWrite(PWM_PIN, absVal);
}
