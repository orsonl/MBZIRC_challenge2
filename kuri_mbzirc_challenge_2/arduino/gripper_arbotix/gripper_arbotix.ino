#include <ax12.h> //include the ArbotiX DYNAMIXEL library

const int SERVO_ID = 1;
int fsr_threshold = 700;

int fsrPin = A0;     // the FSR and 10K pulldown are connected to a0
int switchPin = 16;

int fsrReading;     // the analog reading from the FSR resistor divider
bool is_pressed = false;

double distance = 0;
double time_to_100 = 28000.0;
double delay_time = 50.0;
int state = 0;

void setup()
{
  Serial.begin(9600);

  pinMode(switchPin, INPUT_PULLUP);      // sets the digital pin as output
  
  dxlInit(1000000);  //start dynamixel library at 1mbps to communicate with the servos
  
  int dxlMode = dxlGetMode(SERVO_ID); //get the mode for servo # SERVO_ID

  //check if the servo's mode. Only set joint mode if the mode is not already in joint mode. This helps to preserve the lifespan of the EEPROM memory on the servo
  if(dxlMode != WHEEL_MODE)
  {
    mxSetWheelMode(SERVO_ID);
  }


  // Wait until we have an input to start (sometimes it goes crazy when it loses power)
  while (1)
  {
    char input = Serial.read();
    if (input == '0')
      break;

    // Indicate that we are not initialized
    Serial.println(-1);
  }

  // Put the hand to the starting position (fully open)
  while (1)
  {
    is_pressed = !digitalRead(switchPin);

    if (is_pressed)
    {
      stopGripper();
      distance = 0;
      break;
    }

    distance = 0.5*time_to_100;
    openGripper();
  }
  
}

void stopGripper()
{
  // OFF
  dxlSetGoalSpeed(SERVO_ID, 0);
  delay(delay_time);
  state = 0;
}

void openGripper()
{
  if (distance > 0)
  {
    // CLOCKWISE
    dxlSetGoalSpeed(SERVO_ID, 1023);
    delay(delay_time);
    distance -= delay_time;
  }
  else
  {
    stopGripper();
  }
}

void forceOpen()
{
  // CLOCKWISE
  dxlSetGoalSpeed(SERVO_ID, 1023);
  delay(delay_time);
  distance -= delay_time;
}

void closeGripper()
{
  // COUNTER-CLOCKWISE
  if (distance < time_to_100 && fsrReading < fsr_threshold)
  {
    dxlSetGoalSpeed(SERVO_ID, 2047);
    delay(delay_time);
    distance += delay_time;
  }
  else
  {
    stopGripper();
  }
}



void loop()
{
  fsrReading = analogRead(fsrPin);
  double percent = distance/time_to_100;
  
  Serial.print(fsrReading);
  Serial.print(',');
  Serial.println(percent);

  
  is_pressed = !digitalRead(switchPin);
  if (is_pressed)
  {
    distance = 0;
  }

  char input = Serial.read();
  if (input == '0')
  {
    state = 0;
  }
  else if (input == '1')
  {
    state = 1;
  }
  else if (input == '2')
  {
    state = 2;
  }
  else if (input == '3')
  {
    state = 3;
  }


  switch(state)
  {
    case 0:
      stopGripper();
      break;

    case 1:
      openGripper();
      break;

    case 2:
      closeGripper();
      break;
    case 3:
      forceOpen();
      break;
  }

 
  //do nothing  
}



