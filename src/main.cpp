#include <Arduino.h>
#include <PID_v1.h>
#include "RPi_Pico_TimerInterrupt.h"
#include <RotaryEncoder.h>

#define TIMER_INTERRUPT_DEBUG 1
#define _TIMERINTERRUPT_LOGLEVEL_ 4
#define _CPU_STATS_ 0

struct BusData
{
  int16_t motor_1 = 0;
  int16_t motor_2;
  int16_t motor_3;
  int8_t led_status;
};
BusData DriveData;

// Create a place to hold incoming uart data
const uint8_t MAX_MSG_LENGTH = 6;
char DATA[MAX_MSG_LENGTH];
uint8_t msg_pos = 0;

//************************* Motor Driver **********************************

class Motor_cytron_md30c
{
public:
  int pwm;
  int _dir_pin;
  int _pwm_pin;

  Motor_cytron_md30c(int pwm_pin, int dir_pin)
  {
    _dir_pin = dir_pin;
    _pwm_pin = pwm_pin;
    pinMode(_dir_pin, OUTPUT_8MA);
    analogWrite(pwm_pin, 0);
  }

  void setPWM(int PWM)
  {
    if (PWM > 0)
    {
      digitalWrite(_dir_pin, 1);
      analogWrite(_pwm_pin, PWM);
    }
    else if (PWM < 0)
    {
      digitalWrite(_dir_pin, 0);
      analogWrite(_pwm_pin, abs(PWM));
    }
    else
    {
      digitalWrite(_dir_pin, 0);
      analogWrite(_pwm_pin, 0);
    }
  }
};

//*************************************************************************

// Motor_cytron_md30c motor(3, 2);

//************************* BTS motor Driver ******************************
class BTS7890
{
private:
  int _PWM_1;
  int _PWM_2;

public:
  BTS7890(int PWM_1, int PWM_2)
  {
    /** Driver code for BTS790...  */

    _PWM_1 = PWM_1;
    _PWM_2 = PWM_2;

    analogWrite(_PWM_1, 0);
    analogWrite(_PWM_2, 0);
  }
  void setPWM(int pwm_value)
  {
    if (pwm_value > 0)
    {
      analogWrite(_PWM_1, 0); // Stop the other direction
      analogWrite(_PWM_2, pwm_value);
    }
    else if (pwm_value < 0)
    {
      analogWrite(_PWM_1, -(pwm_value)); // Remove the negetive sign
      analogWrite(_PWM_2, 0);
    }
    else
    {
      // If pwm is 0 turn on all low
      analogWrite(_PWM_1, 0);
      analogWrite(_PWM_2, 0);
    }
  }
};

//*************************************************************************

BTS7890 motor_1(3, 2);
BTS7890 motor_2(6, 7);
BTS7890 motor_3(8, 9);

//************************* Motor Encoder Constant ************************

#define PPR 1300
#define RPM_TIMER_INTERVAL_MS 20 // How frequently RPM should be calculated

#define ENCODER_2_IN1 26
#define ENCODER_2_IN2 22

#define ENCODER_3_IN1 21
#define ENCODER_3_IN2 20

#define ENCODER_IN1 28
#define ENCODER_IN2 27

volatile long int encoder_ticks = 0;
volatile long previous_encoder_ticks = 0;
volatile long RPM = 0;

RotaryEncoder *encoder = nullptr;   // Damm a good methord to make instace var
RotaryEncoder *encoder_2 = nullptr; // Damm a good methord to make instace var
RotaryEncoder *encoder_3 = nullptr; // Damm a good methord to make instace var
// Solves the OLED display error

void checkPosition()
{
  encoder->tick(); // just call tick() to check the state.
}
void checkPosition_2()
{
  encoder_2->tick(); // just call tick() to check the state.
}
void checkPosition_3()
{
  encoder_3->tick(); // just call tick() to check the state.
}

//*************************************************************************

//************************** PID Constants ********************************

double Setpoint, Input, Output;
double Kp = 1.3, Ki = 0.9, Kd = 0.2;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

//*************************************************************************

//***************************** PICO timer ********************************

RPI_PICO_Timer ITimer(0); // Create a single timer instance

// If the timer callback return false it will stop calling it self again
bool TimerHandler_Calculate_RPM(struct repeating_timer *t)
{
  (void)t; // This line is used to suppress unused parameter warnings

  long int curent_ticks = encoder->getPosition();
  long int curent_ticks_2 = encoder_2->getPosition();
  long int curent_ticks_3 = encoder_3->getPosition();

  Serial.print(curent_ticks);
  Serial.print(" ");
  Serial.print(curent_ticks_2);
  Serial.print(" ");
  Serial.println(curent_ticks_3);

  encoder_ticks = curent_ticks;

  RPM = ((curent_ticks - previous_encoder_ticks) * 60 * 1000) / PPR / RPM_TIMER_INTERVAL_MS;
  previous_encoder_ticks = curent_ticks;

  return true; // Countinously run
}
//*************************************************************************

void setup()
{
  Serial.setTimeout(2);
  Serial.begin(115200);

  encoder = new RotaryEncoder(ENCODER_IN1, ENCODER_IN2, RotaryEncoder::LatchMode::TWO03);
  attachInterrupt(digitalPinToInterrupt(ENCODER_IN1), checkPosition, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_IN2), checkPosition, CHANGE);

  encoder_2 = new RotaryEncoder(ENCODER_2_IN1, ENCODER_2_IN2, RotaryEncoder::LatchMode::TWO03);
  attachInterrupt(digitalPinToInterrupt(ENCODER_2_IN1), checkPosition_2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_2_IN2), checkPosition_2, CHANGE);

  encoder_3 = new RotaryEncoder(ENCODER_3_IN1, ENCODER_3_IN2, RotaryEncoder::LatchMode::TWO03);
  attachInterrupt(digitalPinToInterrupt(ENCODER_3_IN1), checkPosition_3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_3_IN2), checkPosition_3, CHANGE);

  // turn the PID on
  myPID.SetMode(AUTOMATIC);

  // Attach the timer interrupt with a 50ms interval
  if (ITimer.attachInterruptInterval(RPM_TIMER_INTERVAL_MS * 1000, TimerHandler_Calculate_RPM))
    ;
  // Serial.println(millis());
  else
    Serial.println(F("Can't set Timer Callback. Select another Timer, freq. or timer"));

  Setpoint = 43;
}

void update_struct(char *DATA)
{
  //******** Process data ********
  // Get First byte of msg which identifyes the motor which this this data is for
  char Id = DATA[0];

  // Get the pwm
  const int pwm_digit_len = 5; // 0000 - 2047
  char pwm[pwm_digit_len];
  strcpy(pwm, DATA + 1);

  // Char array to int conversion
  // Note:End of char array should have null char ie "\0" for atoi() to work
  pwm[pwm_digit_len] = '\0';
  int int_pwm = atoi(pwm);

  //******** Update Structure ********
  switch (Id)
  {
  case '1':
    DriveData.motor_1 = int_pwm - 1023;
    break;
  case '2':
    DriveData.motor_2 = int_pwm - 1023;
    break;
  case '3':
    DriveData.motor_3 = int_pwm - 1023;
    break;
  case 'l':
    DriveData.led_status = 0;
    break;

  default:
    break;
  }
}
void loop()
{

  // just call tick() to check the state.
  encoder->tick();
  encoder_2->tick();

  if (Serial.available() > 0)
  {
    char recived_byte = Serial.read();
    if (recived_byte != '\n')
    {
      DATA[msg_pos] = recived_byte;
      msg_pos++;
    }
    else
    {
      msg_pos = 0;
      update_struct(DATA);
      for (uint8_t i = 0; i < 6; i++) // Empty the DATA
        DATA[i] = '\0';
    }
  }
  motor_1.setPWM(DriveData.motor_1 / 4);
  motor_2.setPWM(DriveData.motor_2 / 4);
  motor_3.setPWM(DriveData.motor_3 / 4);

  // Serial.println("Encoder Ticks");
  // Serial.println(encoder_ticks);
  // Serial.println("---");
}

void print_data()
{
  Serial.println(DriveData.motor_1);
  Serial.println(DriveData.motor_2);
  Serial.println(DriveData.motor_3);
}

// Input = RPM;
// myPID.Compute();
// Serial.print("Input = ");
// Serial.println(Input);
// Serial.print("Output = ");
// Serial.println(Output);
// Serial.print("Setpoint = ");
// Serial.println(Setpoint);
// Serial.println("---");