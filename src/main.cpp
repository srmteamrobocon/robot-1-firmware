#include <Arduino.h>
#include <PID_v1.h>
#include "RPi_Pico_TimerInterrupt.h"
#include <RotaryEncoder.h>
#include "Commands.h"
#include "Bts.h"
#include "Cytron_MD30C.h"

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

// BTS7890 motor_1(12, 14);
// BTS7890 motor_2(6, 7);
// BTS7890 motor_3(8, 9);

Motor_cytron_md30c motor(6, 7);

//************************* Motor Encoder Constant ************************

#define PPR 1300
#define RPM_TIMER_INTERVAL_MS 20 // How frequently RPM should be calculated

#define ENCODER_2_IN1 26
#define ENCODER_2_IN2 22

#define ENCODER_3_IN1 21
#define ENCODER_3_IN2 20

#define ENCODER_IN1 3 // 28
#define ENCODER_IN2 2 // 27

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

  // Serial.print(curent_ticks);
  // Serial.print(" ");
  // Serial.print(curent_ticks_2);
  // Serial.print(" ");
  // Serial.println(curent_ticks_3);

  encoder_ticks = curent_ticks;

  RPM = ((curent_ticks - previous_encoder_ticks) * 60 * 1000) / PPR / RPM_TIMER_INTERVAL_MS;
  previous_encoder_ticks = curent_ticks;

  return true; // Countinously run
}
//*************************************************************************

// void setPwmToMotors();

void run_commands(char COMMAND)
{
  switch (COMMAND)
  {
  case MOTOR_RAW_PWM:
    // Adding braces removes jump case error
    {
      char buffer_motor_1_PWM[5];
      char buffer_motor_2_PWM[5];
      char buffer_motor_3_PWM[5];

      Serial.readBytesUntil(',', buffer_motor_1_PWM, sizeof(buffer_motor_1_PWM));
      Serial.readBytesUntil(',', buffer_motor_2_PWM, sizeof(buffer_motor_2_PWM));
      Serial.readBytesUntil(',', buffer_motor_3_PWM, sizeof(buffer_motor_3_PWM));

      // Not sure about its use case
      char endBiyte = Serial.read();

      // Convert valid char array to Intiger
      int PWM_1 = atoi(buffer_motor_1_PWM);
      int PWM_2 = atoi(buffer_motor_2_PWM);
      int PWM_3 = atoi(buffer_motor_3_PWM);

      motor.setPWM(PWM_1);

      // Serial.print(" Pwm 1 ");
      // Serial.print(PWM_1);
      // Serial.print(" Pwm 2 ");
      // Serial.print(PWM_2);
      // Serial.print(" Pwm 3 ");
      // Serial.println(PWM_3);
      break;
    }

  case READ_ENCODERS:
  {
    Serial.println(encoder_ticks);
    break;
  }
  }
}

void setup()
{
  Serial.setTimeout(1000);
  Serial.begin(115200);
}

void loop()
{
  if (Serial.available() > 0)
  {
    byte cmd = Serial.read();
    run_commands(cmd);
  }
  // motor_1.setPWM(DriveData.motor_1 / 4);
  // motor_2.setPWM(DriveData.motor_2 / 4);
  // motor_3.setPWM(DriveData.motor_3 / 4);
}

// RP2040 Second core
void setup1()
{

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

void loop1()
{
  encoder->tick();
  encoder_2->tick();
  encoder_3->tick();

  // just call tick() to check the state.

  // Serial.println("Core 2");
  //  Serial.println("Encoder Ticks");
  // Serial.println("---");
  // Serial.println("Core 1");
  // delay(1000);
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