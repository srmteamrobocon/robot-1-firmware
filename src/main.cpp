#include <Arduino.h>
#include <PID_v1.h>
#include "RPi_Pico_TimerInterrupt.h"
#include <RotaryEncoder.h>

#define TIMER_INTERRUPT_DEBUG 1
#define _TIMERINTERRUPT_LOGLEVEL_ 4
#define _CPU_STATS_ 0

//************************* Motor Encoder Constant ************************

#define PPR 1300
#define RPM_TIMER_INTERVAL_MS 20 // How frequently RPM should be calculated

#define ENCODER_IN1 20
#define ENCODER_IN2 21

volatile long encoder_ticks = 0;
volatile long previous_encoder_ticks = 0;
volatile long RPM = 0;

RotaryEncoder *encoder = nullptr; // Damm a good methord to make instace var
// Solves the OLED display error

void checkPosition()
{
  encoder->tick(); // just call tick() to check the state.
}

//*************************************************************************

//************************** PID Constants ********************************

double Setpoint, Input, Output;
double Kp = 2, Ki = 5, Kd = 1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

//*************************************************************************

//***************************** PICO timer ********************************

RPI_PICO_Timer ITimer(0); // Create a single timer instance

// If the timer callback return false it will stop calling it self again
bool TimerHandler_Calculate_RPM(struct repeating_timer *t)
{
  (void)t; // This line is used to suppress unused parameter warnings

  long int curent_ticks = encoder->getPosition();
  RPM = ((curent_ticks - previous_encoder_ticks) * 60 * 1000) / PPR / RPM_TIMER_INTERVAL_MS;
  previous_encoder_ticks = curent_ticks;

  Serial.print("RPM = ");
  Serial.println(RPM);
  Serial.print("count = ");
  Serial.println(curent_ticks);
  Serial.println("---");

  return true; // Countinously run
}
//*************************************************************************

void setup()
{
  Serial.begin(115200);

  encoder = new RotaryEncoder(ENCODER_IN1, ENCODER_IN2, RotaryEncoder::LatchMode::TWO03);
  attachInterrupt(digitalPinToInterrupt(ENCODER_IN1), checkPosition, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_IN2), checkPosition, CHANGE);

#if (_CPU_STATS_ > 0)

  Serial.print(F("\nStarting Timer Callback on "));
  Serial.println(BOARD_NAME);
  Serial.println(RPI_PICO_TIMER_INTERRUPT_VERSION);
  Serial.print(F("CPU Frequency = "));
  Serial.print(F_CPU / 1000000);
  Serial.println(F(" MHz"));

#endif

  // Attach the timer interrupt with a 50ms interval
  if (ITimer.attachInterruptInterval(RPM_TIMER_INTERVAL_MS * 1000, TimerHandler_Calculate_RPM))
  {
    // chat gpt print rpm here
    // Serial.println(millis());
  }
  else
    Serial.println(F("Can't set Timer Callback. Select another Timer, freq. or timer"));
}

void loop()
{
  encoder->tick(); // just call tick() to check the state.
}