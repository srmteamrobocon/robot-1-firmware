#include <Arduino.h>
#include <PID_v1.h>
#include "RPi_Pico_TimerInterrupt.h"

#define TIMER_INTERRUPT_DEBUG 1
#define _TIMERINTERRUPT_LOGLEVEL_ 4
#define _CPU_STATS_ 0
#define PPR 1300

#define RPM_TIMER_INTERVAL_MS 50 // How frequently RPM should be calculated

#define ENCODER_IN1 20
#define ENCODER_IN2 21

volatile long encoder_ticks = 0;          // Variable to store the encoder position
volatile long previous_encoder_ticks = 0; // Variable to store the encoder position
volatile long RPM = 0;                    // Variable to store the encoder position
int last_A_state = LOW;                   // Assuming the initial state is LOW

// Specify the links and initial tuning parameters
double Setpoint, Input, Output;
double Kp = 2, Ki = 5, Kd = 1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void handleEncoder()
{
  // Read the current state of the encoder pins
  int stateA = digitalRead(ENCODER_IN1);
  int stateB = digitalRead(ENCODER_IN2);
  // Determine the direction of rotation
  if (stateA != last_A_state)
  {

    if (stateB != stateA)
    {
      encoder_ticks++; // Clockwise rotation
    }
    else
    {
      encoder_ticks--; // Counterclockwise rotation
    }

    // Print the current encoder position
    last_A_state = stateA;
    // Serial.println(encoder_ticks);
  }
}

RPI_PICO_Timer ITimer(0);                                  // Create a single timer instance
bool TimerHandler_Calculate_RPM(struct repeating_timer *t) // If the timer callback return false it will stop calling it self again
{
  (void)t; // This line is used to suppress unused parameter warnings

  RPM = ((encoder_ticks - previous_encoder_ticks) / PPR) * 60 * 1000 / RPM_TIMER_INTERVAL_MS;
  previous_encoder_ticks = encoder_ticks;
  Serial.print("RPM = ");
  Serial.println(RPM);
  Serial.print("count = ");
  Serial.println(encoder_ticks);
  Serial.println("---");

  return true; // Countinously run
}

void setup()
{
  Serial.begin(115200);
  pinMode(ENCODER_IN1, INPUT_PULLUP); // Set encoderPinA as input with pull-up resistor
  pinMode(ENCODER_IN2, INPUT_PULLUP); // Set encoderPinB as input with pull-up resistor

  attachInterrupt(digitalPinToInterrupt(ENCODER_IN1), handleEncoder, CHANGE); // Attach interrupt for encoderPinA
  attachInterrupt(digitalPinToInterrupt(ENCODER_IN2), handleEncoder, CHANGE); // Attach interrupt for encoderPinB

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
}