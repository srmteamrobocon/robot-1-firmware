#include <Arduino.h>
#include <PID_v1.h>
#include <RotaryEncoder.h>
#include "RPi_Pico_TimerInterrupt.h"

#define TIMER_INTERRUPT_DEBUG 1
#define _TIMERINTERRUPT_LOGLEVEL_ 4
#define _CPU_STATS_ 0
#define TIMER_INTERVAL_MS 50 // How frequently RPM should be calculated

#define PIN_IN1 20
#define PIN_IN2 21

double Setpoint, Input, Output;

// Specify the links and initial tuning parameters

double Kp = 2, Ki = 5, Kd = 1;

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
RotaryEncoder *encoder = nullptr;

void checkPosition()
{
  encoder->tick();
}

bool TimerHandler_Calculate_RPM(struct repeating_timer *t) // If the timer callback return false it will stop calling it self again
{
  (void)t; // This line is used to suppress unused parameter warnings

#if (TIMER_INTERRUPT_DEBUG > 0)
  Serial.print("Timer: millis() = ");
  Serial.println(millis()); // Just to check how accurate timer is
#endif

  // Print "Hello, world!" to the serial monitor
  Serial.println("Hello, world!");

  return true; // Countinously run
}

RPI_PICO_Timer ITimer(0); // Create a single timer instance

void setup()
{
  Serial.begin(115200);

  Serial.println("InterruptRotator example for the RotaryEncoder library.");
  encoder = new RotaryEncoder(PIN_IN1, PIN_IN2, RotaryEncoder::LatchMode::TWO03);

  // register interrupt routine
  attachInterrupt(digitalPinToInterrupt(PIN_IN1), checkPosition, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_IN2), checkPosition, CHANGE);

#if (_CPU_STATS_ > 0)

  Serial.print(F("\nStarting Timer Callback on "));
  Serial.println(BOARD_NAME);
  Serial.println(RPI_PICO_TIMER_INTERRUPT_VERSION);
  Serial.print(F("CPU Frequency = "));
  Serial.print(F_CPU / 1000000);
  Serial.println(F(" MHz"));

#endif

  // Attach the timer interrupt with a 50ms interval
  if (ITimer.attachInterruptInterval(TIMER_INTERVAL_MS * 1000, TimerHandler_Calculate_RPM))
  {
    // chat gpt print rpm here
    // Serial.println(millis());
  }
  else
    Serial.println(F("Can't set Timer Callback. Select another Timer, freq. or timer"));
}

void loop()
{
  static int pos = 0;

  encoder->tick(); // just call tick() to check the state.

  int newPos = encoder->getPosition();
  if (pos != newPos)
  {
    Serial.print("pos:");
    Serial.print(newPos * 2);
    Serial.print(" dir:");
    Serial.println((int)(encoder->getDirection()));
    pos = newPos;
  }
}

/*



void setup()
{

  // Setpoint = 100;
}

void loop()
{
  } // if
} // loop ()

// To use other pins with Arduino UNO you can also use the ISR directly.
// Here is some code for A2 and A3 using ATMega168 ff. specific registers.

// Setup flags to activate the ISR PCINT1.
// You may have to modify the next 2 lines if using other pins than A2 and A3
//   PCICR |= (1 << PCIE1);    // This enables Pin Change Interrupt 1 that covers the Analog input pins or Port C.
//   PCMSK1 |= (1 << PCINT10) | (1 << PCINT11);  // This enables the interrupt for pin 2 and 3 of Port C.

// The Interrupt Service Routine for Pin Change Interrupt 1
// This routine will only be called on any signal change on A2 and A3.
// ISR(PCINT1_vect) {
//   encoder->tick(); // just call tick() to check the state.
// }

// The End
*/