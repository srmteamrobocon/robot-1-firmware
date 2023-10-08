#include <Arduino.h>
#include <PID_v1.h>
#include "RPi_Pico_TimerInterrupt.h"
#include "Commands.h"
#include "Bts.h"
#include "Cytron_MD30C.h"

#define _TIMERINTERRUPT_LOGLEVEL_ 4
#define _CPU_STATS_ 0

#define _LOG_MOTOR_PWM_ 0
#define _LOG_ENCODER_TICK_RPM_ 0
#define _LOG_PID_STATS_ 0

// PPR of motor
#define PPR 1300
// How frequently RPM should be calculated
#define RPM_TIMER_INTERVAL_MS 20

#define ENCODER_2_IN1 26
#define ENCODER_2_IN2 22

#define ENCODER_3_IN1 21
#define ENCODER_3_IN2 20

#define EncoderPinA 3 // 28
#define EncoderPinB 2 // 27

double Setpoint, Input, Output;

double Kp = 0.01;
double Ki = 0.01;
double Kd = 0.01;

volatile int32_t counts = 0;
volatile int32_t previous_encoder_ticks = 0;
volatile float RPM = 0;

struct BusData
{
  int16_t motor_1;
  int16_t motor_2;
  int16_t motor_3;
  int8_t led_status;
};
struct BusData DriveData = {0, 0, 0, 0};

// BTS7890 motor_2(6, 7);
// BTS7890 motor_3(8, 9);
// BTS7890 motor_1(12, 14);

Motor_cytron_md30c motor(6, 7);
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
RPI_PICO_Timer ITimer(0);

bool TimerHandler_Calculate_RPM(struct repeating_timer *t)
{
  (void)t; // This line is used to suppress unused parameter warnings
  RPM = ((counts / 4 - previous_encoder_ticks) * 60 * 1000) / PPR / RPM_TIMER_INTERVAL_MS;
  previous_encoder_ticks = counts / 4;
  return true; // Run Countinously
}

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

      // Convert valid PWM char array to Intiger and store it to Struct
      DriveData.motor_1 = atoi(buffer_motor_1_PWM);
      DriveData.motor_2 = atoi(buffer_motor_2_PWM);
      DriveData.motor_3 = atoi(buffer_motor_3_PWM);

      break;
    }

  case READ_ENCODERS:
  {
    Serial.print(counts);
    Serial.print(" ");
    Serial.print(counts);
    Serial.print(" ");
    Serial.println(counts);
    break;
  }
  }
}

void readEncoder()
{
  static int8_t prevA = 0;
  static int8_t prevB = 0;

  int8_t currA = digitalRead(EncoderPinA);
  int8_t currB = digitalRead(EncoderPinB);

  // Determine the direction of rotation
  int8_t direction = (prevA ^ currB) - (prevB ^ currA);

  if (direction == 1 || direction == -1)
  {
    counts += direction;
  }

  prevA = currA;
  prevB = currB;
}

void setup()
{
  Serial.setTimeout(1000);
  Serial.begin(115200);

  pinMode(EncoderPinA, INPUT_PULLUP); // Use internal pull-up resistors
  pinMode(EncoderPinB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(EncoderPinA), readEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(EncoderPinB), readEncoder, CHANGE);
}

// CORE 1 Main Loop
void loop()
{
  if (Serial.available() > 0)
  {
    byte cmd = Serial.read();
    run_commands(cmd);
  }

  motor.setPWM(DriveData.motor_1);

  // motor_1.setPWM(DriveData.motor_1 / 4);
  // motor_2.setPWM(DriveData.motor_2 / 4);
  // motor_3.setPWM(DriveData.motor_3 / 4);
  Serial.println("Hello robocon");
  delay(100);
#if (_LOG_MOTOR_PWM_)
  Serial.println(DriveData.motor_1);
  Serial.println(DriveData.motor_2);
  Serial.println(DriveData.motor_3);
#endif
}

// RP2040 Second core
void setup1()
{
  // turn the PID on

  // Attach the timer interrupt with a 50ms interval
  if (ITimer.attachInterruptInterval(RPM_TIMER_INTERVAL_MS * 1000, TimerHandler_Calculate_RPM))
    ;
  // Serial.println(millis());
  else
    Serial.println(F("Can't set Timer Callback. Select another Timer, freq. or timer"));

  // Input = counts;
  // Setpoint = 25000;
  // myPID.SetMode(AUTOMATIC);
}

void loop1()
{
  // Serial.println(counts / 4);
  // Serial.println(RPM);

  // Input = counts;
  // myPID.Compute();
  // motor.setPWM(-Output);

#if (_LOG_PID_STATS_)
  uint32_t time = millis();
  Serial.print(time / 1000);
  Serial.print(" ");
  Serial.print(Input);
  Serial.print("Output = ");
  Serial.print(Output);
  Serial.print(" ");
  Serial.println(Setpoint);
#endif
}