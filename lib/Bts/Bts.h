#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

class Motor
{
private:
    int _PWM_1;
    int _PWM_2;

public:
    Motor(int PWM_1, int PWM_2);
    void setPWM(int pwm_value);
    void stopMotor();
    void motorBreak();
};

Motor::Motor(int PWM_1, int PWM_2)
{
    /** Driver code for BTS790...  */

    _PWM_1 = PWM_1;
    _PWM_2 = PWM_2;

    pinMode(_PWM_1, OUTPUT);
    pinMode(_PWM_2, OUTPUT);
    analogWrite(_PWM_1, 0);
    analogWrite(_PWM_2, 0);
}
void Motor::setPWM(int pwm_value)
{
    if (pwm_value > 0)
    {
        analogWrite(_PWM_1, 0); // Stop the other direction
        analogWrite(_PWM_2, pwm_value);
    }
    else if (pwm_value < 0)
    {
        analogWrite(_PWM_2, 0);
        analogWrite(_PWM_1, -(pwm_value)); // Remove the negetive sign
    }
    else
    {
        // If pwm is 0 turn on all low
        analogWrite(_PWM_1, 0);
        analogWrite(_PWM_2, 0);
    }
}
#endif