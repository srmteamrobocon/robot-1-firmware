#ifndef _MD30C_
#include <Arduino.h> // Removing this is giving warning
#define _MD30C_

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
#endif
