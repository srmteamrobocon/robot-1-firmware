#ifndef _BTS_
#define _BTS_
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
        pwm_value = pwm_value % 150;
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
    // implement max pwm fun % and stop all motors
    void set_max_pwm(int max_pwm);
};
#endif