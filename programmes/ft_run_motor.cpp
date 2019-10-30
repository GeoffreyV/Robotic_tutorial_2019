#include "mbed.h"
#include "test_motor.h"

/**********************************************************************/
/**                         ft_run_motor                             **/
/* Args :   - direction (e_direction): direction to turn the motor    */
/*          - duty_cycle (double): duty cycle for the PWM             */
/*          - pwm_mot (PwmOut): pin to set the PWM                    */
/*          - dirA and dirB (DigitalOut): Pins to set the direction   */
/**********************************************************************/
void ft_run_motor(  e_direction direction, double duty_cycle,
                    PwmOut pwm_mot, DigitalOut  dirA, DigitalOut  dirB)
{
    if (direction == FORWARD)
    {
        // To be completed
    }
    else /* (direction == BACKWARD) */
    {
        // To be completed
    }

    /* TO DO : apply duty cycle to pwm_mot */
    // To be completed
}
