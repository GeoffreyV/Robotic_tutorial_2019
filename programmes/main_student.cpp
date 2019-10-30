#include "mbed.h"

#include "console_output.h"
#include "test_us.h"
#include "test_compass.h"
#include "test_cny.h"
#include "test_motor.h"

#include "pin_connexions.h"


Serial      pc      (PA_2, PA_3, 115200);

int main() {
    e_test user_choice ;
    ft_print_test_menu(pc);
    user_choice = ft_get_user_choice(pc);


    /* Main loop */
    while(1)
    {
        /* Which component to test ? */
        switch (user_choice)
        {
        case ULTRASONIC :
            do
            {
                ft_print_all_values_US(ultra_sonic, pc);
            } while (!pc.readable());
            user_choice = ft_get_user_choice(pc);
            pc.printf("\n");
            break;

        case COMPASS :
            do
            {
                ft_print_compass_values(compass, pc);
            } while (!pc.readable());
            pc.printf("\n");
            user_choice = ft_get_user_choice(pc);
            break;

        case CNY70 :
            do
            {
                pc.printf("\r CNY_1");
                //ft_print_cny_analog_voltage(CNY1, pc);
                pc.printf("\t CNY_2");
                //ft_print_cny_analog_voltage(CNY2, pc);
                pc.printf("\t CNY_3");
                //ft_print_cny_analog_voltage(CNY3, pc);
            } while (!pc.readable());
            pc.printf("\n");
            user_choice = ft_get_user_choice(pc);
            break;

        case VBAT :
            do
            {
                ft_print_to_be_imp(pc);
            } while (!pc.readable());
            pc.printf("\n");
            user_choice = ft_get_user_choice(pc);
            break;

        case LEFT_MOTOR :
            ft_print_to_be_imp(pc) //TODO : Replace this line
            user_choice = ft_get_user_choice(pc);
            break;

        case RIGHT_MOTOR:
            ft_print_to_be_imp(pc) //TODO : Replace this line
            user_choice = ft_get_user_choice(pc);
            break;


        default :
            ft_print_test_menu(pc);
            user_choice = ft_get_user_choice(pc);
            break;
        }
    }

}
