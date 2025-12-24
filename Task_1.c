/* e-puck2 lab Task 1 --> POTATO THE EXPLORER!
 *
 * Description:
 * Control the e-puck robot (potato) to explore inside the arena (garden) without crashing into walls.
 * - It explores the garden freely always moving forward until it encounters a wall in front.
 * - To avoid crashing into walls potato stops and identifies which side is the most clearer from obstacles,
 *		then it turns towards that side.
 * - When the obstacle is at the same distance from both sides even with a side margin added to prevent this,
 *		then potato will prioritize turning right.
 * - LEDs indicate if exploring or avoiding in the garden: front LED on = avoiding garden walls, body LED on = potato exploring the garden.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <main.h>
#include "leds.h"
#include "spi_comm.h"
#include "sensors/proximity.h"
#include "motors.h"

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

// Parameters
//#Define Epuck2 as potato;
//#Define Arena as garden;
static const int speed   = 600;
static const int obstructed   = 450;
static const int clear   = 400;

int main(void)
{

    halInit();
    chSysInit();
    mpu_init();

    // Proximity
    messagebus_init(&bus, &bus_lock, &bus_condvar);
    proximity_start(0);
    calibrate_ir();

    //LEDs
    clear_leds();
    spi_comm_start();

    //Motors
    motors_init();

	// state parameters
	static bool avoiding = false;
	static int turn_dir = 0; // left = -1, right = +1

	while (1)
	{
		// Right proxs
		int p0 = get_calibrated_prox(0); // front right
		int p1 = get_calibrated_prox(1); // mid front right
		int p2 = get_calibrated_prox(2); // right
		int p3 = get_calibrated_prox(3); // back right --> not used in this code logic

		// Left proxs
		int p7 = get_calibrated_prox(7); // front left
		int p6 = get_calibrated_prox(6); // mid front left
		int p5 = get_calibrated_prox(5); // left
		int p4 = get_calibrated_prox(4); // back left --> not used in this code logic

		// Calculating each side proximity values.
		int right_side = (p0 + p1 + p2) / 3;
		int left_side  = (p7 + p6 + p5) / 3;
		int front = (p0 + p7) / 2;

		const int side_margin = 40; // side threshold to avoid ties in left/right readings

		// Decision logic
		if (!avoiding) { // if there's no obstacle be vigilant
		    if (front >= obstructed) { // if there's a wall in front
		        avoiding = true; // changing to avoiding sate
		        if (left_side + side_margin < right_side) { // left side has more open space that right side
		        	set_led(LED7, 1);
		            turn_dir = -1; // turn left
		        }
		    	else if (right_side + side_margin < left_side) { // right side has more open space that left side
		    		set_led(LED3, 1);
		            turn_dir = +1; // turn right
		        }
		    	else { // right/left tie
		    		set_led(LED2, 1); set_led(LED3, 1); set_led(LED6, 1); set_led(LED7, 1);
		            turn_dir = +1; // turn right
		        }
		    }
		    else if (right_side >= obstructed || left_side >= obstructed) { // if there's a garden wall on either side
		        avoiding = true; // changing to avoiding state
		        set_led(LED2, 1); set_led(LED3, 1); set_led(LED6, 1); set_led(LED7, 1);
		        turn_dir = (right_side > left_side) ? -1 : +1; // potato turns toward the side with more open sapce
		    }
		}
		else { // if potato avoiding garden wall
		    if (front < clear && right_side < clear && left_side < clear) { // check if both sides and front are clear
		        avoiding = false; // stop avoiding
		        turn_dir = 0;
		    }
		}


		// Movement logic
		if (avoiding) { // when potato is avoiding a wall, turn
			set_body_led(0);
			set_front_led(1);
			left_motor_set_speed(turn_dir * speed);
			right_motor_set_speed(-turn_dir * speed);
		}
		else { // when potato exploring garden, move forward
			set_front_led(0);
			set_body_led(1);
			set_led(LED2, 0); set_led(LED3, 0); set_led(LED6, 0); set_led(LED7, 0);
			left_motor_set_speed(speed);
			right_motor_set_speed(speed);
		}
		chThdSleepMilliseconds(30); // small water break for stability
	}

}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
