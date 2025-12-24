/* e-puck2 lab Task 2 --> POTATO THE EXPLORER! (BEWARE OF THE KETCHUP POLICE).
 *
 * Description:
 * Control the epuck robot (potato) to follow an object (ketchup).
 * - It measures the distance to ketchup continuously, if too far potato approaches, if too close potato retreats,
 *		if far enough potato stays there.
 * - When ketchup is lost or not in sight range, potato search for it turning left and right alternating direction with time.
 * - LEDs indicate if searching or chasing ketchup: front LED on = potato chasing, body LED on = potato searching.
 * https://www.chibios.org/dokuwiki/doku.php?id=chibios:documentation:books:rt:kernel_time
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <main.h>


#include "leds.h"
#include "spi_comm.h"
#include "motors.h"
#include "sensors/proximity.h"
#include "sensors/VL53L0X/VL53L0X.h"


messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

// Parameters
//#Define Epuck2 as potato;
//#Define Object as ketchup;
static const int close_distance = 70; // 7 cm - run away ketchup's too close
static const int far_distance = 100; // 10 cm - chase ketchup
static const int max_distance = 200; // 20 cm - max distance to detect ketchup (object)
static const int prox_distance = 200; // proximity detection --> task 1 = 450 & 400
static const int speed = 700;
#define TIME_MS2I



int main(void)
{
    // Initialization
    halInit();
    chSysInit();
    mpu_init();

    // Communication
    messagebus_init(&bus, &bus_lock, &bus_condvar);

    // Start proximity sensors
    proximity_start(0);
    calibrate_ir();

    // Start Time-of-Flight sensor
    VL53L0X_start();

    // LEDs and SPI
    clear_leds();
    spi_comm_start();

    // Initialize motors
    motors_init();

	// state parameters
    static bool searching = false;
    static int move_dir = 0; // backwards/left = -1, forward/right = +1
	static systime_t turn_time = 0; // turn start time
	static int turn_wait = 0; // in ms
	static bool left = true;


    // Main Control Loop
    while (1)
    {
        // Sensor reading
        uint16_t front_distance = VL53L0X_get_dist_mm();
        int front_left = get_calibrated_prox(7);
        int front_right = get_calibrated_prox(0);


	// Decision logic
	if (!searching) {
		set_front_led(1);
		if (front_distance > far_distance && front_distance <= max_distance) { // ketchup too far from potato within the detection range
			move_dir = 1; // move forward
		}
		else if (front_distance < close_distance) { // ketchup too close to potato
			move_dir = -1; // move backwards
		}
		else if (front_distance >= close_distance && front_distance <= far_distance) { // ketchup far enough from potato
			move_dir = 0; // no movement
		}
		else { // ketchup not found
			set_front_led(0);
			searching = true; // start searching for ketchup
			left = true;
			move_dir = 0;
		}
		left_motor_set_speed(move_dir * speed);
		right_motor_set_speed(move_dir * speed);
	}
	else {
		if (front_distance <= max_distance) {
			searching = false; // stop searching, ketchup's been found
			set_body_led(0);
			move_dir = 0;
		}
		else { // ketchup is still missing
			set_body_led(1);
			systime_t now = chVTGetSystemTimeX(); // get time now for start of turn
			if (move_dir == 0) { // if potato not turning
				if (left) {
					move_dir = -1; // turn left
					left = false;
					turn_wait = 1500; // left turn aprox 180d at fix speed
				}
				else {
					move_dir = 1; // turn right
					left = true;
					turn_wait = 3*turn_wait; // right turn aprox 360d (including return from previous turn)
				}
				turn_time = now; // turn time start
			}
			else if (chVTTimeElapsedSinceX(turn_time) >= TIME_MS2I(turn_wait)) { // if turn_wait has been completed
				move_dir = 0; // reset turn
			}
		}
		left_motor_set_speed(move_dir * speed);
		right_motor_set_speed(-move_dir * speed);
	}
        chThdSleepMilliseconds(30); // small water break for stability
    }
}

// Stack Protection
#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}

