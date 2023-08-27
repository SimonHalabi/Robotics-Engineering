/*
 * File:          test.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/motor.h>, etc.
 */
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/light_sensor.h>
#include <stdio.h>
#include <string.h>
#include <webots/Supervisor.h>

#define LRANGE 4200
#define AXLE_LENGTH = 0.052
#define WHEEL_RADIUS = 0.02
#define TIME_STEP 64
#define MAX_SPEED 6.28
#define NUM_IR_SENSORS 4
#define NUM_OPTICAL_SENSORS 4
#define NUM_ROBOTS 5

#include <stdlib.h>
#include <stdbool.h>
#include <math.h>

/* distance sensor */
#define NUMBER_OF_DISTANCE_SENSORS 8
static WbDeviceTag distance_sensors[NUMBER_OF_DISTANCE_SENSORS];
static const char *distance_sensors_names[NUMBER_OF_DISTANCE_SENSORS] = {"ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7"};

/*
 * The sensor has noise
 * So even there is no obstacle, sensor values is not zero
 * So to detect obstacle, we must use this threshold
 * Based on my experiment, the good threshold value is 140
 * Obstacle detected condition is true if the sensor values is larger then this threshold value
 * */
#define SENSOR_VALUE_DETECTION_THRESHOLD 140

/* speed of robot to spinning in place (in degrees per second) */
#define ROBOT_ANGULAR_SPEED_IN_DEGREES 283.588111888

// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char *argv[]) {

  WbDeviceTag light_sensor[8], left_motor, right_motor;
  int i,j,i0,i1,d0,d1,d2;
  double g;
  double speed[2];
  double light_sensor_value[8];
  double Distance_sensor_value[8];
  double braitenberg_light[8][2] = {{0., 2.5}, {0., 0}, {0., 0}, {0., 0}, {0., 0}, {0., 0}, {0., 0}, {2.5, 0}};
  int time_step = 64;
 void robot_controller_init(int time_step)
{
	//TIME_STEP = time_step;
  /* get a handler to the motors and set target position to infinity (speed control) */
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);
  
  /* get a handler to the sensors */
	for (int i = 0; i < NUMBER_OF_DISTANCE_SENSORS; i++) {
			distance_sensors[i] = wb_robot_get_device(distance_sensors_names[i]);
			wb_distance_sensor_enable(distance_sensors[i], TIME_STEP);
	}
}

  float calculate_rotation_time(float degrees)
  {
	return abs(degrees) / ROBOT_ANGULAR_SPEED_IN_DEGREES;
  } 
 
   void motor_stop() {
  wb_motor_set_velocity(left_motor, 0);
  wb_motor_set_velocity(right_motor, 0);
}

/* function to set motor velocity to move forward */
void motor_move_forward() {
for(i=0;i<2;i++) {
      speed[i] = 0.0;
        for (j=0;j<8;j++)
       speed[i] += braitenberg_light[j][i] * (light_sensor_value[j] / LRANGE);
    wb_motor_set_velocity(left_motor, speed[0]);
    wb_motor_set_velocity(right_motor,speed[1]);
}
}

/* function to set motor velocity to rotate right in place*/
void motor_rotate_right() {
  wb_motor_set_velocity(left_motor, MAX_SPEED);
  wb_motor_set_velocity(right_motor, -MAX_SPEED);
}

/* function to set motor velocity to rotate left in place*/
void motor_rotate_left() {
  wb_motor_set_velocity(left_motor, -MAX_SPEED);
  wb_motor_set_velocity(right_motor, MAX_SPEED);
}

void motor_rotate_left_in_degrees(float degrees) {
	motor_rotate_left();
	
	float duration = calculate_rotation_time(degrees);
	float start_time = wb_robot_get_time();
	do
	{
		wb_robot_step(TIME_STEP);
	} while (wb_robot_get_time() < start_time + duration);
	
	motor_stop();
}

bool * get_sensors_condition()
{
	static bool sensors_condition[NUMBER_OF_DISTANCE_SENSORS] = {false};
	
	for (int i = 0; i < NUMBER_OF_DISTANCE_SENSORS ; i++) {
		/*
		 * Obstacle detected condition is true if the sensor values is larger then the threshold value
		 * */
		if (wb_distance_sensor_get_value(distance_sensors[i]) > SENSOR_VALUE_DETECTION_THRESHOLD) {
			sensors_condition[i] = true;
		} else {
			sensors_condition[i] = false;
		}
	}
	
	return sensors_condition;
}

/* function to print sensors values
 * */
void print_sensor_values() {
	printf("%s sensor values: ", wb_robot_get_name());
	
	for (int i = 0; i < NUMBER_OF_DISTANCE_SENSORS ; i++) {
		printf("%d:%.3f ", i, wb_distance_sensor_get_value(distance_sensors[i]));
	}
	
	printf("\n");
} 
 
 
  g = 1.0;
  wb_robot_init();
	
	/* init the controller */
	robot_controller_init(time_step);	
	
  if (strcmp(wb_robot_get_model(), "GCtronic e-puck2") == 0) {
    printf("e-puck2 robot\n");
    time_step = 64;
    }else{
    printf("e-puck robot\n");
    time_step = 256;
    }
  
  for (i=0; i<8; i++) {
    char device_name[4];
    sprintf(device_name, "ls%d", i);
    light_sensor[i] = wb_robot_get_device(device_name);
    wb_light_sensor_enable(light_sensor[i], time_step);
   } 

    
  while (wb_robot_step(time_step) != -1) {
   
    for (i=0; i < 8; i++)
    
    for (i=0; i < 8; i++){
    light_sensor_value[i] = wb_light_sensor_get_value(light_sensor[i]);
    Distance_sensor_value[i] = wb_distance_sensor_get_value(distance_sensors[i]);
    }
    
   bool *is_sensors_active = get_sensors_condition();
		print_sensor_values();
		if (is_sensors_active[1] && is_sensors_active[6]) {
			motor_rotate_left_in_degrees(180);
		} else if (is_sensors_active[0] || is_sensors_active[1]) {
			motor_rotate_left();
		} else if (is_sensors_active[7] || is_sensors_active[6]) {
			motor_rotate_right();
		} else {
			motor_move_forward();
       
       if (((light_sensor_value[i1]/LRANGE) < light_sensor_value[i0]/LRANGE) && (light_sensor_value[i1]/LRANGE) < g){
       g = (light_sensor_value[i1]/LRANGE);
    
       printf("%0.4f\n", g);
       }else if((light_sensor_value[i0]/LRANGE) < (light_sensor_value[i1]/LRANGE) && (light_sensor_value[i0]/LRANGE) < g){
       g = (light_sensor_value[i0]/LRANGE);
       printf("%0.4f\n", g);
     
       
    }
    
			
		}
   
    for (i=0; i < 8; i++)
    
    for (i=0; i < 8; i++){
    light_sensor_value[i] = wb_light_sensor_get_value(light_sensor[i]);
    Distance_sensor_value[i] = wb_distance_sensor_get_value(distance_sensors[i]);
    }
    i0 = 1;
    d0 = 1;
    d2 = 2;
    i1 = 7;
    d1 = 7;
    printf("ls%d = %0.4f\t ls%d = %0.4f\n",i0, light_sensor_value[i0]/LRANGE, i1, light_sensor_value[i1]/LRANGE);
    printf("ps%d = %0.4f\t ps%d = %0.4f\t ps%d = %0.4f\n",d0, Distance_sensor_value[d0],d2, Distance_sensor_value[d2], d1, Distance_sensor_value[d1]);
    //printf("%f\n",speed[0]);
    //printf("%f\n",speed[1]);
    //printf("%f\n",speed[2]);
      
      
    //wb_motor_set_velocity(left_motor, speed[0]);
    //wb_motor_set_velocity(right_motor,speed[1]);
    //wb_motor_set_velocity(left_motor, left_speed);
    //wb_motor_set_velocity(right_motor,right_speed);
    }
    wb_robot_cleanup();

    return 0;
}
