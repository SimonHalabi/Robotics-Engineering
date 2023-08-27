/*
 * Copyright 1996-2020 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <webots/accelerometer.h>
#include <webots/camera.h>
#include <webots/distance_sensor.h>
#include <webots/light_sensor.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>
#include <webots/supervisor.h>


#define WHEEL_RADIUS 0.02
#define AXLE_LENGTH 0.052
#define RANGE (1024 / 2)


//move a specified distance


//Computes the current position of the robot
const double* compute_position() {
  //Uses supervisor mode to get the reference for the node attacthed to the robot
  WbNodeRef node = wb_supervisor_node_get_from_def("E-PUCK");
  //Uses supervisor mode to get x,y,z co-ordinates of robot
  const double *pos = wb_supervisor_node_get_position(node);
  //Outputting the current co-ordinates of the robot
  //printf("what does the pos say %0.2f %0.2f %0.2f\n",pos[0],pos[1],pos[2]);
  //Return the current position for use by other functions
  return pos;
 
}
//Computes the orientation of the robot. CONVOLUDED
 //I had to do it this way because the supervisor orientation doesnt give the appropriate orientation
 double compute_orientation() {
 //initialising device tags to move the robot later
 WbDeviceTag  left_motor, right_motor;
 left_motor = wb_robot_get_device("left wheel motor");
 right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0);
  wb_motor_set_velocity(right_motor, 0);
    
 //Uses supervisor mode to get the reference for the node attacthed to the robot
  WbNodeRef node = wb_supervisor_node_get_from_def("E-PUCK");
  //record current position of robot
 const double *pos1 = wb_supervisor_node_get_position(node);
    
   //make the robot move foward at a slow speed
   wb_motor_set_velocity(left_motor, 1);
  wb_motor_set_velocity(right_motor, 1);
 //move foward for 10ms so that it doesn't go very far
 wb_robot_step(50);
 //stop the robot again.
    wb_motor_set_velocity(left_motor, 0);
  wb_motor_set_velocity(right_motor, 0);
  //determine the new position of the robot
   const double *pos2 = wb_supervisor_node_get_position(node);
 
 //this is just vector math to determine the direction facing based on the direction vector we have moved.
   double change_x = pos2[0] - pos1[0];
   double change_z = pos2[2] - pos1[2];
  
   double orientation = atan(change_z/change_x);
   if (change_x <0){
   orientation = orientation + 3.14159;
   }
   if(change_x >0 && change_z <0){
  orientation = orientation + 3.14159*2;
   }
   //return our current orientation.
   return orientation;  
}


//Compute the direction that the robot needs to move towards new destination
double direction_to_move(const double new_position[3]){
//Get the current position of the robot
 const double* position = compute_position();
 //Determine the x component of the direction
 double new_x = new_position[0] - position[0];
 //Determinbe the y component of the direction
 double new_z = new_position[2] - position[2];
 //Determine the direction to travel in radians
 double  direction = atan(new_z/new_x);
  //Return the direction of travel  
  
   
   if (new_x <0){
  direction = direction + 3.14159;
   }
   if(new_x >0 && new_z <0){
  direction = direction + 3.14159*2;
   }
  return direction;
}


//rotation rate at motorspeed 1 is 0.00297263‬ rad/ms
void turn_to_direction(double direction){
WbDeviceTag  left_motor, right_motor;
 left_motor = wb_robot_get_device("left wheel motor");
 right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0);
  wb_motor_set_velocity(right_motor, 0);
 double magic_num = 0.00297263;

double current_direction = compute_orientation();
if(direction > current_direction){
double difference = direction-current_direction;
printf("difference = %g\n",difference);
int time_to_turn = difference/magic_num;
printf("time = %d\n",time_to_turn);
wb_motor_set_velocity(left_motor, 4);
  wb_motor_set_velocity(right_motor, -4);
  wb_robot_step(time_to_turn);
  wb_motor_set_velocity(left_motor, 0);
  wb_motor_set_velocity(right_motor, 0);
}
else if(direction<current_direction){
double difference = current_direction- direction;
printf("difference = %g\n",difference);
int time_to_turn = difference/magic_num;
printf("time = %d\n",time_to_turn);
wb_motor_set_velocity(left_motor, -4);
  wb_motor_set_velocity(right_motor, 4);
  wb_robot_step(time_to_turn);
  wb_motor_set_velocity(left_motor, 0);
  wb_motor_set_velocity(right_motor, 0);
}
}


double distance_to_move(const double new_position[3]){
//Get the current position of the robot
 const double* position = compute_position();
 //Determine the x component of the direction
 double new_x = new_position[0] - position[0];
 //Determinbe the y component of the direction
 double new_z = new_position[2] - position[2];
 //Determine the direction to travel in radians
 double  distance = sqrt(new_x*new_x+new_z*new_z);
  //Return the direction of travel  
  
printf("The distance is: %f\n",distance);
  return distance;
}

//magic number is 0.000100301‬ unit/ms;
void move_distance(double distance){
WbDeviceTag  left_motor, right_motor;
 left_motor = wb_robot_get_device("left wheel motor");
 right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0);
  wb_motor_set_velocity(right_motor, 0);
 double magic_num = 0.000100301;
 double time_to_move = distance/magic_num;
 printf("The time is: %f\n",time_to_move);
 wb_motor_set_velocity(left_motor, 5);
  wb_motor_set_velocity(right_motor, 5);
wb_robot_step(time_to_move);
wb_motor_set_velocity(left_motor, 0);
  wb_motor_set_velocity(right_motor, 0);
  }
  
  void move_to_location(double location[3]){
  double dir = direction_to_move(location);
  turn_to_direction(dir);
  double dis = distance_to_move(location);
  move_distance(dis);
  }
  
int main(int argc, char *argv[]) {
  /* define variables */
 
   WbDeviceTag  left_motor, right_motor,left_position_sensor, right_position_sensor;
  double speed[2];
  
  int time_step;
  int camera_time_step;
  /* initialize Webots */
  wb_robot_init();

  if (strcmp(wb_robot_get_model(), "GCtronic e-puck2") == 0) {
    printf("e-puck2 robot\n");
    time_step = 64;
    camera_time_step = 64;
  } else {  // original e-puck
    printf("e-puck robot\n");
    time_step = 256;
    camera_time_step = 1024;
  }


  /* get a handler to the motors and set target position to infinity (speed control). */
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0);
  wb_motor_set_velocity(right_motor, 0);
  left_position_sensor = wb_robot_get_device("left wheel sensor");
  right_position_sensor = wb_robot_get_device("right wheel sensor");
  wb_position_sensor_enable(left_position_sensor, time_step);
  wb_position_sensor_enable(right_position_sensor, time_step);
  
    
  /* main loop */
  
   wb_motor_set_velocity(left_motor, 0);
  wb_motor_set_velocity(right_motor, 0);
  
  while (wb_robot_step(time_step) != -1) {
    /* get sensors values */
    
    
    int x=0;
    
    for(x;x<1;x++){
   /* double dir4 = compute_orientation();

    printf("%g\n" , dir4);
    
    wb_motor_set_velocity(left_motor, -4);
  wb_motor_set_velocity(right_motor, 4);
  wb_robot_step(1000);
    wb_motor_set_velocity(left_motor, 0);
  wb_motor_set_velocity(right_motor, 0);
   double dir5 = compute_orientation();*/
turn_to_direction(0);
turn_to_direction(3.14159/2);
turn_to_direction(3.14159);
turn_to_direction(3.14159*6/4);
}
 
//determine the position that the bot needs to move in to get the given position
  

 
    
    
}
  wb_robot_cleanup();

  return 0;
}

