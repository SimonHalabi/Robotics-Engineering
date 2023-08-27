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
 //chANGE WHILE LOOP TO HAVE PSO ALGORITHM RUN UNTIL THE IR SENSOR THRESHOLD IS MET THEN OBSTICLE AVOID
 
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/light_sensor.h>
#include <stdio.h>
#include <string.h>
#include <webots/Supervisor.h>
#include <webots/emitter.h>
#include <webots/receiver.h>

#define LRANGE 4200
#define AXLE_LENGTH = 0.052
#define WHEEL_RADIUS = 0.02
#define TIME_STEP 64
#define MAX_SPEED 6.28
#define NUM_IR_SENSORS 4
#define NUM_OPTICAL_SENSORS 4
#define NUM_ROBOTS 5
#define COMMUNICATION_CHANNEL 1
#define WB_CHANNEL_BROADCAST -1
#define RANGE (1024 / 2)

#include <stdlib.h>
#include <stdbool.h>
#include <math.h>

#include <stdio.h>
#include <string.h>

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



double** pso_algorithm(double ls[], double g, double g_pos[], double x[], double x_hat[], double p, double v[])
{
    //ls --> array of light sensors on robot6
    //g --> global best (brightness) 
    //g_pos --> position of g [x_pos,y_pos]
    //x --> 2 dimension array [x_pos,y_pos] of our current position
    //x_hat --> 2 dimension array [x_pos,y_pos] of current own best position
    //p --> personal best (brightness)
    //v --> current velocity array [x_vel,y_vel]

    //define variables
    const double w = 1.0;
    const double c1 = 2.0;
    const double c2 = 2.0;
    const int swarm_size = 5;
    const double tolerance = 0.1;
    double r1 = rand() % 100;
    r1 /= 100; //random double between 0-0.99
    double r2 = rand() % 100;
    r2 /= 100; //random double between 0-0.99
    bool at_brightest = false; //are we at the brightest spot in the arena?

    //compute current objective function
    //j=max of all light sensors
    double j = ls[0];
    for (int i = 1; i < 8; i++)
    {
        if (ls[i] > j)
            j = ls[i];
    }

    //update individual and global bests
    if (j > p) //our current objective function is better than our personal best
    {
        p = j;
        x_hat = x;
    }
    if (j > g) //our current objective function is better than global best
    {
        g = j;
        g_pos = x;
    }
    if (j-g<tolerance)
    {
        at_brightest = true;
    }

    //update velocity
    double v_next[2];
    for (int i = 0; i < 2; i++)
    {
        v_next[i] = w * v[i] + c1 * r1 * (x_hat[i] - x[i]) + c2 * r2 * (g - x[i]);
    }

    //update position
    double x_next[2];
    for (int i = 0; i < 2; i++)
    {
        x_next[i] = x[i] + v_next[i];
    }

    //create array for function to return using dynamic memory - array has 2 rows and 7 columns
    int row =2;
    int col = 7;
    
    double **return_array = (double **)malloc(row * sizeof(double *)); 
    for (int i=0; i<row; i++) 
         return_array[i] = (double *)malloc(col * sizeof(double)); 
    
    //double** return_array = new double* [2];
    //for (int i = 0; i < 2; i++)
    //    return_array[i] = new double [7];

    //populate array
    return_array[0][0] = g;
    return_array[0][4] = p;
    return_array[0][7] = at_brightest;
    for (int i = 0; i < 2; i++)
    {
        return_array[i][1] = g_pos[i];
        return_array[i][2] = x_next[i];
        return_array[i][3] = x_hat[i];
        return_array[i][5] = v_next[i];
    }
    return return_array;
    
    move_to_location(g);
}

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
    
    //PSO ALGORITHM
    double **pso_array;
    double zero_vector [2] = {0.0,0.0};
        double* g_pos = compute_position();
    pso_array = pso_algorithm(light_sensor_value, g, g_pos, compute_position(), zero_vector, 0, zero_vector);
    move_to_location(g_pos);

    
           
            WbDeviceTag tx1, rx1;
    
    tx1 = wb_robot_get_device("emitter");
  wb_emitter_set_channel(tx1, WB_CHANNEL_BROADCAST);
  
  // setup receiver
   
  rx1 = wb_robot_get_device("receiver");
  wb_receiver_enable(rx1, time_step);
  wb_receiver_set_channel(rx1, COMMUNICATION_CHANNEL);
  
      char message[128];
  
   wb_emitter_send(tx1, message, strlen(message) + 1);
   

     
     /* is there at least one packet in the receiver's queue ? */
      if (wb_receiver_get_queue_length(rx1) > 0) {
        
        /* read current packet's data */
        const char *buffer = wb_receiver_get_data(rx1);

        /* print null-terminated message */
       printf("%s received a message:\"%s\"\n",buffer);
        
        /* fetch next packet */
        wb_receiver_next_packet(rx1);
      }
      

     //sprintf(message,"%s ls0=%d ls7=%d",bot_name,ls0,ls7);
     //sprintf(message,g_pos,g);

    
    
    
    wb_robot_cleanup();

    return 0;
}
