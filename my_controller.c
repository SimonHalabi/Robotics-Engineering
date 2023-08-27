/*
 * E-puck Braitenberg vehicle example
 *
 */

#include <stdio.h>
#include <string.h>

#include <webots/light_sensor.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/emitter.h>
#include <webots/receiver.h>

#define WHEEL_RADIUS 0.02
#define AXLE_LENGTH 0.052
#define LRANGE 4200
#define COMMUNICATION_CHANNEL 1
#define WB_CHANNEL_BROADCAST -1

int main(int argc, char *argv[]) {
  /* define variables */
  WbDeviceTag light_sensor[8], left_motor, right_motor, tx1, rx1, robot_name[4];
  int i, j, i0, i1;
  double speed[2];
  double light_sensor_value[8];
  double braitenberg_light[8][2] = {{0., 2.5}, {0., 0}, {0., 0.},  {0., 0.},
                                           {0., 0.}, {0., 0.}, {0., 0.}, {2.5, 0}};
  int time_step;
  /* initialize Webots */
  wb_robot_init();
  
  // get the bot name/ID
  char bot_name[12];
  sprintf(bot_name,"%s",wb_robot_get_name());
  
  
  if (strcmp(wb_robot_get_model(), "GCtronic e-puck2") == 0) {
    printf("e-puck2 robot\n");
    time_step = 64;
  } else {  // original e-puck
    printf("e-puck robot\n");
    time_step = 256;
  }

  
  /* get a handler to the motors and set target position to infinity (speed control). */
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

 
  for (i = 0; i < 8; i++) {
    char device_name[4];
    
    /* get light sensors */
    sprintf(device_name, "ls%d", i);
    light_sensor[i] = wb_robot_get_device(device_name);
    wb_light_sensor_enable(light_sensor[i], time_step);
  }
  
  /* setup and enable communication between the bots */
  
  // setup emitter
  //tx1 = wb_robot_get_device("emitter");
  //wb_emitter_set_channel(tx1, COMMUNICATION_CHANNEL);
  
  tx1 = wb_robot_get_device("emitter");
  wb_emitter_set_channel(tx1, WB_CHANNEL_BROADCAST);
  
  // setup receiver
  rx1 = wb_robot_get_device("receiver");
  wb_receiver_enable(rx1, time_step);
  wb_receiver_set_channel(rx1, COMMUNICATION_CHANNEL);
  

  /* main loop */
  while (wb_robot_step(time_step) != -1) {
    
    /* get sensors values */
    for (i = 0; i < 8; i++)      
      /* get light sensors values */
    for (i = 0; i < 8; i++){
      light_sensor_value[i] = wb_light_sensor_get_value(light_sensor[i]);
     //printf("ls%d values = %0.2f\n",i, light_sensor_value[i]/LRANGE);
     }
     
     i0 = 0;
     i1 = 7;
     //printf("ls%d = %0.4f\t ls%d = %0.4f\n",i0, light_sensor_value[i0]/LRANGE,i1, light_sensor_value[i1]/LRANGE);
     
     for (i = 0; i < 2; i++) {
      speed[i] = 0.0;
           for (j = 0; j < 8; j++)
       speed[i] += braitenberg_light[j][i] * (light_sensor_value[j] / LRANGE);
  
    }
    
    // sensors value conversion
    int ls0 = light_sensor_value[i0];
    int ls7 = light_sensor_value[i1];
    
    /* send null-terminated message */
     char message[128];
     //sprintf(message,"%s ls0=%d ls7=%d",bot_name,ls0,ls7);
     sprintf(message,"%s light sensors values are ls0 = %d, ls7 = %d",bot_name,ls0,ls7);
     
     wb_emitter_send(tx1, message, strlen(message) + 1);
     
     /* is there at least one packet in the receiver's queue ? */
      if (wb_receiver_get_queue_length(rx1) > 0) {
        
        /* read current packet's data */
        const char *buffer = wb_receiver_get_data(rx1);

        /* print null-terminated message */
        printf("%s received a message:\"%s\"\n", bot_name,buffer);
        
        /* fetch next packet */
        wb_receiver_next_packet(rx1);
      }
      

    /* set speed values */
    wb_motor_set_velocity(left_motor, speed[0]);
    wb_motor_set_velocity(right_motor, speed[1]);
  }

  wb_robot_cleanup();

  return 0;
}
