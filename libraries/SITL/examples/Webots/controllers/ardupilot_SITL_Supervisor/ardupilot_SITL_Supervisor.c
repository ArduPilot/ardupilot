/*
 * File:          ardupilot_SITL_Supervisor.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/motor.h>, etc.
 */

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/types.h>
#include <webots/robot.h>
#include <webots/supervisor.h>
#include <webots/emitter.h>

#define MAX_NUM_ROBOTS 4

//define DEBUG_DATA

typedef struct {
  WbNodeRef robot_node;       // to track the robot node
  int       receiver_channel; // receiver channel
  double    velocity[3];      // to track robot's position
  
} Robot;


static WbDeviceTag emitter;
static WbNodeRef world_info, self_node;
static Robot *robots[MAX_NUM_ROBOTS];
static int actualRobots = 0;
double *linear_velocity;


static double northDirection[3] = {1.0,0.0,0.0};

/*
 * You may want to add macros here.
 */
static int timestep;


void initialize (int argc, char *argv[])
{

  WbNodeRef root, node;
  WbFieldRef children, field;
  int n, i , note_type;

  /* necessary to initialize webots stuff */
  wb_robot_init();

  timestep = (int)wb_robot_get_basic_time_step();

  self_node = wb_supervisor_node_get_self();
  
  root = wb_supervisor_node_get_root();
  children = wb_supervisor_node_get_field(root, "children");
  n = wb_supervisor_field_get_count(children);
  printf("This world contains %d nodes:\n", n);
  for (i = 0, actualRobots=0; i < n; i++) {
    node = wb_supervisor_field_get_mf_node(children, i);
    field = wb_supervisor_node_get_field(node, "name");
    note_type = wb_supervisor_node_get_type(node);
    if ( note_type  == WB_NODE_WORLD_INFO)
    {
      world_info = node; 
      //break;
    }
    else if ((note_type == WB_NODE_ROBOT) && (node != self_node))
    {
      /* code */
       if (actualRobots < MAX_NUM_ROBOTS)
        {
          WbFieldRef channel = wb_supervisor_node_get_field(node, "customData");
          Robot *robot = malloc(sizeof(Robot));
          robots[actualRobots] = robot;
          robots[actualRobots]->robot_node = node;
          robots[actualRobots]->receiver_channel = atoi((const char *)(wb_supervisor_field_get_sf_string(channel)));
         
          printf("Robot %s with ch %d is added \n",wb_supervisor_field_get_sf_string(field),robots[actualRobots]->receiver_channel);

          actualRobots++;
        }
        else
        {
          printf("Robot %s is ignored \n",wb_supervisor_field_get_sf_string(field));
        }
    }
    
    
  }
  node = wb_supervisor_field_get_mf_node(children, 0);
  field = wb_supervisor_node_get_field(node, "northDirection");
  memcpy(&northDirection,wb_supervisor_field_get_sf_vec3f(field),sizeof(double)*3);
    
  if (northDirection[0] == 1)
  {
    printf ("Axis Default Directions\n");
  }

  printf("WorldInfo.northDirection = %g %g %g\n\n", northDirection[0], northDirection[1], northDirection[2]);



  
  printf("SUPVERVISOR timestep %d\n", timestep);
  

  emitter = wb_robot_get_device("emitter");
  
  
}

/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv) {

  initialize( argc, argv);

  /*
   * You should declare here WbDeviceTag variables for storing
   * robot devices like this:
   *  WbDeviceTag my_sensor = wb_robot_get_device("my_sensor");
   *  WbDeviceTag my_actuator = wb_robot_get_device("my_actuator");
   */


  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */
  while (true) {

    wb_robot_step(timestep);
    //printf("timestep %f\n", wb_robot_get_time());
                    
    /*
     * Read the sensors :
     * Enter here functions to read sensor data, like:
     *  double val = wb_distance_sensor_get_value(my_sensor);
     */

    /* Process sensor data here */
    /*
     * Enter here functions to send actuator commands, like:
     * wb_motor_set_position(my_actuator, 10.0);
     */
    static double outputData[4];
    for (int i=0; i < actualRobots; ++i)
    {

      linear_velocity = (double *) wb_supervisor_node_get_velocity (robots[i]->robot_node);
      //sprintf (buf,"[%lf, %lf, %lf]", linear_velocity[0], linear_velocity[1], linear_velocity[2]);
      //WbFieldRef channel = wb_supervisor_node_get_field(robots[i]->robot_node, "customData");
      //wb_supervisor_field_set_sf_string (channel, &buf[0]);
      //printf("%s",buf);
      memcpy(outputData,linear_velocity, sizeof(double) * 3);

      /*
      * This is used to handle sensors axis when northDirection is different than [1,0,0]
      * Note: that only two values are handled here.
      * Local map northDirection [1,0,0]
      * OpenStreetView map northDirection [0,0,1]
      */
      outputData[3] = northDirection[0]; // send Map indicator;
      wb_emitter_set_channel(emitter, robots[i]->receiver_channel);
      wb_emitter_send(emitter, (const void *) outputData, sizeof(double) * 4);
      #ifdef DEBUG_DATA
      printf ("#%d at %lf Robot#%d  [%lf, %lf, %lf, %lf]\n", test, wb_robot_get_time(), i, outputData[0], outputData[1], outputData[2], outputData[3]); 
      #endif
    }
  };

  /* Enter your cleanup code here */
  printf("OUT OF LOOP\n");
  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}
