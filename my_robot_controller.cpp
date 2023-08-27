#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/LightSensor.hpp>
#include <webots/Node.hpp>
#include <webots/Supervisor.hpp>
#include <cstdlib>

constexpr auto TIME_STEP = 64;
constexpr auto MAX_SPEED = 6.28;
constexpr auto NUM_IR_SENSORS = 4;
constexpr auto NUM_OPTICAL_SENSORS = 4;
constexpr auto NUM_ROBOTS = 5;

using namespace webots;

double** pso_algorithm(LightSensor* ls[], double g, double g_pos[], double x[], double x_hat[], double p, double v[])
{
    //ls --> array of light sensors on robot
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
    double j = ls[0]->getValue();
    for (int i = 1; i < NUM_OPTICAL_SENSORS; i++)
    {
        if (ls[i]->getValue() > j)
            j = ls[i]->getValue();
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
    double** return_array = new double* [2];
    for (int i = 0; i < 2; i++)
        return_array[i] = new double [7];

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
}

//bug0 algorithm
bool bug0_algorithm(DistanceSensor* ps[], double g_pos[], double x[])
{
    //ps --> array of IR distance sensors on robot
    //g_pos --> position of global best [x_pos,y_pos]
    //x --> 2 dimension array [x_pos,y_pos] of our current position
    //avoiding --> is the robot is the processing of avoiding an obstacle?

    //define variables
    const double min_distance = 70.0; //the closest the robot can get to an obstacle before it begins avoidance
    bool avoiding = false;

    //move towards goal - PSO algorithm will do this (no action required in this function)

    //determine if obstacle is present in front of bot
    //if so, turn until it is no longer in front of bot, instead it is approx 90 degrees so bot moves along the obstacles until it is at the end of obstacle
    //while (front_obstacle < min_distance) <-- Matt's job to configure the sensors
        //turn left<-- Kyle's job to manage movement
        //avoiding = true;
    //while (obstacle_right)
        //move forward
        //avoiding = true;
    return avoiding;
}


int main(int argc, char** argv) 
{
        //Lachlan's job --> to manage Supervisor and comms between bots

        // create the Robot instance.
        Robot* robot = new Robot();

        //initialise motors
        Motor* leftMotor = robot->getMotor("left wheel motor");
        Motor* rightMotor = robot->getMotor("right wheel motor");
        leftMotor->setPosition(INFINITY);
        rightMotor->setPosition(INFINITY);
        leftMotor->setVelocity(0.0);
        rightMotor->setVelocity(0.0);

        // initialize devices
        DistanceSensor* ps[NUM_IR_SENSORS];
        char psNames[NUM_IR_SENSORS][4] = {
          "ps0", "ps3","ps4","ps7"
        };

        for (int i = 0; i < NUM_IR_SENSORS; i++) {
            ps[i] = robot->getDistanceSensor(psNames[i]);
            ps[i]->enable(TIME_STEP);
        }

        //initialise light sensors
        LightSensor* ls[NUM_OPTICAL_SENSORS];
        char lsNames[NUM_OPTICAL_SENSORS][4] = {
            "ls1","ls3","ls4","ls6"
        };
        const int sampling_period = 100;
        for (int i = 0; i < NUM_OPTICAL_SENSORS; i++)
        {
            ls[i] = new LightSensor(lsNames[i]);
            ls[i]->enable(sampling_period);
        }

        //initalise PSO algorithm
        double pso_array[2][7];
        //pso_array = pso_algorithm(ls,0,[0,0],robot_position,[0,0],0,[0,0])
        
        while (robot->step(TIME_STEP) != -1) 
        {
            //if (!bug0_algorithm(ps,pso_array[1],robot_position))
                //if(!pso_algorithm(ls,pso_array[0][0],pso_array[1],robot_position,pso_array[3],pso_array[0][4],pso_array[5])[1][7])
                    //pso_array = pso_algorithm();
        }

        delete robot;
        return 0; //EXIT_SUCCESS
}
