#include <math.h>
#include <time.h>


class Pos_Control
{
    private:

        float roll_cmd, pitch_cmd, yaw_cmd, throttle_cmd; // control signals

        float kp = 0.5; // proportional gain
        float ki = 0.1; // integral gain
        float kd = 0.2; // derivative gain

        float vx, vy, vz;

        // assume the drone has a maximum speed of 1 m/s
        float max_speed = 1;

        float kp = 0.5;
        float ki = 0.1;
        float kd = 0.2;

        float integral_x = 0, integral_y = 0, integral_z = 0;
        float error_x_prev = 0, error_y_prev = 0, error_z_prev = 0;

        float dt = 0.01;
        bool _controller_free  = true, target_set = false;
        float derivative_x = 0, derivative_y = 0, derivative_z = 0;


    public:

        float x, y, z; // current position
        float vx, vy, vz; // current velocity
        float desired_x, desired_y, desired_z; // desired position

        


    /*
    Constructor
    */
    Pos_Control(float Kp, float Ki, float Kd)
    {
        kp = Kp, ki = Ki, kd = Kd;

    }


    /*
        fed position in NED Frame (in cms) 
    */

    void set_desired_position(float current_x, float current_y, float current_z, float X, float Y, float Z)
    {
        desired_x = X, desired_y = Y, desired_z = Z;
        x = current_x, y = current_y, z = current_z;
        
        target_set = true;
    }

    float get_roll_cmd()
    {
        if(!_controller_free)
        return roll_cmd;
    }

    float get_pitch_cmd()
    {
        if(!_controller_free)
        return pitch_cmd;
    }

    float get_yaw_cmd()
    {
        if(!_controller_free)
        return yaw_cmd;
    }

    float get_throttle_cmd()
    {
        if(!_controller_free)
        return throttle_cmd;
    }


    bool update_controller()
    {
        if(target_set)
            {
                target_set = false;
                _controller_free = false;
                
            }
    }


    void update()
        {


            if (! _controller_free)
            {
                            
                float error_x = desired_x - x;
                integral_x += error_x;
                float derivative_x = (error_x - error_x_prev)  / dt;

                roll_cmd = kp * error_x + ki * integral_x + kd * derivative_x;

                error_x_prev = error_x;

                float error_y = desired_y - y;
                integral_y += error_y;
                derivative_y = (error_y - error_y_prev) / dt;

                pitch_cmd = kp * error_y + ki * integral_y + kd * derivative_y;

                error_y_prev = error_y;

                float error_z = desired_z - z;
                integral_z += error_z;
                derivative_z = error_z - error_z_prev;

                throttle_cmd = kp * error_z + ki * integral_z + kd * derivative_z;

                error_z_prev = error_z;

                
                vx += roll_cmd * max_speed;
                vy += pitch_cmd * max_speed;
                vz += throttle_cmd * max_speed;

                x += vx * dt;
                y += vy * dt;
                z += vz * dt;

                // check if we have reached the desired position
                float distance = sqrt((x - desired_x) * (x - desired_x) + (y - desired_y) * (y - desired_y) + (z - desired_z) * (z - desired_z));

                if (distance < 0.1) { // assume we have reached the position when we are within 10 cm of the target
                    clear_state();
                    }

                    // should return pitch_cmd, roll_cmd, yaw_cmd, throttle_cmd
            }

        }

    void clear_state()
    {
        integral_x = 0, integral_y = 0, integral_z = 0;
        error_x_prev = 0, error_y_prev = 0, error_z_prev = 0;
        _controller_free = true;
        roll_cmd = 0, pitch_cmd = 0, yaw_cmd = 0, throttle_cmd = 0;
        derivative_x = 0, derivative_y = 0, derivative_z = 0;
        x = 0, y = 0; z = 0;
        vx = 0, vy = 0, vz = 0;
        

    }

};




void run(float desired_x, float desired_y, float desired_z, )
{

    float x, y, z;
    float vx, vy, vz;
    float roll_cmd, pitch_cmd, yaw_cmd, throttle_cmd;

    float kp = 0.5;
    float ki = 0.1;
    float kd = 0.2;

    float integral_x = 0, integral_y = 0, integral_z = 0;
    float error_x_prev = 0, error_y_prev = 0, error_z_prev = 0;

    float dt = 0.01;

    while (true) {

        /*
        Implement the control algorithm: Choose a control algorithm that suits your needs, 
        such as a proportional-integral-derivative (PID) controller. Implement the algorithm in C++ code. 
        Here's an example PID controller for the x-axis:
        */ 

        float error_x = desired_x - x;
        integral_x += error_x;
        float derivative_x = error_x - error_x_prev;

        roll_cmd = kp * error_x + ki * integral_x + kd * derivative_x;

        error_x_prev = error_x;

        float error_y = desired_y - y;
        integral_y += error_y;
        float derivative_y = (error_y - error_y_prev) / dt;

        pitch_cmd = kp * error_y + ki * integral_y + kd * derivative_y;

        error_y_prev = error_y;

        float error_z = desired_z - z;
        integral_z += error_z;
        float derivative_z = error_z - error_z_prev;

        throttle_cmd = kp * error_z + ki * integral_z + kd * derivative_z;

        error_z_prev = error_z;

        // assume the drone has a maximum speed of 1 m/s
        float max_speed = 1;
        vx += roll_cmd * max_speed;
        vy += pitch_cmd * max_speed;
        vz += throttle_cmd * max_speed;

        x += vx * dt;
        y += vy * dt;
        z += vz * dt;

        // check if we have reached the desired position
        float distance = sqrt((x - desired_x) * (x - desired_x) + (y - desired_y) * (y - desired_y) + (z - desired_z) * (z - desired_z));

    if (distance < 0.1) { // assume we have reached the position when we are within 10 cm of the target
        break;

        }

    }


    // should return pitch_cmd, roll_cmd, yaw_cmd, throttle_cmd

    
}

