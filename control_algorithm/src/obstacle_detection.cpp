// Line-Follower Robot
// Author: Elizaveti Metreveli
// Description: Obstacle detection and avoidance behavior


#include "mbed.h"
#include "hcsr04.h"




AnalogIn S5(A3); //right
AnalogIn S4(A4);
AnalogIn S3(A5);
AnalogIn S2(A6);
AnalogIn S1(A0); //left


PwmOut rightfor(PA_11_ALT0); //D10
PwmOut rightback(D9);        
PwmOut leftfor(D11);          
PwmOut leftback(D12);


PwmOut servo(D7);


HCSR04 sensor(A1, A2);


const float SPEED_FORWARD = 0.3f;


const float SPEED_TURN = 0.5f;


void move_forward(){
    rightfor.write(SPEED_FORWARD);
    rightback.write(0.0f);
    leftfor.write(SPEED_FORWARD);
    leftback.write(0.0f);
}
void STOP(){
    rightfor.write(0.0f);
    rightback.write(0.0f);
    leftfor.write(0.0f);
    leftback.write(0.0f);


}


void TURN(){
    rightfor.write(SPEED_TURN);
    rightback.write(0.0f);
    leftfor.write(SPEED_TURN);
    leftback.write(0.0f);
}


void turn_left(){
    servo.write(0.068f);
    rightfor.write(SPEED_TURN);
    rightback.write(0.0f);
    leftfor.write(0.0f);
    leftback.write(0.0f);






}
void turn_right(){
    servo.write(0.128f);
    rightfor.write(0.0f);
    rightback.write(0.0f);
    leftfor.write(SPEED_TURN);
    leftback.write(0.0f);


}
void keep_neutral(){
    servo.write(0.098f);
}




float sensor_values[5];


float Kp = 0.008f; // constant for servo turning


float Kp2= 0.2; // constant for wheel speed adjustment


float Ki = 0.0f;
float Kd = 0.0f;


float Kd2=0.0f;


float previous_error = 0.0f;
float integral = 0.0f;






float calculateError() {
    float threshold = 0.6f;
    int weights[5] = {-2, -1, 0, 1, 2};


    static float last_known_position = 0.0f;


    static float last_error = 0.0f;


    float numerator = 0.0f;
    int denominator = 0;


    for (int i = 0; i < 5; ++i) {
        int active = (sensor_values[i]>threshold);
        numerator += weights[i] * active;
        denominator += active;
    }


    float position;
    if (denominator != 0) {
        position = numerator / (float)denominator;
        last_known_position = position;
        last_error = 0.0f - position;
    } else {
        position=last_known_position;


    }


   
    return 0.0f - position;
}


bool obstacle_detected(unsigned int threshold_cm) {
    sensor.start();  // Start measurement
    ThisThread::sleep_for(100ms);  

    unsigned int d = sensor.get_dist_cm();  // Get distance
    return (d <= threshold_cm);  // Return true if obstacle within threshold
}






int main()
{
    rightfor.period(0.02f); //Period for motors
    rightback.period(0.02f);
    leftfor.period(0.02f);
    leftback.period(0.02f);


    servo.period(0.02f); //Servo Motor


    Timer white_timer;
    bool white_detected = false; //stopping at white line


   


    while (true) {
         
        sensor_values[0] = S1.read();  // value between 0.0 and 1.0
        sensor_values[1] = S2.read();
        sensor_values[2] = S3.read();
        sensor_values[3] = S4.read();
        sensor_values[4] = S5.read();


        bool all_white = true;


        for (int i = 0; i < 5; ++i) {
            if (sensor_values[i] < 0.6f) {  // threshold for white detection
                all_white = false;
                break;
            }
        }


        if (all_white) {
            if (!white_detected) {
                white_detected = true;
                white_timer.reset();
                white_timer.start();
            } else {
                if (white_timer.elapsed_time() <1.0s) {
                    STOP();          
                    keep_neutral();  
                    while (true);     // Halt forever
                } else {
                    white_timer.reset();
                    white_timer.start();
                }
            }
        }






     




        float e =  calculateError();
       
        integral += e;
        float derivative = e - previous_error;
        float output = Kp * e + Ki * integral + Kd * derivative;
        previous_error = e;


        float output2=Kp2*e + Kd2*derivative ; // constant for controlling the speed


   


        float steering_angle = 0.098f - output;  
       
        // Clamp to safe limits
        if (steering_angle > 0.12f) steering_angle = 0.12f;
        if (steering_angle < 0.075f) steering_angle = 0.075f;




        float left_speed  = SPEED_FORWARD + output2;
        float right_speed = SPEED_FORWARD - output2;


      
        if (left_speed > 0.5f) left_speed = 0.5f;
        if (left_speed < 0.0f) left_speed = 0.0f;
        if (right_speed > 0.5f) right_speed = 0.5f;
        if (right_speed < 0.0f) right_speed = 0.0f;


        if(obstacle_detected(25)){
            STOP();
            thread_sleep_for(500);
            turn_right();
            thread_sleep_for(2000);
            keep_neutral();
            thread_sleep_for(500);
            turn_left();
            thread_sleep_for(2000);  
   
           


        }


        else {


            leftfor.write(left_speed);
            leftback.write(0.0f);
            rightfor.write(right_speed);
            rightback.write(0.0f);


            servo.write(steering_angle);
            thread_sleep_for(100);








        }




}





}
