// Define the pins for this
#define dirPin    5
#define stepPin   6
#define dirPin_L  7
#define stepPin_L 8

//Add support for the radio.

int led = 13;
#define stepsPerRevolution 200   //This will depend on a few other things
#define MICRO_RATIO 1.0  //How much microstepping
#include "NRF52_MBED_TimerInterrupt.h"
#include <Arduino_LSM9DS1.h>  
#include <math.h>


NRF52_MBED_Timer ITimer0(NRF_TIMER_3); //What does this do? 

//Parameters for controlling the stepper motors
volatile uint32_t preMillisTimer0 = 0;
static bool toggle0 = false;
volatile int Steps=0;  //The number of steps taken by the controller
volatile int Steps_L=0;  //The number of steps taken by the controller

volatile float Right_Motor_Speed=0.0; //The speed of the right motor
volatile float Left_Motor_Speed=0.0; //The speed of the right motor

volatile int Right_Motor_Steps=0; //How many steps the right motor has taken
volatile int Left_Motor_Steps=0; //How many steps the right motor has taken

bool Right_Step_Mem=0;  //Whether the step pin is high or low
bool Left_Step_Mem=0;  

volatile float ticks_per_step;
volatile float ticks_per_step_L;


#define TIMER0_FREQ_HZ       100000.00 //55555.555  //  5555.555


void TimerHandler0()  //I could put this all in the main loop?  but it seems better to use the interupts
{

  //Could there be something bad here about low speed operation?  Or switching directions frequently?
  Steps++;
  //Right motor //Just need to know the rate that this is running at to compute the speed
  //if Right_Motor_Speed
  if (abs(Right_Motor_Speed)>.01) {
        ticks_per_step=TIMER0_FREQ_HZ/(MICRO_RATIO*2.0*abs(Right_Motor_Speed)*stepsPerRevolution/60.0);

        //What could I do?  If sign switchs, reset to 0?  that seems logical? Or count up and down the 
    if (Steps>(ticks_per_step)) { //If it is time to advance the motor
      if (Right_Motor_Speed>0.0) {
          digitalWrite(dirPin,HIGH);//  Spin Backwards
          Right_Motor_Steps=Right_Motor_Steps+1; //Increment the steps
      }
      else {
          digitalWrite(dirPin,LOW);//   Spin forwards 
          Right_Motor_Steps=Right_Motor_Steps-1; //Incrememnt the steps
      }     
      Right_Step_Mem =!Right_Step_Mem; //Switch to the other sign
      digitalWrite(stepPin,Right_Step_Mem);// 
      Steps=0;
      //digitalWrite(stepPin,LOW);//   //could try finding the minimum delay needed by the stepper? 
    }
  }

  //Repeat everything for the left motor
  Steps_L++;
  if (abs(Left_Motor_Speed)>.01) {
        ticks_per_step=TIMER0_FREQ_HZ/(MICRO_RATIO*2.0*abs(Left_Motor_Speed)*stepsPerRevolution/60.0);
  
    if (Steps_L>(ticks_per_step)) { //If it is time to advance the motor
      if (Left_Motor_Speed>0.0) {
          digitalWrite(dirPin_L,HIGH);//  Spin Backwards
          Left_Motor_Steps=Left_Motor_Steps+1; //Incrememnt the steps
      }
      else {
          digitalWrite(dirPin_L,LOW);//   Spin forwards 
          Left_Motor_Steps=Left_Motor_Steps-1; //Incrememnt the steps
      }     
      Left_Step_Mem =!Left_Step_Mem; //Switch the voltage on the step pin to the opposite
      digitalWrite(stepPin_L,Left_Step_Mem); 
      Steps_L=0;
      //digitalWrite(stepPin,LOW);//   //could try finding the minimum delay needed by the stepper? 
    }
  }
  
}


void RGB(int R, int G, int B) { digitalWrite(22, R); digitalWrite(23, G); digitalWrite(24, B); }


void setup() {
  // put your setup code here, to run once:

   pinMode(stepPin, OUTPUT);
   pinMode(dirPin, OUTPUT);
    // put your main code here, to run repeatedly:
  Serial.begin(115200);
  //while (!Serial);
  Serial.println("Started");

  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  
  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Acceleration in G's");
  Serial.println("X\tY\tZ");

  // Set up the interupts to run the motor. Frequency in float Hz
    if (ITimer0.attachInterrupt(TIMER0_FREQ_HZ, TimerHandler0))
    {
      Serial.print(F("Starting ITimer0 OK, millis() = ")); Serial.println(millis());
    }
    else
      Serial.println(F("Can't set ITimer0. Select another freq. or timer"));
  
}

long t;
long t_switch=0;
long t_min=600;
long last_t;
double dt=10.0/1000.0; //Time gap
double tilt=0.0;
double accel_all=0.0;
unsigned long currentMillis = millis();                     //  Update current time
float Pend_Angle=(Right_Motor_Steps%stepsPerRevolution)*360.0/(stepsPerRevolution*1.0);
float Angle_Des=0.0;
float Max_Speed=500.0; //Maximum Speed in RPMs

void loop() {
  float x, y, z;
  float x_g, y_g, z_g;

  unsigned long currentMillis = millis();                     //  Update current time

  //Measure the available information from the IMU- use the BN0080 to get these estimates, or just use this? 

  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(x_g, y_g, z_g);
  }

  if (IMU.accelerationAvailable()) {
    last_t=t;
    t=millis();
    dt=(t-last_t)/1000.0;
    IMU.readAcceleration(x, y, z);

    //If I have the gravity vector, I can just project it into the vertical plane to get the answer
    //Lots of instability when the disk is flat...

    //if I have the orientation, Could just project the world down vector into local frame
    
    //IMU.readAcceleration(x, y, z); 
    tilt=-atan2(x, y)*180.0/3.14; //The angle with the Z plane
    accel_all=sqrt(x*x+y*y+z*z);
    //Also compute the "tilt to ground"
    
  }

  
  
  //Manage the Position of the pendulum

  //What sort of controller to use for this thing?  maybe just a bang bang controller? 
                                                                                                                                                                                                                       
  Pend_Angle=(Right_Motor_Steps%stepsPerRevolution)*360.0/(stepsPerRevolution*1.0*MICRO_RATIO); //Need to include microstepping here? 
  Angle_Des=tilt;
  float Error_Angle=Angle_Des-Pend_Angle;
  float thresh=15.0;
  if (abs(Error_Angle)<thresh){
     //Right_Motor_Speed=0.0;
  } else {
    if (Error_Angle>0){
      //Right_Motor_Speed=Max_Speed;
    } else {
     // Right_Motor_Speed=-Max_Speed;
    }
  }
  Right_Motor_Speed=Max_Speed;
  Serial.print(Right_Motor_Steps);
  Serial.print('\t');
  Serial.print(ticks_per_step);
  Serial.print('\t');
  Serial.print(Angle_Des);
  Serial.print('\t');
  Serial.print(Pend_Angle);
  Serial.print('\t');
  Serial.print(Right_Motor_Speed);
  Serial.print('\t');
  Serial.println(tilt);
  
  //Device Kinematics
  
  
  //Compute the angle of the disk
  
  
  //Compute the tilt of the disk

  
  //The axial angular velocity

  //Control this thing to reach an equilibrium

  //For now, keep the mass at the point of contact
  

  
  

}
