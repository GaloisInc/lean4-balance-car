/****************************************************************************8
   Yabo Intelligent Technology Co., Ltd.
   Product Name：Arduino Smart balance car
   Product Model：BST-ABC ver 1.2

   See also https://create.arduino.cc/projecthub/gunjalsuyog/phpoc-arduino-self-balancing-robot-with-bt-web-control-0afab9

*/
// #include <PinChangeInt.h>

// #include <MsTimer2.h>
// //Realizing Speed ​​PID Control by Using Speed ​​Measuring Code Disk to Count

// //I2Cdev、MPU605 with PID_v1
// // The class library needs to be installed in advance Arduino 
// // under the class library folder.
// #include "I2Cdev.h"
// #include "MPU6050_6Axis_MotionApps20.h"
// #include "Wire.h"

#include <stdio.h>
#include <math.h>

#define PI 3.141592653589793238462643

struct BalanceCar {
  int pulseright;
  int pulseleft;
  int stopl;
  int stopr;
  double angleoutput;
  double pwm1;
  double pwm2;
  double speeds_filterold;
  double positions;
  int turnmax;
  int turnmin;
  double turnout;
  int flag1;
};

double constrain(double val, double low, double high)
{
    if (val < low) val = low;
    if (val > high) val = high;
    return val;
}

// MPU6050 mpu; //Instantiate one MPU6050 Object, the object name ismpu
struct BalanceCar car;

void initCar()
{
  car.pulseright = 0;
  car.pulseleft = 0;
  car.stopl = 0;
  car.stopr = 0;
  car.angleoutput = 0;
  car.pwm1 = 0;
  car.pwm2 = 0;
  car.speeds_filterold = 0;
  car.positions = 0;
  car.turnmax = 0;
  car.turnmin = 0;
  car.turnout = 0;
  car.flag1 = 0;
}

int ax, ay, az, gx, gy, gz;
//TB6612FNG Drive module control signal
#define IN1M 7
#define IN2M 6
#define IN3M 13
#define IN4M 12
#define PWMA 9
#define PWMB 10
#define STBY 8
#define kp_speed 3.1
#define ki_speed 0.05
#define kd_speed 0.0 // The parameters you need to modify
#define kp_turn 28
#define ki_turn 0
#define kd_turn 0.29 // Spin PID set up
#define kp 38
#define kd 0.58  // The parameters you need to modify
#define qAngle 0.001
#define qGyro 0.005
#define rAngle 0.5
#define c0 1
#define K1 0.05 // Weight of accelerometer value
#define angle0 0.00 // Mechanical balance angle
#define PinA_left 2  //Interrupt 0
#define PinA_right 4 //Interrupt 1
#define sampleIntervalMs 5
#define dt (sampleIntervalMs * 0.001) // Note: the value of dt is the filter sampling time





// Declare custom variables
int time;
double angle, angle_dot;  // Balance angle value
double Outputs = 0; //Speed DIP Set point, input, output



//********************angle data*********************//
double Gyro_y = 0; // Y-axis gyroscope data temporary storage
double Gyro_x = 0;
double Gyro_z = 0;
double angle6 = 0;


//********************angle data*********************//

//***************Kalman_Filter*********************//
double P[2][2] = {{ 1, 0 },
  { 0, 1 }
};
double Pdot[4] = { 0, 0, 0, 0};

double q_bias, PCt_0, PCt_1, E, K_0, K_1, t_0, t_1;
//***************Kalman_Filter*********************//

// Declare MPU6050 control and status variables



//*********************************************
//******************** speed count ************
//*********************************************
// The volatile long types are used to ensure that the 
// external interrupt pulse count value is used in other 
// functions to ensure that the value is valid
volatile long count_right = 0; 
volatile long count_left = 0;
int speedcc = 0;



////////////// Steering and rotation parameters ///////////////////////////////
int turncount = 0; // Turn to intervention time calculation


////////////// Steering and rotation parameters ///////////////////////////////

////////////// Bluetooth control volume ///////////////////
int front = 0; // Forward variable
int back = 0;  // Back variable
int turnl = 0; // Turn left sign
int turnr = 0; // Turn right sign
int spinl = 0; // Rotate left sign
int spinr = 0; // Right rotation sign

////////////////// Ultrasonic velocity //////////////////

int chaoshengbo = 0; // Ultrasound
int tingzhi = 0; // stop
int jishi = 0; // even if


void Kalman_Filter(double angle_m, double gyro_m)
{
  angle += (gyro_m - q_bias) * dt;
  double angle_err = angle_m - angle;
  Pdot[0] = qAngle - P[0][1] - P[1][0];
  Pdot[1] = - P[1][1];
  Pdot[2] = - P[1][1];
  Pdot[3] = qGyro;
  P[0][0] += Pdot[0] * dt;
  P[0][1] += Pdot[1] * dt;
  P[1][0] += Pdot[2] * dt;
  P[1][1] += Pdot[3] * dt;
  PCt_0 = c0 * P[0][0];
  PCt_1 = c0 * P[1][0];
  E = rAngle + c0 * PCt_0;
  K_0 = PCt_0 / E;
  K_1 = PCt_1 / E;
  t_0 = PCt_0;
  t_1 = c0 * P[0][1];
  P[0][0] -= K_0 * t_0;
  P[0][1] -= K_0 * t_1;
  P[1][0] -= K_1 * t_0;
  P[1][1] -= K_1 * t_1;
  angle += K_0 * angle_err; // Optimal angle
  q_bias += K_1 * angle_err;
  angle_dot = gyro_m - q_bias; // Optimal angular velocity
}

/////////////// Kalman filter calculation angle ////////////////////////
void Angletest()
{
  // int flag;
  // Balance parameter
  double Angle = atan2(ay , az) * 57.3; // Angle calculation formula
  Gyro_x = (gx - 128.1) / 131; // Angle conversion
  Kalman_Filter(Angle, Gyro_x); // Kalman filter
  // Rotation angle Z axis parameters
  if (gz > 32768) gz -= 65536; //Forced conversion 2g  1g
  Gyro_z = -gz / 131.0; // Z axis parameter conversion

  double angleAx = atan2(ax, az) * 180 / PI; // Calculate the angle with the x axis
  Gyro_y = -gy / 131.0; // Calculate angular velocity
  angle6 = K1 * angleAx + (1 - K1) * (angle6 + Gyro_y * dt);
}


//////////////////////BalanceCar Methods///////////////////////
double speedpiout()
{
  double speeds = (car.pulseleft + car.pulseright) * 1.0;
  car.pulseright = 0;
  car.pulseleft = 0;
  car.speeds_filterold *= 0.7;
  double speeds_filter = car.speeds_filterold + speeds * 0.3;
  car.speeds_filterold = speeds_filter;
  car.positions += speeds_filter;
  car.positions += front;
  car.positions += back;
  car.positions = constrain(car.positions, -3550,3550);
  double output = ki_speed * (0.0 - car.positions) + kp_speed * (0.0 - speeds_filter);
  if(car.flag1 == 1)
  {
    car.positions = 0;
  }
  
  return output;
}


double turnoutput = 0;

void turnspin()
{
  int spinonce = 0;
  double turnspeed = 0;
  double rotationratio = 0;
  
  if (turnl == 1 || turnr == 1 || spinl == 1 || spinr == 1)
  {
    if (spinonce == 0)
    {
      turnspeed = (car.pulseright + car.pulseleft);
      spinonce++;
    }
    if (turnspeed < 0)
    {
      turnspeed = -turnspeed;
    }
    if(turnl == 1 || turnr == 1)
    {
     car.turnmax = 3;
     car.turnmin = -3;
    }
    if( spinl == 1 || spinr == 1)
    {
      car.turnmax=10;
      car.turnmin=-10;
    }
    rotationratio = 5 / turnspeed;
    if (rotationratio < 0.5) rotationratio = 0.5;
    if (rotationratio > 5) rotationratio = 5;
  }
  else
  {
    rotationratio = 0.5;
    spinonce = 0;
    turnspeed = 0;
  }
  if (turnl == 1 || spinl == 1)
  {
    car.turnout += rotationratio;
  }
  else if (turnr == 1 || spinr == 1)
  {
    car.turnout -= rotationratio;
  }
  else
    car.turnout = 0;
  if (car.turnout > car.turnmax) car.turnout = car.turnmax;
  if (car.turnout < car.turnmin) car.turnout = car.turnmin;

  turnoutput = -car.turnout * kp_turn - Gyro_z * kd_turn;
}

void pwma(double speedoutput)
{

  car.pwm1 = -car.angleoutput - speedoutput - turnoutput;
  car.pwm2 = -car.angleoutput - speedoutput + turnoutput;

  if (car.pwm1 > 255)  car.pwm1 = 255;
  if (car.pwm1 < -255) car.pwm1 = -255;
  if (car.pwm2 > 255)  car.pwm2 = 255;
  if (car.pwm2 < -255) car.pwm2 = -255;

  if (angle > 30 || angle < -30)
  {
    car.pwm1 = 0;
    car.pwm2 = 0;
  }
  
  if ((angle6 > 10 || angle6 < -10) && turnl == 0 && turnr == 0 && spinl == 0 && spinr == 0 && front == 0 && back == 0)
  {
    if(car.stopl + car.stopr > 1500 || car.stopl + car.stopr < -3500)
    {
      car.pwm1  = 0;
      car.pwm2  = 0;
      car.flag1 = 1;
    }
  }
  else 
  {
   car.stopl = 0;
   car.stopr = 0;
   car.flag1 = 0;
  }
}





//////////////////////Pulse calculation///////////////////////
void countpulse()
{

  int lpluse = count_left;
  int rpluse = count_right;
  count_left = 0;
  count_right = 0;



  if ((car.pwm1 < 0) && (car.pwm2 < 0)) 
  {
    // Judgment of the direction of movement of the trolley.
    // When moving backwards (PWM means the motor voltage is negative), 
    // the number of pulses is negative.
    rpluse = -rpluse;
    lpluse = -lpluse;
  }
  else if ((car.pwm1 > 0) && (car.pwm2 > 0))
  {
    // Judgment of the direction of movement of the trolley.
    // When moving forward (PWM, that is, the motor voltage is
    // positive), the number of pulses is negative.
    rpluse = rpluse;
    lpluse = lpluse;
  }
  else if ((car.pwm1 < 0) && (car.pwm2 > 0)) 
  {
    // Judgment of the direction of movement of the trolley.
    // When moving forward (PWM, that is, the motor voltage is
    // positive), the number of pulses is negative.
    rpluse = rpluse;
    lpluse = -lpluse;
  }
  else if ((car.pwm1 > 0) && (car.pwm2 < 0))
  {
    // Judgment of the direction of movement of the trolley.
    // Rotate left, the number of right pulses is negative,
    // and the number of left pulses is positive.
    rpluse = -rpluse;
    lpluse = lpluse;
  }
  //  else if ((pwm1 == 0) && (pwm2 == 0))             
  //  {
  // //  Judgment of the direction of movement of the trolley.
  // //  Rotate right, the number of left pulses is negative,
  // //  and the number of right pulses is positive
  //  rpluse = 0;
  //  lpluse = 0;
  //  }

  // Raise judgment
  car.stopr += rpluse;
  car.stopl += lpluse;

  // When entering interrupt every 5ms, 
  // the number of pulses is superimposed.
  car.pulseright += rpluse;
  car.pulseleft += lpluse;

}


////////////////// Angle PD ////////////////////
void angleout()
{
  car.angleoutput = kp * (angle + angle0) + kd * Gyro_x;//PD 角度环控制
}

int motion_group = 0;

void getMotion6()
{
    switch (motion_group)
    {
        case 1:
            ax = ay = az = gx = gy = gz = 100; break;
        case 2:
            ax = ay = az = gx = gy = gz = 256; break;
        case 3:
            ax = ay = az = 90;
            gx = gy = gz = 256;
            break;
        case 4:
            ax = ay = az = gx = gy = gz = -100; break;
        case 5:
        case 6:
            ax = 5;
            ay = 10;
            az = 15;
            gx = 20;
            gy = 25;
            gz = 30;
            break;
        case 7:
            ax = 5;
            ay = 10;
            az = 15;
            gx = 0;
            gy = 5;
            gz = 8;
            break;
        case 8:
            ax = 50;
            ay = -10;
            az = -15;
            gx = -20;
            gy = -25;
            gz = -30;
            break;        
        case 0:
        default:
            motion_group = 0;
            ax = ay = az = gx = gy = gz = 0; break;
            break;
    }

    fprintf(
        stderr,
        "ax: %d, ay: %d, az: %d, gx: %d, gy: %d, gz: %d\n",
        ax, ay, az, gx, gy, gz);

    motion_group++;
}

void set_counts()
{
    switch (motion_group)
    {
        case 0:
        case 1:
            count_left = count_right = 0;
            break;
        case 2:
        case 3:
            count_left = count_right = 50;
            break;
        case 4:
            count_left = 25;
            count_right = 35;
            break;
        case 5:
            count_left = 25;
            count_right = 35;
            break;
        case 6:
            count_left = 30;
            count_right = 38;
            break;
        case 7:
            count_left = 2;
            count_right = 10;
            break;
        case 8:
            count_left = 20;
            count_right = 3;
            break;
        default:
            count_left = 34;
            count_right = 39;
            break;
    }
    fprintf(
        stderr,
        "count_left: %ld, count_right: %ld\n",
        count_left, count_right);
}


void digitalWrite(int pin, int pwm)
{
    //fprintf(stderr, "digitalWrite(%d, %d)\n", pin, pwm);
}

void analogWrite(int pin, int pwm)
{
    //fprintf(stderr, "analogWrite(%d, %d)\n", pin, pwm);
}

/////////////////////////////////////////////////////////////////////////////
////////////////// Interrupt timing 5ms timing interrupt ////////////////////
/////////////////////////////////////////////////////////////////////////////
void inter()
{
  set_counts();
  countpulse(); // Pulse superposition subroutine
  //IIC obtains MPU6050 six-axis data ax ay az gx gy gz
  // mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  getMotion6();
 
  
  // Get angle and Kalman filter
  Angletest();
  // Angle loop PD control
  angleout();
  turncount++;
  // 10ms to enter the rotation control
  if (turncount > 1)
  {
    // Rotation function
    turnspin();                                    
    turncount = 0;
  }
  speedcc++;
  // 50ms into speed loop control
  if (speedcc >= 10)
  {
    Outputs = speedpiout();
    speedcc = 0;
  }
  // Total PWM output of trolley
  pwma(Outputs);

  fprintf(stderr, "car.pwm1 = %f\n", car.pwm1);
  fprintf(stderr, "car.pwm2 = %f\n", car.pwm2);

  if (car.pwm1 >= 0) {
    digitalWrite(IN2M, 0);
    digitalWrite(IN1M, 1);
    analogWrite(PWMA, car.pwm1);
  } else {
    digitalWrite(IN2M, 1);
    digitalWrite(IN1M, 0);
    analogWrite(PWMA, -car.pwm1);
  }
  //电机的正负输出判断        右电机判断
  if (car.pwm2 >= 0) {
    digitalWrite(IN4M, 0);
    digitalWrite(IN3M, 1);
    analogWrite(PWMB, car.pwm2);
  } else {
    digitalWrite(IN4M, 1);
    digitalWrite(IN3M, 0);
    analogWrite(PWMB, -car.pwm2);
  }
}


void debugPrintCar()
{
  fprintf(stderr, "- - - - - - - - VARIABLES - - - - - - - - \n");  
  fprintf(stderr, "pulseright: %d\n", car.pulseright);
  fprintf(stderr, "pulseleft: %d\n", car.pulseleft);
  fprintf(stderr, "stopl: %d\n", car.stopl);
  fprintf(stderr, "stopr: %d\n", car.stopr);
  fprintf(stderr, "angleoutput: %lf\n", car.angleoutput);
  fprintf(stderr, "pwm1: %lf\n", car.pwm1);
  fprintf(stderr, "pwm2: %lf\n", car.pwm2);
  fprintf(stderr, "speeds_filterold: %lf\n", car.speeds_filterold);
  fprintf(stderr, "positions: %lf\n", car.positions);
  fprintf(stderr, "turnmax: %d\n", car.turnmax);
  fprintf(stderr, "turnmin: %d\n", car.turnmin);
  fprintf(stderr, "turnout: %lf\n", car.turnout);
  fprintf(stderr, "flag1: %d\n", car.flag1);
  fprintf(stderr, "angle: %lf\n", angle);  // Balance angle value
  fprintf(stderr, "angle_dot: %lf\n", angle_dot);
  fprintf(stderr, "Outputs: %lf\n", Outputs); //Speed DIP Set point, input, output
  fprintf(stderr, "Gyro_y: %lf\n", Gyro_y); // Y-axis gyroscope data temporary storage
  fprintf(stderr, "Gyro_x: %lf\n", Gyro_x);
  fprintf(stderr, "Gyro_z: %lf\n", Gyro_z);
  fprintf(stderr, "angle6: %lf\n", angle6);
  fprintf(stderr, "P00: %lf\n", P[0][0]);
  fprintf(stderr, "P01: %lf\n", P[0][1]);
  fprintf(stderr, "P10: %lf\n", P[1][0]);
  fprintf(stderr, "P11: %lf\n", P[1][1]);
  fprintf(stderr, "Pdot0: %lf\n", Pdot[0]);
  fprintf(stderr, "Pdot1: %lf\n", Pdot[1]);
  fprintf(stderr, "Pdot2: %lf\n", Pdot[2]);
  fprintf(stderr, "Pdot3: %lf\n", Pdot[3]);
  fprintf(stderr, "q_bias: %lf\n", q_bias);
  fprintf(stderr, "speedcc: %d\n", speedcc);
  fprintf(stderr, "turncount: %d\n", turncount); // Turn to intervention time calculation
  fprintf(stderr, "turnoutput: %lf\n", turnoutput);
  fprintf(stderr, "count_left: %ld\n", count_left);
  fprintf(stderr, "count_right: %ld\n", count_right);
  fprintf(stderr, "front: %d\n", front); // Forward variable
  fprintf(stderr, "back: %d\n", back);  // Back variable
  fprintf(stderr, "turnl: %d\n", turnl); // Turn left sign
  fprintf(stderr, "turnr: %d\n", turnr); // Turn right sign
  fprintf(stderr, "spinl: %d\n", spinl); // Rotate left sign
  fprintf(stderr, "spinr: %d\n", spinr); // Right rotation sign
}


int main()
{
  initCar();

  for (int i = 0; i < 10; i++)
  {
      fprintf(stderr, "\n");
      inter();
      fprintf(stderr, "\n");
      debugPrintCar();
  }
}
