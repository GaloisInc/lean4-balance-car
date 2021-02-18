/****************************************************************************8
   Yabo Intelligent Technology Co., Ltd.
   Product Name：Arduino Smart balance car
   Product Model：BST-ABC ver 1.2

   See also https://create.arduino.cc/projecthub/gunjalsuyog/phpoc-arduino-self-balancing-robot-with-bt-web-control-0afab9

*/
#include <PinChangeInt.h>

#include <MsTimer2.h>
//Realizing Speed ​​PID Control by Using Speed ​​Measuring Code Disk to Count

//I2Cdev、MPU605 with PID_v1
// The class library needs to be installed in advance Arduino 
// under the class library folder.
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

// Uncomment the below `#define` to enter DEBUG mode, where 
// (1) the car will wait until it receives a byte before 
// starting all other processes, and (2) the car will relay
// left/right counts, data from the MPU6050, and computed
// PWM1/PWM2 values accross the serial connection during 
// the `inter` loop for debugging purposes.

// #define DEBUG


struct BalanceCar {
  int pulseright;
  int pulseleft;
  int stopl;
  int stopr;
  double angleoutput;
  double pwm1;
  double pwm2;
  float speeds_filterold;
  float positions;
  int turnmax;
  int turnmin;
  float turnout;
  int flag1;
};


MPU6050 mpu; //Instantiate one MPU6050 Object, the object name ismpu
BalanceCar car;
int16_t ax, ay, az, gx, gy, gz;
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


#define PinA_left 2  //Interrupt 0
#define PinA_right 4 //Interrupt 1





// Declare custom variables
float angle, angle_dot;  // Balance angle value
double Outputs = 0; //Speed DIP Set point, input, output



//********************angle data*********************//
float Q;
float Gyro_y; // Y-axis gyroscope data temporary storage
float Gyro_x;
float Gyro_z;
float angleAx;
float angle6;
#define K1 0.05 // Weight of accelerometer value
float Angle; // The final tilt angle of the trolley calculated by the first-order complementary filter
#define angle0 0.00 // Mechanical balance angle
float accelz = 0;

//********************angle data*********************//

//***************Kalman_Filter*********************//
float P[2][2] = {{ 1, 0 },
  { 0, 1 }
};
float Pdot[4] = { 0, 0, 0, 0};
#define qAngle 0.001
#define qGyro 0.005
#define rAngle 0.5
#define c0 1
float q_bias, angle_err, PCt_0, PCt_1, E, K_0, K_1, t_0, t_1;
#define sampleIntervalMs 5
#define dt (sampleIntervalMs * 0.001) // Note: the value of dt is the filter sampling time
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



////////////////////// Pulse calculation /////////////////////////
int lz = 0;
int rz = 0;
int rpluse = 0;
int lpluse = 0;

////////////// Steering and rotation parameters ///////////////////////////////
int turncount = 0; // Turn to intervention time calculation



float turnoutput = 0;




////////////// Steering and rotation parameters ///////////////////////////////

////////////// Bluetooth control volume ///////////////////
int front = 0; // Forward variable
int back = 0;  // Back variable
int turnl = 0; // Turn left sign
int turnr = 0; // Turn right sign
int spinl = 0; // Rotate left sign
int spinr = 0; // Right rotation sign


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
//////////////////////BalanceCar Methods///////////////////////
double speedpiout()
{
  float speeds = (car.pulseleft + car.pulseright) * 1.0;
  car.pulseright = 0;
  car.pulseleft = 0;
  car.speeds_filterold *= 0.7;
  float speeds_filter = car.speeds_filterold + speeds * 0.3;
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


float turnspin()
{
  int spinonce = 0;
  float turnspeed = 0;
  float rotationratio = 0;
  float turnout_put = 0;
  
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
      car.turnmax = 10;
      car.turnmin = -10;
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

  turnout_put = -car.turnout * kp_turn - Gyro_z * kd_turn;
  return turnout_put;
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

  lpluse = count_left;
  rpluse = count_right;
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


#ifdef DEBUG
volatile bool ready = false; // don't start sending data until we're ready to receive it all (we don't want to miss any)
// Temporary variables to collect values to be sent all at once 
// to an external process for comparison.
int16_t tx_ax, tx_ay, tx_az, tx_gx, tx_gy, tx_gz;
long tx_count_right, tx_count_left;
long iteration = 0;
#endif


/////////////////////////////////////////////////////////////////////////////
////////////////// Interrupt timing 5ms timing interrupt ////////////////////
/////////////////////////////////////////////////////////////////////////////
void inter()
{
  // Open interrupt. Due to the limitation of the AVR chip, no matter
  // any interrupt is entered, the chip will close the total interrupt
  // in the corresponding interrupt function, which will affect the angle
  // data obtained by the MPU. Therefore, the global interrupt operation must
  // be opened here. But in the timed interrupt, the code executed cannot exceed
  // 5ms, otherwise it will destroy the overall interrupt.
  sei();
#ifdef DEBUG
  if (!ready) return;
  tx_count_left = count_left; tx_count_right = count_right;
#endif
  
  countpulse(); // Pulse superposition subroutine
  //IIC obtains MPU6050 six-axis data ax ay az gx gy gz
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

#ifdef DEBUG
  tx_ax = ax; tx_ay = ay; tx_az = az; tx_gx = gx; tx_gy = gy; tx_gz = gz;
#endif
  
  // Get angle and Kalman filter
  Angletest();
  // Angle loop PD control
  angleout();
  turncount++;
  // 10ms to enter the rotation control
  if (turncount > 1)
  {
    // Rotation function
    turnoutput = turnspin();                                    
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

#ifdef DEBUG
  if (Serial.availableForWrite())
  {
    Serial.print(iteration); Serial.print(" ");
    Serial.print(tx_count_left); Serial.print(" ");
    Serial.print(tx_count_right); Serial.print(" ");
    Serial.print(tx_ax); Serial.print(" ");
    Serial.print(tx_ay); Serial.print(" ");
    Serial.print(tx_az); Serial.print(" ");
    Serial.print(tx_gx); Serial.print(" ");
    Serial.print(tx_gy); Serial.print(" ");
    Serial.print(tx_gz); Serial.print(" ");
    Serial.print(car.pwm1, 5); Serial.print(" ");
    Serial.println(car.pwm2, 5);
  }
  iteration++;
#endif
}
//////////////////////////////////////////////
//////////// Interrupt timing ////////////////
//////////////////////////////////////////////



// ===    初始设置     ===
void setup() {
  // TB6612FNGN Drive module control signal initialization

  // Control the direction of motor 1:
  // 01 is forward rotation, 10 is reverse rotation
  pinMode(IN1M, OUTPUT);
  pinMode(IN2M, OUTPUT);
  // Control the direction of motor 2:
  // 01 is forward rotation, 10 is reverse rotation
  pinMode(IN3M, OUTPUT);
  pinMode(IN4M, OUTPUT);
  // Left motor PWM
  pinMode(PWMA, OUTPUT);
  // Right motor PWM
  pinMode(PWMB, OUTPUT);
  // TB6612FNG enable
  pinMode(STBY, OUTPUT);


  // Initialize the motor drive module
  digitalWrite(IN1M, 0);
  digitalWrite(IN2M, 1);
  digitalWrite(IN3M, 1);
  digitalWrite(IN4M, 0);
  digitalWrite(STBY, 1);
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);

  // Speed Code Input
  pinMode(PinA_left, INPUT);
  pinMode(PinA_right, INPUT);

  // Join the I2C bus
  Wire.begin();

  Serial.begin(115200); // Open the serial port and set the baud rate to 115200
  delay(1500);
  mpu.initialize(); // Initialize MPU6050
  delay(2);




  // 5ms timer interrupt setting Use timer2
  // Note: Using timer2 will affect the PWM output of
  // pin3 pin11, because PWM uses timer to control the
  // duty ratio, so when using timer, pay attention to
  // check the corresponding timer pin.
  MsTimer2::set(5, inter);
  MsTimer2::start();


  // In the main function, cycle detection and superimposition
  // of pulses to determine the speed of the trolley. Use level
  // change to enter the pulse superposition. Increase the number
  // of pulses of the motor to ensure the accuracy of the trolley.
  attachInterrupt(0, Code_left, CHANGE);
  attachPinChangeInterrupt(PinA_right, Code_right, CHANGE);
  initCar();
}


//////////////////////// bluetooth //////////////////////
void control()
{
  while (Serial.available()) // Waiting for Bluetooth data
  {
    switch (Serial.read()) // Read Bluetooth data
    {
      case 'w': front = 700; break;  // go ahead
      case 's': back = -700; break;  // back
      case 'q': turnl = 1;   break;  // Turn left
      case 'e': turnr = 1;   break;  // Turn right
      case 'a': spinl = 1;   break;  // Rotate left
      case 'd': spinr = 1;   break;  // Rotate right
      case '1': 
      {
        turnl = 0;
        turnr = 0;
        front = 0;
        back  = 0;
        spinl = 0;
        spinr = 0;
        break; // Ensure that the button is released for parking operation
      }
      case '2':
      {
        spinl = 0;
        spinr = 0;
        front = 0;
        back = 0;
        turnl = 0;
        turnr = 0;
        break; // Ensure that the button is released for parking operation
      }
      case '3':
      {
        front = 0;
        back = 0;
        turnl = 0;
        turnr = 0;
        spinl = 0;
        spinr = 0;
        turnoutput = 0;
        break; // Ensure that the button is released for parking operation
      }
      default: 
      {
        front = 0;
        back = 0;
        turnl = 0;
        turnr = 0;
        spinl = 0;
        spinr = 0;
        turnoutput = 0;
        break;
      }
    }
  }
}


////////////////////////////////////////turn//////////////////////////////////



// ===       Main loop program body       ===
void loop() {
#ifdef DEBUG
  if (!ready && Serial.available())
  {
    ready = true;  
  }
#else
  control();
#endif
}

////////////////////////////////////////pwm///////////////////////////////////



////////////////////Pulse interrupt calculation////////////////////////////

void Code_left() {
#ifdef DEBUG
  if (!ready) return;
#endif
  count_left ++;

} // Counting on the left speed code plate



void Code_right() {
#ifdef DEBUG
  if (!ready) return;
#endif
  count_right ++;

} // Right speed code plate count


/////////////// Kalman filter calculation angle ////////////////////////
void Angletest()
{
  // int flag;
  // Balance parameter
  Angle = atan2(ay , az) * 57.3; // Angle calculation formula
  Gyro_x = (gx - 128.1) / 131; // Angle conversion
  Kalman_Filter(Angle, Gyro_x); // Kalman filter
  // Rotation angle Z axis parameters
  if (gz > 32768) gz -= 65536; //Forced conversion 2g  1g
  Gyro_z = -gz / 131.0; // Z axis parameter conversion
  accelz = az / 16.4;

  angleAx = atan2(ax, az) * 180 / PI; // Calculate the angle with the x axis
  Gyro_y = -gy / 131.00; // Calculate angular velocity
  angle6 = K1 * angleAx + (1 - K1) * (angle6 + Gyro_y * dt);
}


//////////// Kalman filter calculation angle /////////////////////////

//////////////////////// kalman /////////////////////////

void Kalman_Filter(double angle_m, double gyro_m)
{
  angle += (gyro_m - q_bias) * dt;
  angle_err = angle_m - angle;
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

////////////////////////kalman/////////////////////////
