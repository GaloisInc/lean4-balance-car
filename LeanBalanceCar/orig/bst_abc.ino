/****************************************************************************8
   Yabo Intelligent Technology Co., Ltd.
   Product Name：Arduino Smart balance car
   Product Model：BST-ABC ver 1.2

   See also https://create.arduino.cc/projecthub/gunjalsuyog/phpoc-arduino-self-balancing-robot-with-bt-web-control-0afab9

*/
#include <PinChangeInt.h>

#include <MsTimer2.h>
//Realizing Speed ​​PID Control by Using Speed ​​Measuring Code Disk to Count

#include <BalanceCar.h>

//I2Cdev、MPU605 with PID_v1
// The class library needs to be installed in advance Arduino 
// under the class library folder.
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"



MPU6050 mpu; //Instantiate one MPU6050 Object, the object name ismpu
BalanceCar balancecar;
int16_t ax, ay, az, gx, gy, gz;
//TB6612FNG Drive module control signal
#define IN1M 7
#define IN2M 6
#define IN3M 13
#define IN4M 12
#define PWMA 9
#define PWMB 10
#define STBY 8

#define tx A0
#define rx A1



#define PinA_left 2  //Interrupt 0
#define PinA_right 4 //Interrupt 1



// Declare custom variables
int time;
byte inByte; // Serial port receive byte
int num;
float angle, angle_dot;  // Balance angle value
double Setpoint;  // Angle DIP Set point, input, output
double kp = 38, ki = 0.0, kd = 0.58;  // The parameters you need to modify
double Setpoints, Outputs = 0; //Speed DIP Set point, input, output
double kp_speed = 3.1, ki_speed = 0.05, kd_speed = 0.0; // The parameters you need to modify
double kp_turn = 28, ki_turn = 0, kd_turn = 0.29; // Spin PID set up
// Turn to PID parameter

double setp0 = 0, dpwm = 0, dl = 0; // Angle balance point, PWM difference, dead zone, PWM1, PWM2
float value;


//********************angle data*********************//
float Q;
float Gyro_y; // Y-axis gyroscope data temporary storage
float Gyro_x;
float Gyro_z;
float Angle_ax; // Tilt angle calculated from acceleration
float Angle_ay;
float angleAx;
float angle6;
float K1 = 0.05; // Weight of accelerometer value
float Angle; // The final tilt angle of the trolley calculated by the first-order complementary filter
float angle0 = 0.00; // Mechanical balance angle
float accelz = 0;
int slong;

//********************angle data*********************//

//***************Kalman_Filter*********************//
float P[2][2] = {{ 1, 0 },
  { 0, 1 }
};
float Pdot[4] = { 0, 0, 0, 0};
float Q_angle = 0.001, Q_gyro = 0.005; // Confidence of angular data, confidence of angular velocity data
float R_angle = 0.5 , C_0 = 1;
float q_bias, angle_err, PCt_0, PCt_1, E, K_0, K_1, t_0, t_1;
float timeChange = 5; // Filter method sampling interval milliseconds
float dt = timeChange * 0.001; // Note: the value of dt is the filter sampling time
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

////////////////// Ultrasonic velocity //////////////////

int chaoshengbo = 0; // Ultrasound
int tingzhi = 0; // stop
int jishi = 0; // even if



//////////////////////Pulse calculation///////////////////////
void countpluse()
{

  lz = count_left;
  rz = count_right;
  count_left = 0;
  count_right = 0;

  lpluse = lz;
  rpluse = rz;


  if ((balancecar.pwm1 < 0) && (balancecar.pwm2 < 0)) 
  {
    // Judgment of the direction of movement of the trolley.
    // When moving backwards (PWM means the motor voltage is negative), 
    // the number of pulses is negative.
    rpluse = -rpluse;
    lpluse = -lpluse;
  }
  else if ((balancecar.pwm1 > 0) && (balancecar.pwm2 > 0))
  {
    // Judgment of the direction of movement of the trolley.
    // When moving forward (PWM, that is, the motor voltage is
    // positive), the number of pulses is negative.
    rpluse = rpluse;
    lpluse = lpluse;
  }
  else if ((balancecar.pwm1 < 0) && (balancecar.pwm2 > 0)) 
  {
    // Judgment of the direction of movement of the trolley.
    // When moving forward (PWM, that is, the motor voltage is
    // positive), the number of pulses is negative.
    rpluse = rpluse;
    lpluse = -lpluse;
  }
  else if ((balancecar.pwm1 > 0) && (balancecar.pwm2 < 0))
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
  balancecar.stopr += rpluse;
  balancecar.stopl += lpluse;

  // When entering interrupt every 5ms, 
  // the number of pulses is superimposed.
  balancecar.pulseright += rpluse;
  balancecar.pulseleft += lpluse;

}


////////////////// Angle PD ////////////////////
void angleout()
{
  balancecar.angleoutput = kp * (angle + angle0) + kd * Gyro_x;//PD 角度环控制
}

#define INIT_FUEL 200  // printing every 200 5ms interrupts == every 1 seconds
int fuel = INIT_FUEL; 

/////////////////////////////////////////////////////////////////////////////
////////////////// Interrupt timing 5ms timing interrupt ////////////////////
/////////////////////////////////////////////////////////////////////////////
void inter()
{
  fuel -= 1;
  // Open interrupt. Due to the limitation of the AVR chip, no matter
  // any interrupt is entered, the chip will close the total interrupt
  // in the corresponding interrupt function, which will affect the angle
  // data obtained by the MPU. Therefore, the global interrupt operation must
  // be opened here. But in the timed interrupt, the code executed cannot exceed
  // 5ms, otherwise it will destroy the overall interrupt.
  sei();
  countpluse(); // Pulse superposition subroutine
  //IIC obtains MPU6050 six-axis data ax ay az gx gy gz
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  if (fuel <= 0)
  {
    fuel = INIT_FUEL;
    Serial.print("ax: "); Serial.print(ax, DEC);
    Serial.print(", ay: "); Serial.print(ay, DEC);
    Serial.print(", az: "); Serial.print(az, DEC);
    Serial.print(", gx: "); Serial.print(gx, DEC);
    Serial.print(", gy: "); Serial.print(gy, DEC);
    Serial.print(", gz: "); Serial.println(gz, DEC);
  }
  
  // Get angle and Kalman filter
  Angletest();
  // Angle loop PD control
  angleout();
  turncount++;
  // 10ms to enter the rotation control
  if (turncount > 1)
  {
    // Rotation function
    turnoutput = balancecar.turnspin(turnl,turnr,spinl,spinr,kp_turn,kd_turn,Gyro_z);                                    
    turncount = 0;
  }
  speedcc++;
  // 50ms into speed loop control
  if (speedcc >= 10)
  {
    Outputs = balancecar.speedpiout(kp_speed,ki_speed,kd_speed,front,back,setp0);
    speedcc = 0;
  }
  balancecar.posture++;
  // Total PWM output of trolley
  balancecar.pwma(Outputs,turnoutput,angle,angle6, turnl,turnr,spinl,spinr,front,back,accelz,IN1M,IN2M,IN3M,IN4M,PWMA,PWMB);
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

  Serial.begin(115200); // Open the serial port and set the baud rate to 9600
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

  // In the main function, cycle detection and superimposition
  // of pulses to determine the speed of the trolley. Use level
  // change to enter the pulse superposition. Increase the number
  // of pulses of the motor to ensure the accuracy of the trolley.
  attachInterrupt(0, Code_left, CHANGE);
  attachPinChangeInterrupt(PinA_right, Code_right, CHANGE);

  // control
  control();


}

////////////////////////////////////////pwm///////////////////////////////////



////////////////////Pulse interrupt calculation////////////////////////////

void Code_left() {

  count_left ++;

} // Counting on the left speed code plate



void Code_right() {

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
  Gyro_z = -gz / 131; // Z axis parameter conversion
  accelz = az / 16.4;

  angleAx = atan2(ax, az) * 180 / PI; // Calculate the angle with the x axis
  Gyro_y = -gy / 131.00; // Calculate angular velocity
  Yijielvbo(angleAx, Gyro_y); // First order filter


}

//////////////////////////yijielvbo////////////////////
void Yijielvbo(float angle_m, float gyro_m)
{
  angle6 = K1 * angle_m + (1 - K1) * (angle6 + gyro_m * dt);
}



//////////// Kalman filter calculation angle /////////////////////////

//////////////////////// kalman /////////////////////////

void Kalman_Filter(double angle_m, double gyro_m)
{
  angle += (gyro_m - q_bias) * dt;
  angle_err = angle_m - angle;
  Pdot[0] = Q_angle - P[0][1] - P[1][0];
  Pdot[1] = - P[1][1];
  Pdot[2] = - P[1][1];
  Pdot[3] = Q_gyro;
  P[0][0] += Pdot[0] * dt;
  P[0][1] += Pdot[1] * dt;
  P[1][0] += Pdot[2] * dt;
  P[1][1] += Pdot[3] * dt;
  PCt_0 = C_0 * P[0][0];
  PCt_1 = C_0 * P[1][0];
  E = R_angle + C_0 * PCt_0;
  K_0 = PCt_0 / E;
  K_1 = PCt_1 / E;
  t_0 = PCt_0;
  t_1 = C_0 * P[0][1];
  P[0][0] -= K_0 * t_0;
  P[0][1] -= K_0 * t_1;
  P[1][0] -= K_1 * t_0;
  P[1][1] -= K_1 * t_1;
  angle += K_0 * angle_err; // Optimal angle
  q_bias += K_1 * angle_err;
  angle_dot = gyro_m - q_bias; // Optimal angular velocity
}

////////////////////////kalman/////////////////////////
