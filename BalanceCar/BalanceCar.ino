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



MPU6050 mpu; //Instantiate one MPU6050 Object, the object name ismpu
int16_t axis[6]; // {ax, ay, az, gx, gy, gz}
//TB6612FNG Drive module control signal
#define IN1M 7
#define IN2M 6
#define IN3M 13
#define IN4M 12
#define PWMA 9
#define PWMB 10
#define STBY 8

#define PinA_left 2  //Interrupt 0
#define PinA_right 4 //Interrupt 1


//////////////////////////////////////////////
//////////// Interrupt timing ////////////////
//////////////////////////////////////////////

void inter()
{
  // Open interrupt. Due to the limitation of the AVR chip, no matter
  // any interrupt is entered, the chip will close the total interrupt
  // in the corresponding interrupt function, which will affect the angle
  // data obtained by the MPU. Therefore, the global interrupt operation must
  // be opened here. But in the timed interrupt, the code executed cannot exceed
  // 5ms, otherwise it will destroy the overall interrupt.
  sei();
  //IIC obtains MPU6050 six-axis data ax ay az gx gy gz
  mpu.getMotion6(&axis[0], &axis[1], &axis[2], &axis[3], &axis[4], &axis[5]);
  if (Serial.availableForWrite() >= (sizeof(axis)))
  {
    Serial.write('i');
    Serial.write((byte*)&axis[0], sizeof(axis));
  }
}

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

  // Open the serial port and set the baud rate to 115,200 baud (bits/sec)
  // N.B., if we're sending 6 16-bit integers per interrupt at a 5ms
  // period, we need a min one-way bitrate of 19,200 bits/sec to keep up.
  Serial.begin(115200);
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

int rx_tag, rx_pin, rx_val;

// Communicate over serial to receive commands
void serial_rx()
{
  while (Serial.available()) // Waiting for Bluetooth data
  {
    rx_tag = Serial.read();
    rx_pin = Serial.read();
    rx_val = Serial.read();
    switch (rx_tag)  // Read pin number, then read value
    {
      case 'd': // digital write
      {
        switch(rx_pin)
        {
          case 1: digitalWrite(Pin1, rx_val == 0 ? LOW : HIGH); break;
          case 2: digitalWrite(Pin2, rx_val == 0 ? LOW : HIGH); break;
          case 3: digitalWrite(Pin3, rx_val == 0 ? LOW : HIGH); break;
          case 4: digitalWrite(Pin4, rx_val == 0 ? LOW : HIGH); break;
          default: break; // this shouldn't happen...
        }
        break;
      }
      case 'a': // analogue write
      {
        switch(rx_pin)
        {
          case 5: analogWrite(PinPWMA, rx_val); break;
          case 6: analogWrite(PinPWMB, rx_val); break;
          default: break; // this shouldn't happen...
        }
      }
      default: break; // shouldn't happen...
    }
  }
}

////////////////////Pulse interrupt calculation////////////////////////////

void Code_left() {
  Serial.write('l');
} // Counting on the left speed code plate



void Code_right() {
  Serial.write('r');
} // Right speed code plate count




// ===       Main loop program body       ===
void loop() {

  // In the main function, cycle detection and superimposition
  // of pulses to determine the speed of the trolley. Use level
  // change to enter the pulse superposition. Increase the number
  // of pulses of the motor to ensure the accuracy of the trolley.
  attachInterrupt(0, Code_left, CHANGE);
  attachPinChangeInterrupt(PinA_right, Code_right, CHANGE);

  // receive commands over serial
  serial_rx();
}
