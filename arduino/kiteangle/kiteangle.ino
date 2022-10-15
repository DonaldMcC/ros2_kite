
//. Motor driver shield- 2012 Copyright (c) Seeed Technology Inc.
// 
//  Original Author: Jimbo.we
//  Contribution: LG
//  Now modified to include ROS messaging and include a potentiometer reading as well
//  
//  This library is free software; you can redistribute it and/or
//  modify it under the terms of the GNU Lesser General Public
//  License as published by the Free Software Foundation; either
//  version 2.1 of the License, or (at your option) any later version.
//
//  This library is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//  Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public
//  License along with this library; if not, write to the Free Software
//  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

#include <ros.h>
#include <std_msgs/Int16.h>

ros::NodeHandle  nh;

//  Now trying to build in some safety - two aspects - maxright and maxleft value of the sensor
//  Will be set here and enforced in the callback - think we don't all left and right above
//  Those values but can go opposite way obviously
//  Second issue is motor commands being sent and resistor not changing
//  Think we stop when that happens too but probably after a second or so of movement

int MAXLEFT = 770;
int MAXRIGHT = 930;
int currdirection = 0;
bool motorson = false;
bool safetystop = false;
unsigned long startmotorstime;
unsigned long runtime;
int safetymove = 4;  // amount the resistance should change in safetycycle or safetystop goes true and motors no longer run 
int startsensor;
int sensormin;
int sensormax;
int safetycycle = 2000; // amount the resistor should move in period - if less motors stop until direction changes 
int storedirection;
int prevsensor = 0;

std_msgs::Int16 msg;
// Declaring String variable
std_msgs::Int16 int_msg; 

ros::Publisher kiteangle("kiteangle", &msg);

// Defining callback as unsigned and no measurement on motors we only really need to send
// left right or stop and logically 0 should be stop - possibly there will eventually be a range
// of speeds so we will go with first digit being direction and final two being speed 
// with 100-199 being left and 200-299 being right

void callback (const std_msgs::Int16&int_msg)
{
int speed = 255;
int rawspeed; 
int direction=0; //direction of motors
direction = int_msg.data / 100;
rawspeed = int_msg.data % 100;
if (rawspeed > 0) {
speed = int((rawspeed * 255) / 100);
}
else {
  speed = 255;
};

currdirection = direction;

if (safetystop == false) {
switch (direction) {
    case 1:
      backward(speed);
      break;
    case 2:
      forward(speed);
      break;
    case 3:
      left(speed);
      break;
    case 4:
      right(speed);
      break;
    case 6:
      leftonly(speed);
      break;
    case 7:
      rightonly(speed);
      break;
    default:
        stop();
    break;
  }
}
}

// Defining Subscriber 
ros::Subscriber<std_msgs::Int16> 
sub("motormsg", callback); 

int pinI1=8;//define I1 interface
int pinI2=11;//define I2 interface 
int speedpinA=9;//enable motor A
int pinI3=12;//define I3 interface 
int pinI4=13;//define I4 interface 
int speedpinB=10;//enable motor B
int spead=255;//define the spead of motor as fast as poss

int sensorPin = A0;    // select the input pin for the potentiometer
int sensorValue = 0;  // variable to store the value coming from the sensor

unsigned long previousmillis = 0;
unsigned long previoussensor = 0;

void setup()
{
  nh.initNode();
  nh.advertise(kiteangle);
  nh.subscribe(sub);
  pinMode(pinI1,OUTPUT);
  pinMode(pinI2,OUTPUT);
  pinMode(speedpinA,OUTPUT);
  pinMode(pinI3,OUTPUT);
  pinMode(pinI4,OUTPUT);
  pinMode(speedpinB,OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(57600);
}


void backward(int speed)
{
     analogWrite(speedpinA,speed);//input a simulation value to set the speed
     analogWrite(speedpinB,speed);
     digitalWrite(pinI4,HIGH);//turn DC Motor B move clockwise
     digitalWrite(pinI3,LOW);
     digitalWrite(pinI2,LOW);//turn DC Motor A move anticlockwise
     digitalWrite(pinI1,HIGH);
}

void forward(int speed)//
{
     analogWrite(speedpinA,speed);//input a simulation value to set the speed
     analogWrite(speedpinB,speed);
     digitalWrite(pinI4,LOW);//turn DC Motor B move anticlockwise
     digitalWrite(pinI3,HIGH);
     digitalWrite(pinI2,HIGH);//turn DC Motor A move clockwise
     digitalWrite(pinI1,LOW);
}

void left(int speed)//
{
     analogWrite(speedpinA,speed);//input a simulation value to set the speed
     analogWrite(speedpinB,speed);
     digitalWrite(pinI4,HIGH);//turn DC Motor B move clockwise
     digitalWrite(pinI3,LOW);
     digitalWrite(pinI2,HIGH);//turn DC Motor A move anticlockwise
     digitalWrite(pinI1,LOW);
}

void leftonly(int speed)//
{
     analogWrite(speedpinA,speed);//input a simulation value to set the speed
     analogWrite(speedpinB,speed);
     digitalWrite(pinI4,HIGH);//turn DC Motor B move clockwise
     digitalWrite(pinI3,LOW);
}

void right(int speed)//
{
     analogWrite(speedpinA,speed);//input a simulation value to set the speed
     analogWrite(speedpinB,speed);
     digitalWrite(pinI4,LOW);//turn DC Motor B move anticlockwise
     digitalWrite(pinI3,HIGH);
     digitalWrite(pinI2,LOW);//turn DC Motor A move clockwise
     digitalWrite(pinI1,HIGH);
}

void rightonly(int speed)//
{
     analogWrite(speedpinA,speed);//input a simulation value to set the speed
     analogWrite(speedpinB,speed);
     digitalWrite(pinI2,LOW);//turn DC Motor A move clockwise
     digitalWrite(pinI1,HIGH);
}

void stop()//
{
     digitalWrite(speedpinA,LOW);// Unenable the pin, to stop the motor. this should be done to avid damaging the motor. 
     digitalWrite(speedpinB,LOW);

}

void loop()
{
  sensorValue = analogRead(sensorPin);
  msg.data = sensorValue;
  kiteangle.publish(&msg);
  nh.spinOnce();

//some test code - should just go in the direction on startup but safety should still kick in
// currdirection = 3;
//  if (currdirection == 3 && safetystop == false) {
//    left(200);
//  };
 
  
  if (currdirection == 3 && sensorValue < MAXLEFT) {
    stop();
  };
 
  if (currdirection == 4 && sensorValue > MAXRIGHT) {
    stop();
  };      

  //Serial.print(runtime);
  //Serial.println();
  
  if (currdirection > 2) {
    if (!motorson) {
      startmotorstime = millis();
      startsensor = sensorValue;
      motorson = true;
      sensormax = sensorValue;
      sensormin = sensorValue;
      //Serial.print("motorson");
      //Serial.println();
    } else {
      // already running
      runtime = millis() - startmotorstime; 
      if (runtime > safetycycle) {
          if ((sensormax - sensormin) > safetymove) {
            //start a new interval
            startmotorstime = millis();
            sensormax = sensorValue;
            sensormin = sensorValue;
          } else {
            //stop the motors until direction changes
            safetystop = true;
            //Serial.print("instopzone");
            //Serial.println();
            storedirection = currdirection;
            stop();
          }
      } else {
        //update max and min cumulation was unreliable
        if (sensorValue > sensormax) {
          sensormax = sensorValue;
        }
        if (sensorValue < sensormin) {
          sensormin = sensorValue;
        }
      }
    }
  } else {
      // stopped moving
   motorson = false;        
  };  

  if (currdirection != storedirection) {
    safetystop = false;
  };
  
  prevsensor = sensorValue;    
  delay(20);
}
