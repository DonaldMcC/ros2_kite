//This will sort of follow the main kiteserial but for use with arduino and no motors 
//for testing on assumption that motors move at set pace 
//we might remove the safety element and a few things but keep std structure


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
unsigned int motormsg;

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

const byte buffSize = 40;
char inputBuffer[buffSize];
const char startMarker = '<';
const char endMarker = '>';
byte bytesRecvd = 0;
boolean readInProgress = false;
boolean newDataFromPC = false;

char messageFromPC[buffSize] = {0};

unsigned long curMillis;
unsigned long prevReplyToPCmillis = 0;
unsigned long replyToPCinterval = 1000;


// Defining callback as unsigned and no measurement on motors we only really need to send
// left right or stop and logically 0 should be stop - possibly there will eventually be a range
// of speeds so we will go with first digit being direction and final two being speed
// with 100-199 being left and 200-299 being right

void callback()
{
int speed = 255;
int rawspeed;
int direction=0; //direction of motors
direction = motormsg / 100;
rawspeed = motormsg % 100;
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
      left_only_forward(speed);
      break;
    case 7:
      left_only_backward(speed);
      break;
    case 8:
      right_only_forward(speed);
      break;
    case 9:
      right_only_backward(speed);
      break;
    default:
        stop();
    break;
  }
}
}


void setup()
{
  pinMode(pinI1,OUTPUT);
  pinMode(pinI2,OUTPUT);
  pinMode(speedpinA,OUTPUT);
  pinMode(pinI3,OUTPUT);
  pinMode(pinI4,OUTPUT);
  pinMode(speedpinB,OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(57600);
  delay(500); // delay() is OK in setup as it only happens once
  // tell the PC we are ready
  Serial.println("<Arduino is ready>");
}


void backward(int speed)  //1
{
     analogWrite(speedpinA,speed);//input a simulation value to set the speed
     analogWrite(speedpinB,speed);
     digitalWrite(pinI4,HIGH);//turn DC Motor B move clockwise
     digitalWrite(pinI3,LOW);
     digitalWrite(pinI2,LOW);//turn DC Motor A move anticlockwise
     digitalWrite(pinI1,HIGH);
}

void forward(int speed) //2
{
     analogWrite(speedpinA,speed);//input a simulation value to set the speed
     analogWrite(speedpinB,speed);
     digitalWrite(pinI4,LOW);//turn DC Motor B move anticlockwise
     digitalWrite(pinI3,HIGH);
     digitalWrite(pinI2,HIGH);//turn DC Motor A move clockwise
     digitalWrite(pinI1,LOW);
}

void left(int speed) //3
{
     analogWrite(speedpinA,speed);//input a simulation value to set the speed
     analogWrite(speedpinB,speed);
     digitalWrite(pinI4,HIGH);//turn DC Motor B move clockwise
     digitalWrite(pinI3,LOW);
     digitalWrite(pinI2,HIGH);//turn DC Motor A move anticlockwise
     digitalWrite(pinI1,LOW);
}

void left_only_forward(int speed)//6
{
     analogWrite(speedpinA,speed);//input a simulation value to set the speed
     analogWrite(speedpinB,speed);
     digitalWrite(pinI4,HIGH);//turn DC Motor B move clockwise
     digitalWrite(pinI3,LOW);
}

void left_only_backward(int speed)//
{
     analogWrite(speedpinA,speed);//input a simulation value to set the speed
     analogWrite(speedpinB,speed);
     digitalWrite(pinI4,LOW);//turn DC Motor B move clockwise
     digitalWrite(pinI3,HIGH);
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

void right_only_forward(int speed)//
{
     analogWrite(speedpinA,speed);//input a simulation value to set the speed
     analogWrite(speedpinB,speed);
     digitalWrite(pinI2,LOW);//turn DC Motor A move clockwise
     digitalWrite(pinI1,HIGH);
}

void right_only_backward(int speed)//
{
     analogWrite(speedpinA,speed);//input a simulation value to set the speed
     analogWrite(speedpinB,speed);
     digitalWrite(pinI2,HIGH);//turn DC Motor A move clockwise
     digitalWrite(pinI1,LOW);
}
void stop()//
{
     digitalWrite(speedpinA,LOW);// Unenable the pin, to stop the motor. this should be done to avid damaging the motor.
     digitalWrite(speedpinB,LOW);
}



void loop()
{
  curMillis = millis();
  getDataFromPC();
  callback();
  replyToPC();

  if (currdirection == 3 && sensorValue < MAXLEFT) {
    stop();
  };

  if (currdirection == 4 && sensorValue > MAXRIGHT) {
    stop();
  };

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
  delay(10);
}


void getDataFromPC() {
    // receive data from PC and save it into inputBuffer
  if(Serial.available() > 0) {
    char x = Serial.read();
      // the order of these IF clauses is significant

    if (x == endMarker) {
      readInProgress = false;
      newDataFromPC = true;
      inputBuffer[bytesRecvd] = 0;
      parseData();
    }

    if(readInProgress) {
      inputBuffer[bytesRecvd] = x;
      bytesRecvd ++;
      if (bytesRecvd == buffSize) {
        bytesRecvd = buffSize - 1;
      }
    }

    if (x == startMarker) {
      bytesRecvd = 0;
      readInProgress = true;
    }
  }
}

//=============

void parseData() {
    // split the data into its parts
  char * strtokIndx; // this is used by strtok() as an index
  strtokIndx = strtok(inputBuffer,",");      // get the first part - the string
  strcpy(messageFromPC, strtokIndx); // copy it to messageFromPC

  strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
  motormsg = atoi(strtokIndx);     // convert this part to an integer
}
//=============

void replyToPC() {
  // for mock we replace below wih assumed movement based on time elapsed
  //sensorValue = analogRead(sensorPin);

  if (newDataFromPC) {
    newDataFromPC = false;
    Serial.print("<Msg ");
    Serial.print(motormsg);
    Serial.print(" Sensor ");
    Serial.print(sensorValue);
    Serial.print(" Time ");
    Serial.print(curMillis >> 9); // divide by 512 is approx = half-seconds
    Serial.println(">");
  }
}
