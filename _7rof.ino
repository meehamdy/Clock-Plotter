// units: mm; microseconds; radians
// origin: bottom left of drawing surface (see sketchUp file)
// #define kalibration
#define SERVOFAKTOR 600

// length of arms
#define L1 35          //x and y servo 
#define L2 55.1          //pen handle 
#define L3 13.2        // z servo



// origin points of left and right servo
#define O1X 22         // space between x and origin of the first servo (Right)
#define O1Y -25       // space between y and origin of the first servo (same as the second) 
#define O2X 47        //space between x and origin of the second servo (Right)
#define O2Y -25        //space between y and origin of the second servo (same as the first)

// Zero-position of left and right servo
#define SERVOLEFTNULL 1890        //zero points defined as the left edge of the left servo
#define SERVORIGHTNULL 960        //zero points defined as the left edge of the right servo

// lift positions of lifting servo
#define LIFT0 1080      // on drawing surface
#define LIFT1 925       // between numbers
#define LIFT2 725       // going towards sweeper

#define LIFTSPEED 1500  // speed of liftimg arm, higher is slower

// servo width:

// Servo1: Write: 1080ms; Lift Sweep: 724; Lift Number: 924
// Servo2: 0.65-1.98
// Servo3: 1.1-1.8ms

#include <Wire.h>
#include <TimeLib.h>
#include <DS1307RTC.h>
#include <Servo.h>

int servoLift = 1500;

Servo servo1;  //
Servo servo2;  //
Servo servo3;  //

int val;    // variable to read the value from the analog pin

volatile double lastX = 75;
volatile double lastY = 43;

int last_min = 0;  //initialize the last min as 0 (changes while the code is running due to time change)

void setup()
{
  // Set current time to hours and mins only

  drawTo(75, 44); //moving the pen to this location at start to write the first number
  lift(0);        //moving the pen into the board to start writing
  servo1.attach(2);  //  lifting servo
  servo2.attach(3);  //  left servo
  servo3.attach(4);  //  right servo
  delay(1000);

}

void loop() {
  number(5, 25, 1, .9); 

}


void number(float bx, float by, int num, float scale) {
  

  switch (num) {
     case 1:                      //drawing L
      drawTo(bx + 12 * scale, by + 12 * scale);
      lift(0);
      drawTo(bx + 12 * scale, by + 0 * scale);
      drawTo(bx + 16 * scale, by + 0 * scale);
      lift(1);
      break;
  

       case 2:                   //drawing F
      drawTo(bx + 12 * scale, by + 12 * scale);
      lift(0);
      drawTo(bx + 8 * scale, by + 12 * scale);
      drawTo(bx + 8 * scale, by + 0 * scale);
      lift(1);
      drawTo(bx + 12 * scale, by + 8 * scale);
      lift(0);
      drawTo(bx + 8 * scale, by + 8 * scale);
      lift(1);
      
      break;
  

         case 3:                      //drawing H
      drawTo(bx + 12 * scale, by + 12 * scale);
      lift(0);
      drawTo(bx + 12 * scale, by + 0 * scale);
      
      lift(1);
      drawTo(bx + 16 * scale, by + 12 * scale);
      lift(0);
      drawTo(bx + 16 * scale, by + 0 * scale);
      lift(1);
      drawTo(bx + 12 * scale, by + 6 * scale);
      lift(0);
      drawTo(bx + 16 * scale, by + 6 * scale);
      lift(1);
      
      break;
  

     case 4:                      //drawing A
      drawTo(bx + 3 * scale, by + 3 * scale);
      lift(0);
      drawTo(bx + 6 * scale, by + 12 * scale);
      drawTo(bx + 6 * scale, by + 3 * scale);
      lift(1);
      drawTo(bx + 5 * scale, by + 8 * scale);
      lift(0);
      drawTo(bx + 5 * scale, by + 10 * scale);
      lift(1);
     
      break;
  }
  }

void lift(char lift) {
  switch (lift) {

    case 0:     //850       //Writing 

      if (servoLift >= LIFT0) {     
        while (servoLift >= LIFT0)
        {
          servoLift--;
          servo1.writeMicroseconds(servoLift);
          delayMicroseconds(LIFTSPEED);
        }
      }
      else {
        while (servoLift <= LIFT0) {
          servoLift++;
          servo1.writeMicroseconds(servoLift);
          delayMicroseconds(LIFTSPEED);

        }

      }

      break;

    case 1:              //150                  //lifting between numbers

      if (servoLift >= LIFT1) {
        while (servoLift >= LIFT1) {
          servoLift--;
          servo1.writeMicroseconds(servoLift);
          delayMicroseconds(LIFTSPEED);

        }
      }
      else {
        while (servoLift <= LIFT1) {
          servoLift++;
          servo1.writeMicroseconds(servoLift);
          delayMicroseconds(LIFTSPEED);
        }

      }

      break;

    case 2:                        //going up towards the eraser

      if (servoLift >= LIFT2) {
        while (servoLift >= LIFT2) {
          servoLift--;
          servo1.writeMicroseconds(servoLift);
          delayMicroseconds(LIFTSPEED);
        }
      }
      else {
        while (servoLift <= LIFT2) {
          servoLift++;
          servo1.writeMicroseconds(servoLift);
          delayMicroseconds(LIFTSPEED);
        }
      }
      break;
  }
}


void bogenUZS(float bx, float by, float radius, int start, int ende, float sqee) {
  float inkr = -0.05;
  float count = 0;

  do {
    drawTo(sqee * radius * cos(start + count) + bx,
           radius * sin(start + count) + by);
    count += inkr;
  }
  while ((start + count) > ende);

}

void bogenGZS(float bx, float by, float radius, int start, int ende, float sqee) {
  float inkr = 0.05;
  float count = 0;

  do {
    drawTo(sqee * radius * cos(start + count) + bx,
           radius * sin(start + count) + by);
    count += inkr;
  }
  while ((start + count) <= ende);
}


void drawTo(double pX, double pY) {
  double dx, dy, c;
  int i;

  // dx dy of new point
  dx = pX - lastX;
  dy = pY - lastY;
  //path lenght in mm, times 4 equals 4 steps per mm
  c = floor(4 * sqrt(dx * dx + dy * dy));

  if (c < 1) c = 1;

  for (i = 0; i <= c; i++) {
    // draw line point by point
    set_XY(lastX + (i * dx / c), lastY + (i * dy / c));

  }

  lastX = pX;
  lastY = pY;
}

double return_angle(double a, double b, double c) {
  // cosine rule for angle between c and a
  return acos((a * a + c * c - b * b) / (2 * a * c));
}

void set_XY(double Tx, double Ty)
{
  delay(1);
  double dx, dy, c, a1, a2, Hx, Hy;

  // calculate triangle between pen, servoLeft and arm joint
  // cartesian dx/dy
  dx = Tx - O1X;
  dy = Ty - O1Y;

  // polar lemgth (c) and angle (a1)
  c = sqrt(dx * dx + dy * dy); //
  a1 = atan2(dy, dx); //
  a2 = return_angle(L1, L2, c);

  servo2.writeMicroseconds(floor(((a2 + a1 - M_PI) * SERVOFAKTOR) + SERVOLEFTNULL));

  // calculate joinr arm point for triangle of the right servo arm
  a2 = return_angle(L2, L1, c);
  Hx = Tx + L3 * cos((a1 - a2 + 0.637) + M_PI); //36,5°
  Hy = Ty + L3 * sin((a1 - a2 + 0.637) + M_PI);

  // calculate triangle between pen joint, servoRight and arm joint
  dx = Hx - O2X;
  dy = Hy - O2Y;

  c = sqrt(dx * dx + dy * dy);
  a1 = atan2(dy, dx);
  a2 = return_angle(L1, (L2 - L3), c);

  servo3.writeMicroseconds(floor(((a1 - a2) * SERVOFAKTOR) + SERVORIGHTNULL));
}

