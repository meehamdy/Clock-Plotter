#define SERVOFAKTOR 650

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
volatile double lastY = 35;

int last_min = 0;  //initialize the last min as 0 (changes while the code is running due to time change)

void setup()
{
  // Set current time to hours and mins only

  drawTo(75, 35); //moving the pen to this location at start to write the first number
  lift(0);        //moving the pen into the board to start writing
  servo1.attach(2);  //  lifting servo
  servo2.attach(3);  //  left servo
  servo3.attach(4);  //  right servo
  delay(1000);

}

void loop() {
  tmElements_t tm;
  RTC.read(tm);
  int i = 0;
  if (last_min != tm.Minute) {     //changing the time each min
    lift(0);
    while ((i + 1) * 10 <= tm.Hour)           // writing hours in two digits
    {
      i++;
    }
    number(5, 5, 111, 2);                   // clear screen
    number(7, 10, i, 1.2);                   // first digit defined as 0 or 1 or 2 only
    number(14, 10, (tm.Hour - i * 10), 1.2);   // second digit
    number(20, 10, 11, 1.2);                   // ':'
    i = 0;                                      // reseting the value
    while ((i + 1) * 10 <= tm.Minute)              //writing min in two digits
    {
      i++;
    }
    number(26, 10, i, 1.2);                       // first digit
    number(32, 10, (tm.Minute - i * 10), 1.2);     // second digit
    lift(2);                                      //lifting the pen up to erase

    drawTo(75, 35);     // eraser position
    lift(1);
    last_min = tm.Minute;  //keep tracking time
  }
}

// Writing numeral with bx by being the bottom left originpoint. Scale 1 equals a 20 mm high font.
void number(float bx, float by, int num, float scale) {

  switch (num) {

    case 0:                      //drawing 0
      drawTo(bx + 12 * scale, by + 6 * scale);
      lift(0);
      bogenGZS(bx + 7 * scale, by + 10 * scale, 10 * scale, -0.8, 6.7, 0.5);
      lift(1);
      break;
    case 1:                     //drawing 1

      drawTo(bx + 3 * scale, by + 15 * scale);
      lift(0);
      drawTo(bx + 10 * scale, by + 20 * scale);
      drawTo(bx + 10 * scale, by + 0 * scale);
      lift(1);
      break;
    case 2:                    //drawing 2
      drawTo(bx + 2 * scale, by + 12 * scale);
      lift(0);
      bogenUZS(bx + 8 * scale, by + 14 * scale, 6 * scale, 3, -0.8, 1);
      drawTo(bx + 1 * scale, by + 0 * scale);
      drawTo(bx + 12 * scale, by + 0 * scale);
      lift(1);
      break;
    case 3:                   //drawing 3
      drawTo(bx + 2 * scale, by + 17 * scale);
      lift(0);
      bogenUZS(bx + 5 * scale, by + 15 * scale, 5 * scale, 3, -2, 1);
      bogenUZS(bx + 5 * scale, by + 5 * scale, 5 * scale, 1.57, -3, 1);
      lift(1);
      break;
    case 4:                   //drawing 4
      drawTo(bx + 10 * scale, by + 0 * scale);
      lift(0);
      drawTo(bx + 10 * scale, by + 20 * scale);
      drawTo(bx + 2 * scale, by + 6 * scale);
      drawTo(bx + 12 * scale, by + 6 * scale);
      lift(1);
      break;
    case 5:                   //drawing 5
      drawTo(bx + 2 * scale, by + 5 * scale);
      lift(0);
      bogenGZS(bx + 5 * scale, by + 6 * scale, 6 * scale, -2.5, 2, 1);
      drawTo(bx + 5 * scale, by + 20 * scale);
      drawTo(bx + 12 * scale, by + 20 * scale);
      lift(1);
      break;
    case 6:                   //drawing 6
      drawTo(bx + 2 * scale, by + 10 * scale);
      lift(0);
      bogenUZS(bx + 7 * scale, by + 6 * scale, 6 * scale, 2, -4.4, 1);
      drawTo(bx + 11 * scale, by + 20 * scale);
      lift(1);
      break;
    case 7:                 //drawing 7
      drawTo(bx + 2 * scale, by + 20 * scale);
      lift(0);
      drawTo(bx + 12 * scale, by + 20 * scale);
      drawTo(bx + 2 * scale, by + 0);
      lift(1);
      break;
    case 8:                   //drawing 8
      drawTo(bx + 5 * scale, by + 10 * scale);
      lift(0);
      bogenUZS(bx + 5 * scale, by + 15 * scale, 5 * scale, 4.7, -1.6, 1);
      bogenGZS(bx + 5 * scale, by + 5 * scale, 5 * scale, -4.7, 2, 1);
      lift(1);
      break;

    case 9:                  //drawing 9
      drawTo(bx + 9 * scale, by + 11 * scale);
      lift(0);
      bogenUZS(bx + 7 * scale, by + 15 * scale, 5 * scale, 4, -0.5, 1);
      drawTo(bx + 5 * scale, by + 0);
      lift(1);
      break;

    case 111:              //erasing
      //lift(2);
      //drawTo(75, 47.5);
      lift(0);
      drawTo(65, 43);

      drawTo(65, 43);
      drawTo(5, 43);
      drawTo(5, 35);
      drawTo(65, 35);
      drawTo(65, 30);

      drawTo(5, 30);
      drawTo(5, 25);
      drawTo(65, 25);
      drawTo(65, 20);

      drawTo(5, 20);
      drawTo(5, 10);
      drawTo(65, 10);
      drawTo(65, 5);

      drawTo(5, 5);
      drawTo(60, 44);
      lift(2);
     

      break;
    case 11:                           //drawing ':'
      drawTo(bx + 5 * scale, by + 15 * scale);
      lift(0);
      bogenGZS(bx + 5 * scale, by + 15 * scale, 0.1 * scale, 1, -1, 1);
      lift(1);
      drawTo(bx + 5 * scale, by + 0 * scale);
      lift(0);
      bogenGZS(bx + 5 * scale, by + 5 * scale, 0.1 * scale, 1, -1, 1);
      lift(1);
      break;

  }
}



void lift(char lift) {
  switch (lift) {

    case 2:     //850       //Writing

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

    case 0:                        //going up towards the eraser

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

  a2 = return_angle(L2, L1, c);
  Hx = Tx + L3 * cos((a1 - a2 + 0.637) + M_PI); //36,5°
  Hy = Ty + L3 * sin((a1 - a2 + 0.637) + M_PI);
  dx = Hx - O2X;
  dy = Hy - O2Y;

  c = sqrt(dx * dx + dy * dy);
  a1 = atan2(dy, dx);
  a2 = return_angle(L1, (L2 - L3), c);
  servo3.writeMicroseconds(floor(((a1 - a2) * SERVOFAKTOR) + SERVORIGHTNULL));
}


