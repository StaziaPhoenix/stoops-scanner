#include "Linescanner.h"
#include "Servo.h"
#include "PIDController.h"
#include "math.h"

#define ERR 129

int sync = 12;
int CLK = 11;
int data = A0;

const unsigned int numPixels = 128;
int pixels[numPixels];
int digital[numPixels];

/* PID info */
PIDController controller;
int setpoint;
int process_var;
float error;
float dt;
float prev_millis;
const int adj = 65;
int temp;

//unsigned long utime;
//int itime;

const unsigned int expose = 7390;
int angle = 90;
Servo servo;

Linescanner cam(CLK,sync,data);

void setup() {
  // put your setup code here, to run once:
  pinMode(CLK,OUTPUT);
  pinMode(sync,OUTPUT);
  Serial.begin(9600);

  servo.attach(6);
  servo.write(angle);

  // get line and filter into binary
  cam.scan(expose);
  cam.read(pixels);
  filter();

  // find left and right edges, use them to make setpoint
  int leftIdx = findEdge(0);
  int rightIdx = findEdge(numPixels-1);
  setpoint = (rightIdx-leftIdx)/2 + leftIdx;
  Serial.println(setpoint);
}

void loop() {
  // put your main code here, to run repeatedly:
  cam.scan(expose);
  cam.read(pixels);
  filter();

  int leftIdx = findEdge(0);
  int rightIdx = findEdge(numPixels-1);
  process_var = (rightIdx-leftIdx)/2 + leftIdx;

  int error_pix = process_var-setpoint;
  error = atan2(error_pix,adj)*(180/3.141592);
  dt = (temp = millis())-prev_millis;
  prev_millis = temp;

  Serial.println(error_pix);
//  Serial.println(error);
  Serial.println(90 + controller.pid(error, dt));
  Serial.println("--------");
  //printLine(pixels);
  //printLine(digital);
  delay(50);
  if (leftIdx == ERR || rightIdx == ERR) {
    return;
  }
  servo.write(90 + controller.pid(error, dt));
}


/* --------------------------------
 * HELPER FUNCTIONS 
 * ------------------------------- */

void printLine(int array[]) {
  for (int i=0;i<numPixels;i++) {
    Serial.print(array[i]);
    Serial.print(" ");
    delay(1);
  }
  //delay(1000);
  Serial.println();
}

/*
 * filters the analog data from Linescanner into digital HIGH/LOW, where
 * high indicates the line and low occurs else
 */
void filter() {
  static int threshold = setThreshold();

  for (int i = 0; i < numPixels; i++) {
    if (pixels[i] > threshold) {
      digital[i] = 1;
    } else {
      digital[i] = 0;
    }
  }

}

/* 
 * Sets the cutoff between high and low values. Make dynamic, but for now a constant
 */
int setThreshold() {
  return 100;
}


int findEdge(int startIdx) {
  static const int halfLine = numPixels/2;

  if (startIdx < halfLine) {
    for (int i = startIdx; i < numPixels-4; i++) {
      if (digital[i] == 0 && digital[i+1] == 1 && digital[i+2] == 1 && digital[i+3] == 1) {
        return i+1;
      }
    }
    Serial.println("Could not find left edge");
    return ERR;
  }

  if (startIdx > halfLine) {
    for (int i = startIdx; i > 4; i--) {
      if (digital[i] == 0 && digital[i-1] == 1 && digital[i-2] == 1 && digital[i-3] == 1) {
        return i-1;
      }
    }
    Serial.println("Could not find right edge");
    return ERR;
  }
}

