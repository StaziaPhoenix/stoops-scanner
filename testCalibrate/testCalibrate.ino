#include "Linescanner.h"

int sync = 12;
int CLK = 11;
int data = A0;

const unsigned int expose = 7390;
int threshold;

const unsigned int numPixels = 128;
int pixels[numPixels];
int digital[numPixels];

Linescanner cam(CLK,sync,data);

void setup() {
  pinMode(CLK,OUTPUT);
  pinMode(sync,OUTPUT);

  Serial.begin(9600);

  threshold = cam.calibrate(expose,pixels);
  Serial.println("calibrate set threshold to ");
  Serial.println(threshold);
  Serial.println("The line viewed from calbriate is: ");
}

void loop() {
  cam.scan(expose);
  cam.read(pixels);
  filter();

  cam.printLine(digital);
}

/*
 * filters the analog data from Linescanner into digital HIGH/LOW, where
 * high indicates the line and low occurs else
 */
void filter() {
  //static int threshold = setThreshold();

  for (int i = 0; i < numPixels; i++) {
    if (pixels[i] > threshold) {
      digital[i] = 1;
    } else {
      digital[i] = 0;
    }
  }

}

