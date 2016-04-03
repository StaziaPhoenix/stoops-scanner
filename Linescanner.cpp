#include "Linescanner.h"

void Linescanner::scan(int expose) {
  //Initialize scan (SI)
  digitalWrite(CLK,LOW);
  digitalWrite(sync,HIGH);
  digitalWrite(CLK,HIGH);
  digitalWrite(sync,LOW);
  digitalWrite(CLK,LOW);

  //CLK all pixels
  for (int i=0;i<128;i++) {
    digitalWrite(CLK,HIGH);
    digitalWrite(CLK,LOW);
  }

  //Pixel charge transfer time (t_qt)
  delayMicroseconds(expose);

  digitalWrite(sync,HIGH);
  digitalWrite(CLK,HIGH);
  digitalWrite(sync,LOW);
  digitalWrite(CLK,LOW);
}

void Linescanner::read(int* pixels) {
  //Reading data from cam
  for (int i=0;i<128;i++) {
    pixels[i] = analogRead(data);
    
    digitalWrite(CLK,HIGH);
    digitalWrite(CLK,LOW);
  }
}

int Linescanner::calibrate(int expose, int* pixels) {
  scan(expose);
  read(pixels);
  scan(expose);
  read(pixels);
  scan(expose);
  read(pixels);

  byte maxIdx = findMax(pixels);
  float high_avg = average(maxIdx-5,maxIdx+5, pixels);
  float low_avg = average(0, 5, 123, 128, pixels);

  return (high_avg-low_avg)/2 + low_avg;
}

byte Linescanner::findMax(const int* pixels) {
  int maxim = 0;
  byte idx;
  for (int i = 0; i < 128; i++) {
    if (pixels[i] > maxim) {
      maxim = pixels[i];
      idx = i;
    }
  }
  return idx;
}

float Linescanner::average(int lowIdx, int highIdx, const int* pixels) {
  int sum = 0;
  for (int i = lowIdx; i < highIdx+1; i++) {
    sum += pixels[i];
  }
  return sum/11;
}

float Linescanner::average(int low_lowIdx, int low_highIdx, 
              int high_lowIdx, int high_highIdx, const int* pixels) {
  int sum = 0;
  for (int i = low_lowIdx; i < low_highIdx; i++) {
    sum += pixels[i];
  }
  for (int i = high_lowIdx; i < high_highIdx; i++) {
    sum+= pixels[i];
  }
  return sum/10;
}

void Linescanner::printLine(int array[]) {
  if (array[64] > 1) {
    for (int i=0;i<128;i++) {
      Serial.print(array[i]);
      Serial.print(" ");
    }
  } else {
    for (int i=0;i<128;i++) {
      Serial.print(array[i]);
    }
  }
  
  Serial.println();
}
