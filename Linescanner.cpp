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




