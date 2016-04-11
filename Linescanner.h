#include "Arduino.h"

class Linescanner {
    byte CLK;
    byte sync;
    byte data;
  public:
//    Linescanner();
    
    Linescanner(int CLK,int sync,int data) : CLK(CLK), sync(sync), data(data) {}
    
    void scan(int expose);
    void read(int* pixels);
    int calibrate(int expose, int* pixels);
    void printLine(int* array);
    void printLine(byte* array);
  private:
    byte findMax(const int* pixels);
    float average(int lowIdx, int highIdx, const int* pixels);
    float average(int low_lowIdx, int low_highIdx, 
              int high_lowIdx, int high_highIdx, const int* pixels);
};
//  Linescanner::Linescanner() {
//    
//  }
  



