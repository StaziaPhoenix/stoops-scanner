#include "Arduino.h"

class Linescanner {
    int CLK;
    int sync;
    int data;
  public:
//    Linescanner();
    
    Linescanner(int CLK,int sync,int data) : CLK(CLK), sync(sync), data(data) {}
    
    void scan(int expose);
    void read(int* pixels);
    int calibrate(int expose, int* pixels);
    void printLine(int* array);
  private:
    int findMax(const int* pixels);
    float average(int lowIdx, int highIdx, const int* pixels);
    float average(int low_lowIdx, int low_highIdx, 
              int high_lowIdx, int high_highIdx, const int* pixels);
};
//  Linescanner::Linescanner() {
//    
//  }
  



