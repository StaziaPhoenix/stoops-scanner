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
};
//  Linescanner::Linescanner() {
//    
//  }
  



