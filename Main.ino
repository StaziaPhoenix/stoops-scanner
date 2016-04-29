#include "Linescanner.h"
#include "Servo.h"
#include "PIDController.h"
#include "math.h"

#define END 129
#define NONE 130
#define NOLINE 131
#define center 90
#define left 150
#define right 30
#define setpoint 64
#define adj 65
#define numPixels 128
#define leftThresh 56
#define rightThresh 72

#define sync 12
#define CLK 11
#define data A0
#define motor 3
#define led 13
#define servoPin 9

byte debug = 0;
byte go = 0;
byte lineWidth;
int mdivisor = 1;

int pixels[numPixels];
byte digital[numPixels];

/* PID info */
PIDController controller;
float error;
byte prev_angle = 0;

const unsigned int expose = 7390;
Servo servo;
byte angle = 90;

//int trigger = 2;
int threshold;

// Byte (one character) read in from Virtual Serial COMM (Bluetooth)
byte inByte = 0;

// Acknowledgment variable (while-loop sentinel)
byte ack = 0;

Linescanner cam(CLK,sync,data);

void setup() {
  //attachInterrupt(digitalPinToInterrupt(trigger), killSwitch, CHANGE);
  
  pinMode(CLK,OUTPUT);
  pinMode(sync,OUTPUT);
  
  // Configure digital for OUTPUT
  pinMode(led,OUTPUT);
  digitalWrite(led,HIGH);
  
  analogWrite(motor,0);
  // Open up Bluetooth Virtual Serial COMM port
  Serial.begin(9600);

  servo.attach(servoPin);
  servo.write(angle);

  setPwmFrequency(motor, 64);
}

byte getAck() {
  Serial.write("\r\nPress <c> for command list\r\n");
    Serial.write(">");
    byte inByte = Serial.read();

    if ( inByte == 'c' ) {
      Serial.write(inByte);
      ack = 1;
      printNewLn();
      printCmdList();
      printNewCmdLn();
      return 1;
    }   
    return 0;
}

void loop() {
  while (!ack) {
    ack = getAck();    
    delay(3000);
  }

  doSerialCmd(getSerialCmd());
//  Serial.println(digitalRead(trigger));
}



// ---------------- CONTROL ---------------- //

/*
 * filters the analog data from Linescanner into digital HIGH/LOW, where
 * high indicates the line and low occurs else
 */
void filter() {
  for (int i = 0; i < numPixels; i++) {
    if (pixels[i] > threshold) {
      digital[i] = 1;
    } else {
      digital[i] = 0;
    }
  }
}

void getLine() {
  cam.scan(expose);
  cam.read(pixels);
  threshold = cam.calibrate(expose, pixels); // position 2
  filter();
  if (debug) {
    cam.printLine(digital);
  }
}

int setLineWidth() {
  getLine();

  int leftIdx = findLeftEdge(0);
  int rightIdx = findRightEdge(numPixels-1);

  return rightIdx - leftIdx;
}

void calibrate() {
  lineWidth = setLineWidth();
  go = 0;
  while (!doSerialCmd(getSerialCmd())) {
    PID();
  }
}

void run() {
  lineWidth = setLineWidth();
  go = 1;
  while(!doSerialCmd(getSerialCmd())) {
    PID();
  }
}

float calcError(int process_var) {
  int error_pix = process_var-setpoint;
  return atan2(error_pix,adj)*(180/3.141592);
}

bool checkCases( int leftIdx, int rightIdx) {
  // lost sight of the line, stay turning in previous direction
  if (leftIdx == NOLINE || (rightIdx-leftIdx) > 80) {
    if (angle > 90) {
      servo.write(left);
    } else {
      servo.write(right);
    }
    return true;
  }

  // intersection, stay true
  if ( (rightIdx-leftIdx) > (lineWidth+2) || (leftIdx == END && rightIdx == END)) {
    return true;
  }

//   staircase, turn less severely so you don't lose the line
  if (leftIdx == END) {
    servo.write(right*2);
    return true;
  } else if (rightIdx == END) {
    servo.write(left - right);
    return true;
  }

  // if none of these, adjust according to PID
  return false;
}

void adjustSpeed(int derror) {
  if ((abs(derror) < 25)) {
    analogWrite(motor,77); // 30%
  } else {
    //analogWrite(motor,51); // 20%
    analogWrite(motor,0);
  }
  
//  if ((error < 30) || (error < prev_error)) {
//    analogWrite(motor,128); // 50%
//    //analogWrite(motor,179);
//    //analogWrite(motor,153);
//  } else if (error >= 30 && error < 60) {
//    analogWrite(motor,102); // 40%
//  } else {
//    analogWrite(motor,77); // 30%
//  }
}

int findRightEdge(int startIdx) {
  if (digital[startIdx] == 1 && digital[startIdx-1]== 1) {
    return END;
  }
  for (int i = startIdx; i > 4; i--) {
    if (digital[i] == 0 && digital[i-1] == 1 && digital[i-2] == 1 && 
        digital[i-3] == 1) {
      return i-1;
    }
  }
  // if no line
  //Serial.println("Could not find right edge");
  return NOLINE;
}

int findLeftEdge(int startIdx) {
  if (digital[startIdx] == 1 && digital[startIdx+1] == 1) {
    return END;
  }
  for (int i = startIdx; i < numPixels-4; i++) {
    if (digital[i] == 0 && digital[i+1] == 1 && digital[i+2] == 1 && 
        digital[i+3] == 1) {
      return i+1;
    }
  }
  // if no line
  //Serial.println("Could not find left edge");
  return NOLINE;
}

void PID() {
  getLine();

  int leftIdx = findLeftEdge(0);
  int rightIdx = findRightEdge(numPixels-1);

  if (checkCases(leftIdx, rightIdx)) { return; }
  
  error = calcError((rightIdx-leftIdx)/2 + leftIdx);

  angle = 90 + controller.pid(error);
  if (go) { adjustSpeed(angle-prev_angle); }
  // if ( abs(angle-prev_angle) > 1 ) {
    servo.write(angle);
    //prev_angle = angle;
  //}
  prev_angle = angle;
}

// ---------------- END CONTROL ---------------- //

// ---------------- COMMANDS ---------------- //

// Prints a modified new line
void printNewLn() {
  Serial.write("\r\n");
}


// Prints a new command line cursor
void printNewCmdLn() {
  printNewLn();
  Serial.write(">");
}

void incPid(byte feed) {
  if (feed == 'p') {
    controller.incP();
    Serial.print("kP is ");
    Serial.println(controller.getP());
  } else if (feed == 'i') {
    controller.incI();
    Serial.print("kI is ");
    Serial.println(controller.getI());
  } else if (feed == 'd') {
    controller.incD();
    Serial.print("kD is ");
    Serial.println(controller.getD());
  }
}

void decPid(byte feed) {
  if (feed == 'p') {
    controller.decP();
    Serial.print("kP is ");
    Serial.println(controller.getP());
  } else if (feed == 'i') {
    controller.decI();
    Serial.print("kI is ");
    Serial.println(controller.getI());
  } else if (feed == 'd') {
    controller.decD();
    Serial.print("kD is ");
    Serial.println(controller.getD());
  }
}

void centerServo() {
  digitalWrite(led, LOW);
  delay(300);
  digitalWrite(led, HIGH);
  Serial.print("Servo alignment: CENTER "); Serial.print(center); Serial.println();
  servo.write(center);
}

void leftServo90() {
  digitalWrite(led, LOW);
  delay(300);
  digitalWrite(led, HIGH);
  Serial.print("Servo alignment: LEFT "); Serial.print(left); Serial.println();
  servo.write(left);
}

void rightServo90() {
  digitalWrite(led, LOW);
  delay(300);
  digitalWrite(led, HIGH);
  Serial.write("Servo alignment: RIGHT "); Serial.write(right); Serial.write("\n\r");
  servo.write(right);
}

// Turns LED ON and writes to Serial
void pwm100() {
  digitalWrite(led, LOW);
  delay(300);
  digitalWrite(led, HIGH);
  Serial.write("    PWM 255 (100%) motor is ON!");
  analogWrite(motor,255);
}

// Turns LED OFF and writes to Serial
void pwm90() {
  digitalWrite(led, LOW);
  delay(300);
  digitalWrite(led, HIGH);
  Serial.write("    PWM 230 (90%) motor is ON!");
  analogWrite(motor,230);
}

// Turns LED OFF and writes to Serial
void pwm80() {
  digitalWrite(led, LOW);
  delay(300);
  digitalWrite(led, HIGH);
  Serial.write("    PWM 204 (80%) motor is ON!");
  analogWrite(motor,204);
}

// Turns LED OFF and writes to Serial
void pwm70() {
  digitalWrite(led, LOW);
  delay(300);
  digitalWrite(led, HIGH);
  Serial.write("    PWM 179 (70%) motor is ON!");
  analogWrite(motor,179);
}

// Turns LED OFF and writes to Serial
void pwm60() {
  digitalWrite(led, LOW);
  delay(300);
  digitalWrite(led, HIGH);
  Serial.write("    PWM 153 (60%) motor is ON!");
}

// Turns LED OFF and writes to Serial
void pwm50() {
  digitalWrite(led, LOW);
  delay(300);
  digitalWrite(led, HIGH);
  Serial.write("    PWM 128 (50%) motor is ON!");
  analogWrite(motor,128);
}

//// Turns LED OFF and writes to Serial
void pwm40() {
  digitalWrite(led, LOW);
  delay(300);
  digitalWrite(led, HIGH);
  Serial.write("    PWM 102 (40%) motor is ON!");
  analogWrite(motor,102);
}

// Turns LED OFF and writes to Serial
void pwm30() {
  digitalWrite(led, LOW);
  delay(300);
  digitalWrite(led, HIGH);
  Serial.write("    PWM 77 (30%) motor is ON!");
  analogWrite(motor,77);
}

// Turns LED OFF and writes to Serial
void pwm20() {
  digitalWrite(led, LOW);
  delay(300);
  digitalWrite(led, HIGH);
  Serial.write("    PWM 51 (20%) motor is ON!");
  analogWrite(motor,51);
}

// Turns LED OFF and writes to Serial
void pwm10() {
  digitalWrite(led, LOW);
  delay(300);
  digitalWrite(led, HIGH);
  Serial.write("    PWM 26 (10%) motor is ON!");
  analogWrite(motor,26);
}

// Turns LED OFF and writes to Serial
void pwm0() {
  go = 0;
  digitalWrite(led, LOW);
  Serial.write("    PWM 0 (0%) motor is OFF!");
  analogWrite(motor,0);
}

void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if(pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  } else if(pin == 3 || pin == 11) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x7; break;
      default: return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
  Serial.println(divisor);
}

// Prints the command list
void printCmdList() {
  Serial.write("COMMANDS:\r\n");
  //Servo
  Serial.write("   SERVO:\r\n");
  Serial.write("      <w>    Straight dir\r\n");
  Serial.write("      <a>    Left dir\r\n");
  Serial.write("      <d>    Right dir\r\n");

  //Serial.write("\r\n");

  //motor
  Serial.write("   motor:\r\n");
  Serial.write("      <o>    PWM 255 (100%)\r\n");
//  Serial.write("      <9>    PWM 230  (90%)\r\n");
//  Serial.write("      <8>    PWM 204  (80%)\r\n");
//  Serial.write("      <7>    PWM 179  (70%)\r\n");
//  Serial.write("      <6>    PWM 153  (60%)\r\n");
//  Serial.write("      <5>    PWM 128  (50%)\r\n");
//  Serial.write("      <4>    PWM 102  (40%)\r\n");
//  Serial.write("      <3>    PWM  77  (30%)\r\n");
//  Serial.write("      <2>    PWM  51  (20%)\r\n");
//  Serial.write("      <1>    PWM  26  (10%)\r\n");  
//  Serial.write("      <0>    PWM   0   (0%)\r\n");  
  
  Serial.write("      <c>    Command List\r\n"); 
  Serial.write("      <s>    run\r\n");
  Serial.write("      <b>    Calibrate\r\n");
  
  Serial.write("      <p>    Increase p by 10%\r\n");
  Serial.write("      <l>    Decrease p by 10%\r\n");
  Serial.write("      <m>    Inc d by 10%\r\n");
  Serial.write("      <n>    Dec d by 10%\r\n");
//  Serial.write("      <k>    Inc i by 10%\r\n");
//  Serial.write("      <j>    Dec i by 10%\r\n");
  Serial.write("      <h>    inc motor PWM frequency\r\n");
  Serial.write("      <g>    Toggle debug statements\r\n");

  Serial.write("\n");
}

// Performs serial command based on user input
bool doSerialCmd( byte cmd ) {
  switch( cmd ) {
    // Turn LED HIGH
    case('h'):
      setPwmFrequency(motor, (mdivisor = mdivisor*2));
      printNewCmdLn();
      break;
    case ('w'):
      centerServo();
      printNewCmdLn();
      break;
    case ('d'):
      rightServo90();
      printNewCmdLn();
      break;
    case ('a'):
      leftServo90();
      printNewCmdLn();
      break;
    case ('1'):
      pwm10();
      printNewCmdLn();
      break;
    // Turn LED LOW
    case ('2'):
      pwm20();
      printNewCmdLn();
      break;
    // Turn LED LOW
    case ('3'):
      pwm30();
      printNewCmdLn();
      break;
    // Turn LED LOW
    case ('4'):
      pwm40();
      printNewCmdLn();
      break;
    // Turn LED LOW
    case ('5'):
      pwm50();
      printNewCmdLn();
      break;
    // Turn LED LOW
    case ('6'):
      pwm60();
      printNewCmdLn();
      break;
    // Turn LED LOW
    case ('7'):
      pwm70();
      printNewCmdLn();
      break;
    // Turn LED LOW
    case ('8'):
      pwm80();
      printNewCmdLn();
      break;
    // Turn LED LOW
    case ('9'):
      pwm90();
      printNewCmdLn();
      break;      
    case ('o'):
      pwm100();
      printNewCmdLn();
      break;      
    case ('0'):
      pwm0();
      printNewCmdLn();
      break;
    // Display CMD List
    case ('c'):
      printCmdList();
      printNewCmdLn();
      break;
    case ('s'):
      run();
      printNewCmdLn();
      break;
    case ('b'):
      calibrate();
      printNewCmdLn();
      break;
    case ('p'):
      incPid('p');
      printNewCmdLn();
      break;
    case ('l'):
      decPid('p');
      printNewCmdLn();
      break;
    case ('m'):
      incPid('d');
      printNewCmdLn();
      break;
    case ('n'):
      decPid('d');
      printNewCmdLn();
      break;
//    case ('k'):
//      incPid('i');
//      printNewCmdLn();
//      break;
//    case ('j'):
//      decPid('i');
//      printNewCmdLn();
//      break;
    case ('e'):
      return true;
    case ('g'):
      debug = debug^1;
      Serial.print("Debug is ");
      Serial.println(debug);
      printNewCmdLn();
    case (NONE):
      return false;
  }
  return false;
}

// Prompts User for input serial command
//    Returns serial command
byte getSerialCmd() {
  byte inByte;
  if (Serial.available()) {
    inByte = Serial.read();
    Serial.write(inByte);
    printNewLn();
    return inByte;
  } else {
    return NONE;
  }
}

