#include "Linescanner.h"
#include "Servo.h"
#include "PIDController.h"
#include "math.h"

#define END 129
#define NONE 130
#define NOLINE 131
#define sync 12
#define CLK 11
#define data A0
#define motor 3
#define led 13
#define center 90
#define left 120
#define right 60
#define servoPin 9
#define setpoint 64
#define adj 65
#define numPixels 128

byte debug = 0;

// const unsigned int numPixels = 128;
int pixels[numPixels];
byte digital[numPixels];

/* PID info */
PIDController controller;
//int setpoint = 64;
int process_var;
float error;
byte prev_angle = 0;

//unsigned long utime;
//int itime;

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
}

void loop() {
  while (!ack) {
    ack = getAck();    
    delay(3000);
  }

  doSerialCmd(getSerialCmd());
//  Serial.println(digitalRead(trigger));
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

void calibrate() {
  while (1) {
    doSerialCmd(getSerialCmd());
    PID();
  }
}

void run() {
  pwm100();
  delay(500);
  pwm80();
  while(!doSerialCmd(getSerialCmd())) {
    
    PID();
  }
}

float calcError(int process_var) {
  int error_pix = process_var-setpoint;
  return atan2(error_pix,adj)*(180/3.141592);
}

bool checkCases( int leftIdx, int rightIdx) {
  if (leftIdx == END && rightIdx == END) {
    return true;
  }
  if (leftIdx == END) {
    servo.write(right);
    return true;
  } else if (rightIdx == END) {
    servo.write(left);
    return true;
  }
  if (leftIdx == NOLINE || (rightIdx-leftIdx) > 15) {
    if (angle > 90) {
      servo.write(left);
    } else {
      servo.write(right);
    }
    return true;
  }
  return false;
}

void adjustSpeed(float error) {
  if (error < 30) {
    pwm100();
  } else if (error >= 30 && error < 60) {
    pwm80();
  } else {
    pwm60();
  }
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
  cam.scan(expose);
  cam.read(pixels);
  threshold = cam.calibrate(expose, pixels); // position 2
  filter();
  if (debug) {
    cam.printLine(digital);
  }

  int leftIdx = findLeftEdge(0);
  int rightIdx = findRightEdge(numPixels-1);

  if (checkCases(leftIdx, rightIdx)) { return; }
  
  error = calcError((rightIdx-leftIdx)/2 + leftIdx);
  adjustSpeed(error);

  angle = 90 + controller.pid(error);
  if ( abs(angle-prev_angle) > 1 ) {
    servo.write(angle);
    prev_angle = angle;
  }
}

// ---------------- END CONTROL ---------------- //

// ---------------- COMMANDS ---------------- //

// Prints a new command line cursor
void printNewCmdLn() {
  printNewLn();
  Serial.write(">");
}

// Prints a modified new line
void printNewLn() {
  Serial.write("\r\n");
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
  analogWrite(motor,153);
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
  digitalWrite(led, LOW);
  Serial.write("    PWM 0 (0%) motor is OFF!");
  analogWrite(motor,0);
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
  Serial.write("      <k>    Inc i by 10%\r\n");
  Serial.write("      <j>    Dec i by 10%\r\n");
  Serial.write("      <g>    Toggle debug statements\r\n");

  Serial.write("\n");
}

// Performs serial command based on user input
bool doSerialCmd( byte cmd ) {
  switch( cmd ) {
    // Turn LED HIGH
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
    case ('k'):
      incPid('i');
      printNewCmdLn();
      break;
    case ('j'):
      decPid('i');
      printNewCmdLn();
      break;
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

