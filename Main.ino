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
Servo servo;

int motor = 9;
int led = 13;
int debug = 1;
int center = 90;
int left = 120;
int right = 60;
int servoPin = 6;
int trigger = 8;

// Byte (one character) read in from Virtual Serial COMM (Bluetooth)
byte inByte = 0;

// Acknowledgment variable (while-loop sentinel)
byte ack = 0;

Linescanner cam(CLK,sync,data);

void setup() {
  attachInterrupt(digitalPinToInterrupt(trigger), killSwitch, CHANGE);
  
  pinMode(CLK,OUTPUT);
  pinMode(sync,OUTPUT);
  
  // Configure digital for OUTPUT
  pinMode(led,OUTPUT);
  digitalWrite(led,HIGH);
  
  analogWrite(motor,0);
  // Open up Bluetooth Virtual Serial COMM port
  Serial.begin(9600);

  servo.attach(servoPin);
  servo.write(90);

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
  while (!ack) {
    ack = getAck();    
    delay(3000);
  }

  doSerialCmd(getSerialCmd());
//  Serial.println(digitalRead(trigger));
}


/* --------------------------------
 * HELPER FUNCTIONS 
 * ------------------------------- */

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


int findEdge(int startIdx) {
  static const int halfLine = numPixels/2;

  if (startIdx < halfLine) {
    for (int i = startIdx; i < numPixels-5; i++) {
      if (digital[i] == 0 && digital[i+1] == 1 && digital[i+2] == 1 && 
          digital[i+3] == 1 && digital[i+4] == 1) {
        return i+1;
      }
    }
    Serial.println("Could not find left edge");
    return ERR;
  }

  if (startIdx > halfLine) {
    for (int i = startIdx; i > 5; i--) {
      if (digital[i] == 0 && digital[i-1] == 1 && digital[i-2] == 1 && 
          digital[i-3] == 1 && digital[i-4] == 1) {
        return i-1;
      }
    }
    Serial.println("Could not find right edge");
    return ERR;
  }
}



// Performs serial command based on user input
void doSerialCmd( byte cmd ) {
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
      PID();
      printNewCmdLn();
      break;
  }
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

// Turns LED OFF and writes to Serial
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
  Serial.write("      <9>    PWM 230  (90%)\r\n");
  Serial.write("      <8>    PWM 204  (80%)\r\n");
  Serial.write("      <7>    PWM 179  (70%)\r\n");
  Serial.write("      <6>    PWM 153  (60%)\r\n");
  Serial.write("      <5>    PWM 128  (50%)\r\n");
  Serial.write("      <4>    PWM 102  (40%)\r\n");
  Serial.write("      <3>    PWM  77  (30%)\r\n");
  Serial.write("      <2>    PWM  51  (20%)\r\n");
  Serial.write("      <1>    PWM  26  (10%)\r\n");  
  Serial.write("      <0>    PWM   0   (0%)\r\n");  
  
  Serial.write("      <c>    Command List\r\n"); 
  Serial.write("      <s>    PID");

  Serial.write("\n");
}



void PID() {
  pwm100();
  while(1) {
    doSerialCmd(getSerialCmd());
    
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
  
    if (debug) {
      Serial.println(error_pix);
      Serial.println(90 + controller.pid(error, dt));
      Serial.println("--------");
      //cam.printLine(pixels);
      //cam.printLine(digital);
    }
    if (leftIdx == ERR || rightIdx == ERR) {
      analogWrite(motor,0);
    } else {
      servo.write(90 + controller.pid(error, dt));
    }
  }
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

// Prints a new command line cursor
void printNewCmdLn() {
  printNewLn();
  Serial.write(">");
}

// Prints a modified new line
void printNewLn() {
  Serial.write("\r\n");
}

void killSwitch() {
  analogWrite(motor, 0);
//  analogWrite(pwmB, 0);
  digitalWrite(led, LOW);
}

/* 
 * Sets the cutoff between high and low values. Make dynamic, but for now a constant
 */
int setThreshold() {
  return 70;
}

//void printLine(int array[]) {
//  for (int i=0;i<numPixels;i++) {
//    Serial.print(array[i]);
//    Serial.print(" ");
//    delay(1);
//  }
//  //delay(1000);
//  Serial.println();
//}
