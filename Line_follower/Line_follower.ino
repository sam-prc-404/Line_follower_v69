#include <SoftwareSerial.h>
#include <PID_v1.h>
#include "Line_follower_funcs.h"

// Motor A connections
#define enA 11
#define in1 12
#define in2 13
// Motor B connections
#define enB 10
#define in3 9
#define in4 8
// Data pins
#define RX 2
#define TX 3
#define UEN 4
#define JPULSE 5
#define AN A5


double Input, Output;
double Setpoint = 35.0;
double Kp = 10.0;
double Ki = 0.0;
double Kd = 0.0;
PID pid(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);


// #define btRX 2
// #define btTX 3
// SoftwareSerial btSerial(btTX, btRX);
SoftwareSerial lsaSerial(RX, TX);

// struct DiffV{
//   float omega; //omega of bot
//   float VT; // total velocity of bot
//   float VL; // velocity of left wheel
//   float VR; // velocity of right wheel
// };

// float VL = 0;
// float VR = 0;
// float omega = 0.0;
// int RM = 15;
// float VT = 100;

// DiffV botV = {0.0,50.0,0.0,0.0};
// // calculate differential VR and VL for left and right wheels
// void calculate_diffVelocities(DIffV resV) {
//   // Tmatrix
//   float a, b, c, d;  // elements of Tmatrix
//   a = c = 0.5;
//   b = rm / 2.0;
//   d = -b;

//   // Calculation with pre-evaluated fromulae
//   float resV.VL = (res.Vt * a) + (b * res.omega);
//   float resV.VR = (rsw.Vt * c) + (d * res.omega);
  

//   // printf("Vt = %d, omega = %d, rm = %.2f => VL = %.2f, VR = %.2f\n", Vt, omega, rm, VL, VR);
//   return resV;
// }


void setup() {
  // Set all the motor control pins to outputs
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(UEN, OUTPUT);
  pinMode(JPULSE, INPUT);
  pinMode(AN, INPUT);

  // initialize Data pins
  digitalWrite(UEN, LOW);
  // Turn on motors - Initial state
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);


  Serial.begin(9600);
  // btSerial.begin(9600);
  delay(100);
  lsaSerial.begin(9600);
  while (!lsaSerial) {
    ;
  }

  delay(3000);
  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(-100.0, 100.0);

}


void move();
void stop();
int prev_diff = 0;
int prev_linepos = 35;

void loop() {
  int junc = digitalRead(JPULSE);
  Serial.print("Jpulse: " + String(junc) + " ");

  // get lsa08 line data from UART
  int linepos = 35;
  while (lsaSerial.available()) {  // Wait for data to be available
    linepos = lsaSerial.read();
  };
  Serial.println("Linepos : " + String(linepos));


  // when no line detected
  if (linepos > 70 || linepos == 255) {
    Input = prev_linepos;
    // no_line_panic();
  } else {
    Input = linepos;
    prev_linepos = linepos;
  }

  // compute the PID
  pid.Compute();

  // move the motors with PID computed output
  Serial.println("INPUT: " + String(Input) + "OUTPUT :" + String(Output));

  int spd_mul = 1;
  calculate_diffVelocities();
  move();
  // btSerial.println("Input: "+ String(Input) + " Ouput: " + String(Output));
  // delay(100);
}

void move(int diff_spd) {
  // Turn on motors - Initial state
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  int default_spd = 50;
  // calculatte diff based speeds
  int spdA = (default_spd + diff_spd);
  int spdB = (default_spd - diff_spd);

  // Clamp speeds to be within 0-255
  spdA = constrain(spdA, 0, 255);
  spdB = constrain(spdB, 0, 255);

  analogWrite(enA, spdA);
  analogWrite(enB, spdB);
}






void stop(unsigned long time) {
  digitalWrite(in1, 0);
  digitalWrite(in2, 0);
  digitalWrite(in3, 0);
  digitalWrite(in4, 0);
  analogWrite(enA, 0);
  analogWrite(enB, 0);
  delay(time);
}
