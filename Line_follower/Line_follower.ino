#include <SoftwareSerial.h>
#include <PID_v1.h>
// #include "Line_follower_funcs.h"
#include <math.h>

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

typedef struct DiffV
{
    float omega; // omega of bot
    float VT;    // total velocity of bot
    float VL;    // velocity of left wheel
    float VR;    // velocity of right wheel
};

double Input, Output;
double Setpoint = 35.0;
double Kp = 2.0;
double Ki = 0.0;
double Kd = 0.0;
PID pid(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

SoftwareSerial lsaSerial(RX, TX);

float VT = 100.0;
float RM = 20.0;
// bots velocities and omega
DiffV botV = {0.0, VT, 0.0, 0.0};


// Calculate differential VR and VL for left and right wheels
void calculate_diffVelocities(DiffV &resV, float rm)
{   
  
    // resV {resV.omega,resV.VT,0,0}; 
    float a, b, c, d;
    a = c = 0.5;
    b = rm / 2.0;
    d = -b;

    // Calculation with pre-evaluated formulae
    resV.VL = (resV.VT * a) + (b * resV.omega); // Left wheel velocity
    resV.VR = (resV.VT * c) + (d * resV.omega); // Right wheel velocity

    // printf("Vt = %f, omega = %f, rm = %.2f => VL = %.2f, VR = %.2f\n", resV.VT, resV.omega, rm, resV.VL, resV.VR);
    // botV = resV;
    // return resV;
}


void setup()
{
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
  while (!lsaSerial)
  {
    ;
  }

  delay(3000);
  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(-255.0, 255.0);
}

void move();
void stop();
int prev_diff = 0;
int prev_linepos = 35;

void loop()
{
  int junc = digitalRead(JPULSE);
  Serial.print("Jpulse: " + String(junc) + " ");

  // get lsa08 line data from UART
  int linepos = 35;
  while (lsaSerial.available())
  { // Wait for data to be available
    linepos = lsaSerial.read();
  };
  Serial.print("Linepos : " + String(linepos)+" ");

  // when no line detected
  if (linepos > 70 || linepos == 255)
  {
    Input = prev_linepos;
    VT = 0;
  }
  else
  {
    Input = linepos;
    prev_linepos = linepos;
    VT = 100.0;
  }

  // botV.omega = linepos - 35;
  botV.omega = 35 - linepos;
  // compute the PID
  pid.Compute();

  // move the motors with PID computed output
  Serial.print(" INPUT: " + String(Input) + " OUTPUT :" + String(Output));

  int spd_mul = 1;
  calculate_diffVelocities(botV, RM);
  move();
  // btSerial.println("Input: "+ String(Input) + " Ouput: " + String(Output));
  // delay(100);
}

void move()
{
  int VL = botV.VL;
  int VR = botV.VR;

    // Clamp speeds to be within -255 and 255
  VL = constrain(VL+50,1, 255);
  VR = constrain(VR+50,1, 255);

  Serial.println(" VL:"+String(VL)+" VR:"+String(VR)+" w:"+String(botV.omega));

  // move left motor
  if (VL >= 0  )
  {
    // move in forward dir
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  if ( VL < -25)
  { 
    Serial.print(" reverse VL");
    // move in backward dir
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  analogWrite(enA,fabs(VL));


  // move right motor
  if (VR >= 0 )
  { 
    Serial.print(" reverse VR");
    // move in forward dir
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
  }
  if (VR < -25)
  {
    // move in backward dir
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
  }
  analogWrite(enB,fabs(VR));

}



void stop(unsigned long time)
{
  digitalWrite(in1, 0);
  digitalWrite(in2, 0);
  digitalWrite(in3, 0);
  digitalWrite(in4, 0);
  analogWrite(enA, 0);
  analogWrite(enB, 0);
  delay(time);
}
