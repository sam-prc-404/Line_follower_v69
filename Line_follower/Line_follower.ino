#include <SoftwareSerial.h>
#include <PID_v1.h>
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
#define RX 4
#define TX 5
#define JPULSE 6

bool NO_LINE = false;

typedef struct DiffV {
  float omega;  // omega of bot
  float VT;     // total velocity of bot
  float wL;     // angular velocity of left wheel
  float wR;     // angular velocity of right wheel
};

// variables //
volatile unsigned long pulseCountMLeft = 0;   // Count of encoder pulses for Motor 1
volatile unsigned long pulseCountMRight = 0;  // Count of encoder pulses for Motor 2
const int encoderPinLeft = 2;                 // Pin connected to the encoder output for Motor 1
const int encoderPinRight = 3;                // Pin connected to the encoder output for Motor 2
const int pulsesPerRevolution = 1400;         // Pulses per revolution for the encoder
unsigned long lastTime = 0;                   // Last time we calculated RPM
float rpmMLeft = 0.0;                         // Variable to store RPM for Motor 1
float rpmMRight = 0.0;                        // Variable to store RPM for Motor 2

// Bot's dimensions ande other parms //
double B_len = 0.1;
double r_wheel = 0.02;
double indiv_ir_dist = 0.016;
double VT = 1.5;
double RM = 0.095;
double l_dist = RM / 2;
// bots velocities and omega
DiffV botV = { 0.0, VT, 0.0, 0.0 };

///////////////////////////////////////

// PID setup and paarams configuration //
double Input, Output;
double Setpoint = 0.0;
double Kp = 1.5;
double Ki = 0.0;
double Kd = 0.0;
PID pid(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
//////////////////////////////////////////

SoftwareSerial lsaSerial(RX, TX);



/////////////// ENTRY POINT ////////////////
void setup() {
  // Set all the motor control pins to outputs
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(TX, INPUT);
  pinMode(JPULSE, INPUT);
  // pinMode(encoderPinLeft, INPUT_PULLUP);   // Set encoder pin for Motor 1 as input with pull-up resistor
  // pinMode(encoderPinRight, INPUT_PULLUP);  // Set encoder pin for Motor 2 as input with pull-up resistor

  // Turn on motors - Initial state
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);

  Serial.begin(9600);
  delay(100);
  lsaSerial.begin(9600);  //baudrate = 4 on lsa08
  while (!lsaSerial) {
    ;
  }

  delay(3000);
  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(-90.0, 90.0);
}

void move();
void stop();
int prev_linepos = 35;
double theta = 0.0;
double prev_theta = 0.0;
double linepos = 35;
int prev_movement = 0;  // 0 = straight , -1  = left and 1 = right
uint8_t junction_data = 0;
int junc = 0;

void loop() {
  junc = digitalRead(JPULSE);
  Serial.print("J:" + String(junc) + " ");

  linepos = get_linepos();

  // JUNCTION DETECTED //
  if (junc) {
    float lp = get_linepos();
    // stop(2000);
    while (0.5 <= lp && lp >= 0.5) {
      Serial.print("Jdata:");
      Serial.print(junction_data, BIN);

      // turn into juntion until center of line found
      lp = get_linepos();
      if (weight(junction_data) > 0) {  //left
        botV.wL = 100;
        botV.wR = -100;
      } else {  //right
        botV.wL = -100;
        botV.wR = 100;
      }
      move();
    }
  }

  Serial.print("pp:" + String(prev_movement) + " ");
  // NO LINE DETECTED //
  if (NO_LINE) {
    while (NO_LINE) {
      // rotate right if previously line on left //
      linepos = get_linepos();

      switch (prev_movement) {
        case 0:  // previously straight
          botV.wL = -100;
          botV.wR = -100;
          break;
        case 1:  //previously right
          botV.wL = -100;
          botV.wR = 100;
          break;
        case -1:  //previously left
          botV.wL = 100;
          botV.wR = -100;
          break;
      }
      move();
      Serial.println();
    }
  }

  // BANG BANG //
  // line on right turn left
  if (linepos >= -3.5 && linepos <= -0.5) {
    prev_movement = 1;
    botV.wR = 100;
    botV.wL = 0;
    move();
  }  // line on left turn right
  else if (linepos >= 0.5 && linepos <= 3.5) {
    prev_movement = -1;
    botV.wR = 0;
    botV.wL = 100;
    move();
  } else {  // go straight
    prev_movement = 0;
    botV.wL = 100;
    botV.wR = 100;
    move();
  }

  prev_linepos = linepos;

  delay(100);
  Serial.println();
}


void move() {
  //FIXME flipped values FTM
  int VL = botV.wL;
  int VR = botV.wR;

  // move left motor
  if (VL > 0) {
    // move in forward dir
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else {
    Serial.print(" rVL");
    // move in backward dir
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  // move right motor
  if (VR > 0) {
    // move in forward dir
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
  } else {
    Serial.print(" rVR");
    // move in backward dir
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
  }

  analogWrite(enB, fabs(VR));
  analogWrite(enA, fabs(VL));
}


void junction_here(int junc) {
  // junction detected
  if (junc) {
    stop(10);
    // turn in the direction of junction until linepos is 35
    bool in_jturn = true;
    int lp = prev_linepos;
    int Vturn = 70;
    while (in_jturn) {
      while (lsaSerial.available()) {  // Wait for data to be available
        lp = lsaSerial.read();
      }
      Serial.println("Lp : " + String(lp));
      if (lp <= 30) {
        // turn left
        analogWrite(enA, fabs(Vturn));
      } else if (lp >= 30) {
        // turn left
        analogWrite(enB, fabs(Vturn));
      } else {
        stop(100);
        break;
        in_jturn = false;
      }
    };
  }
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

float weight(uint8_t inp) {
  float retval = 0;
  for (int i = 0; i < 8; i++) {
    int thing = ((inp & (1 << i)) >> i);
    retval += thing * (3.5 - i);
  }

  int ones = 0;
  for (int i = 0; i < 8; i++)
    if (inp & (1 << i)) ones++;

  if (ones)
    return retval / ones;
  else return -100;
}


float get_linepos() {
  // // // get lsa08 line data from UART
  uint8_t rawsensorData;

  while (lsaSerial.available() > 0) {  // Wait for data to be available
    rawsensorData = lsaSerial.read();
  }
  Serial.print("DATA: ");
  Serial.print(rawsensorData, BIN);
  Serial.print(" ");
  Serial.print(rawsensorData);

  // save junction data if on juntion
  if (junc) {
    junction_data = rawsensorData;
  }

  float sum = weight(rawsensorData);
  Serial.print(" wght:" + String(sum) + " ");
  linepos = sum;

  Serial.print("lpos:" + String(linepos));

  if (sum == -100.00) {
    theta = 0;
    linepos = 0;
    NO_LINE = true;
  } else {
    NO_LINE = false;
  }


  return linepos;
}
