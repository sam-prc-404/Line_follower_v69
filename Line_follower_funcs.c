#include <stdio.h>

// Left Side Motor
// Motor A connections
#define enA 3
#define in1 2
#define in2 4

// Right Side Motor
// Motor B connections
#define enB 6
#define in3 5
#define in4 7

// IR module array pins
// leftmost_ir == ir_PINS[0] == 8
const int ir_PINS[5] = {8, 9, 10, 11, 12}; // Digital pins on Arduino Nano

#define is_LINE 0
#define not_LINE 1

float VL = 0;
float VR = 0;
float omega = 0.0;
int RM = 15;
float VT = 100;

// calculate differential VR and VL for left and right wheels
void calculate_diffVelocities(float Vt, float omega, float rm)
{
    // Tmatrix
    float a, b, c, d; // elements of Tmatrix
    a = c = 0.5;
    b = rm / 2.0;
    d = -b;

    // Calculation with pre-evaluated fromulae
    float VL = (Vt * a) + (b * omega);
    float VR = (Vt * c) + (d * omega);

    printf("Vt = %d, omega = %d, rm = %.2f => VL = %.2f, VR = %.2f\n", Vt, omega, rm, VL, VR);
}

int main()
{
    printf("hello world");
    calculate_diffVelocities(VT, 10.0, RM);

    return 0;
}
