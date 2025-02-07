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

struct DiffV
{
    float omega; // omega of bot
    float VT;    // total velocity of bot
    float VL;    // velocity of left wheel
    float VR;    // velocity of right wheel
};

float RM = 15;
float VT = 100;

// bots velocities and omega
struct DiffV botV = {0.0, 50.0, 0.0, 0.0};

// calculate differential VR and VL for left and right wheels
struct DiffV calculate_diffVelocities(struct DiffV resV)
{
    // Tmatrix
    float a, b, c, d; // elements of Tmatrix
    a = c = 0.5;
    b = RM / 2.0;
    d = -b;

    // Calculation with pre-evaluated fromulae
    resV.VL = (resV.VT * a) + (b * resV.omega);
    resV.VR = (resV.VT * c) + (d * resV.omega);

    printf("Vt = %f, omega = %f, rm = %.2f => VL = %.2f, VR = %.2f\n", resV.VT, resV.omega, RM, resV.VL, resV.VR);
    return resV;
}

int main()
{
    printf("hello world");
    calculate_diffVelocities(botV);

    return 0;
}

// Function to check the calculate_diffVelocities function

void test_calculate_diffVelocities(){
    struct DiffV testInput;
    struct DiffV expectedOutput;
    struct DiffV actualOutput;

    // Test case 1
    testInput = (struct DiffV){1.0, 100.0, 0.0, 0.0}; // omega = 1.0, VT = 100.0
    expectedOutput = (struct DiffV){1.0, 100.0, 57.50, 42.50}; // Expected VL and VR
    actualOutput = calculate_diffVelocities(testInput);
    // Check results

    if (fabs(actualOutput.VL - expectedOutput.VL) < 0.01 && fabs(actualOutput.VR - expectedOutput.VR) < 0.01)
    {

        printf("Test Case 1 Passed!\n");
    }else    {

        printf("Test Case 1 Failed! Expected VL = %.2f, VR = %.2f; Got VL = %.2f, VR = %.2f\n",

               expectedOutput.VL, expectedOutput.VR, actualOutput.VL, actualOutput.VR);
    }
    // Additional test cases can be added here
}
