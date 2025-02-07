// Line_follower_funcs.c
#include <stdio.h>
#include <math.h>
#include "Line_follower_funcs.h"

float RM = 15;   // Robot radius
int VT = 100;    // Translational velocity

// Bot's velocities and omega
struct DiffV botV = {0.0, 50.0, 0.0, 0.0};

// Calculate differential VR and VL for left and right wheels
struct DiffV calculate_diffVelocities(struct DiffV resV, float RM)
{
    float a, b, c, d;
    a = c = 0.5;
    b = RM / 2.0;
    d = -b;

    // Calculation with pre-evaluated formulae
    resV.VL = (resV.VT * a) + (b * resV.omega);
    resV.VR = (resV.VT * c) + (d * resV.omega);

    printf("Vt = %f, omega = %f, rm = %.2f => VL = %.2f, VR = %.2f\n", resV.VT, resV.omega, RM, resV.VL, resV.VR);
    return resV;
}

// Function to test the calculate_diffVelocities function
void test_calculate_diffVelocities() {
    printf("Starting permute test...\n");
    permute_test();  // Call the permute_test function to check multiple test cases

    printf("\n\ncase test\n");
    test();
};

void test()
{
    float test_cases[4][2] = {
        {1.0, 0.0},  // Case 10: VT = 1.0, omega = 0.0
        {0.0, 1.0},  // Case 01: VT = 0.0, omega = 1.0
        {1.0, 1.0},  // Case 11: VT = 1.0, omega = 1.0
        {2.0, 3.0}   // Case 23: VT = 2.0, omega = 3.0
    };
    
    // Expected results (VL, VR) for each test case
    struct DiffV expected[4] = {
        {1.0, 0.0, 0.5, 0.5},   // Expected VL=0.5, VR=0.5 for Case 10
        {0.0, 1.0, 7.5, -7.5},  // Expected VL=7.5, VR=-7.5 for Case 01
        {1.0, 1.0, 8.0, -7.0},  // Expected VL=8.0, VR=-7.0 for Case 11
        {2.0, 3.0, 23.5, -21.5}  // Expected VL=23.5, VR=21.5 for Case 23
    };

    int passed_tests = 0;

    // Run through all test cases
    for (int i = 0; i < 4; i++) {
        // Set VT and omega for the test case
        struct DiffV testV = {test_cases[i][1], test_cases[i][0], 0.0, 0.0};  // Set omega and VT

        // Call the function to calculate VL and VR
        testV = calculate_diffVelocities(testV,RM);

        // Check if the calculated results match the expected values
        if (fabs(testV.VL - expected[i].VL) < 0.01 && fabs(testV.VR - expected[i].VR) < 0.01) {
            printf("Test Case %d PASSED!\n\n", i + 1);
            passed_tests++;
        } else {
            printf("Test Case %d FAILED!\n\n", i + 1);
            printf("Expected VL = %.2f, VR = %.2f, but got VL = %.2f, VR = %.2f\n",
                   expected[i].VL, expected[i].VR, testV.VL, testV.VR);
        }
    }

    if (passed_tests == 4) {
        printf("All test cases passed!\n");
    }
}
// Permutation test function
void permute_test() {
    int i, j, k;

    // Example test case values
    float VT_val[5] = {3.0, 1.5, 2.2, 4.1, 0.5};    // Translational velocities
    float RM_val[5] = {1.0, 2.0, 1.5, 2.5, 3.0};    // Robot radius values
    float omega_val[5] = {0.5, 1.0, 1.5, 2.0, 2.5}; // Angular velocities

    // Example expected results for VL and VR, you should fill these based on your logic
    float expected[5][5][2] = {
        {{1.0, 0.5}, {1.5, -0.5}, {2.2, 1.0}, {2.5, -1.5}, {3.0, 1.5}},
        {{2.0, 0.0}, {2.2, -0.5}, {2.5, 1.0}, {3.0, -1.0}, {3.5, 2.0}},
        {{2.5, 0.5}, {3.0, -0.5}, {3.5, 1.0}, {4.0, -1.5}, {4.5, 2.5}},
        {{3.0, 1.0}, {3.5, -0.5}, {4.0, 2.0}, {4.5, -2.0}, {5.0, 3.0}},
        {{0.5, 0.0}, {1.0, -0.5}, {1.5, 1.0}, {2.0, -1.0}, {2.5, 1.5}}
    };

    // Iterate through all combinations of VT, RM, and omega
    for (i = 0; i < 5; i++) {
        for (j = 0; j < 5; j++) {
            for (k = 0; k < 5; k++) {
                float VT = VT_val[i];
                float RM = RM_val[j];
                float omega = omega_val[k];

                struct DiffV test_input = {omega, VT, 0.0, 0.0};
                struct DiffV output = calculate_diffVelocities(test_input, RM);

                float output_VL = output.VL;
                float output_VR = output.VR;

                // Check if the calculated result matches the expected values
                if (fabs(output_VL - expected[i][j][0]) < 0.01 && fabs(output_VR - expected[i][j][1]) < 0.01) {
                    printf("Passed for VT = %.2f, RM = %.2f, omega = %.2f\n", VT, RM, omega);
                } else {
                    printf("Error for VT = %.2f, RM = %.2f, omega = %.2f\n", VT, RM, omega);
                    printf("Expected VL = %.2f, VR = %.2f, but got VL = %.2f, VR = %.2f\n", 
                        expected[i][j][0], expected[i][j][1], output_VL, output_VR);
                    return; // Exit on error
                }
            }
        }
    }

    printf("All test cases passed!\n");
}

int main() {
    printf("hello world\n");
    calculate_diffVelocities(botV, RM);
    test_calculate_diffVelocities();  // Call the test function here
    return 0;
}
