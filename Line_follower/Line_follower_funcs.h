#ifndef LINE_FOLLOWER_FUNCS
#define LINE_FOLLOWER_FUNCS

struct DiffV
{
    float omega; // omega of bot
    float VT;    // total velocity of bot
    float VL;    // velocity of left wheel
    float VR;    // velocity of right wheel
};


// calculate differential VR and VL for left and right wheels
struct DiffV calculate_diffVelocities(struct DiffV resV);

// Function to check the calculate_diffVelocities function
void test_calculate_diffVelocities();


#endif 