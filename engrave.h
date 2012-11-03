
/* Justinas Miseikis
 * Nina Sauthoff
 * Matthias Wueest */

#include <actuator.h>

// These are system includes
#include <stdio.h>
// These are the includes needed for the Smaract positioner
#include <smaract_proxy_SCU.h>


// Needed for data types
#include <stdint.h>
#include <unistd.h>
#include <stdlib.h>
#include <math.h>

// Engraves 2d figures
int32_t engrave_p(uint32_t smaract_system,float target_x[],float target_y[],float target_z[], int n, unsigned int ampl, unsigned int freq, int useScale);

// Read file
int32_t readfile(float *x, float *y, float *z, int *size);
