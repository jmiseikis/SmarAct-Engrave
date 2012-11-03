
/* Justinas Miseikis
 * Nina Sauthoff
 * Matthias Wueest */

#include "engrave.h"

int32_t engrave_p(uint32_t smaract_system,float target_x[],float target_y[],float target_z[], int n, unsigned int ampl, unsigned int freq, int useScale)

{
    uint16_t i;

    // Check that the position is actually at zero
    float posX, posY, posZ;
    smaract_get_pos(smaract_system, CHANNELX, &posX);
    smaract_get_pos(smaract_system, CHANNELY, &posY);
    smaract_get_pos(smaract_system, CHANNELZ, &posZ);
    fprintf(stderr, "Zero position coords: %f %f %f\n", posX, posY, posZ);

	// Setting scaling
    float divider = 1000000;
    float scale = 1;

    // Check for scale
    if (useScale == 1) {
        scale = SCALE;
    }

    // Draw figure
    for (i=0; i < n; i++) {
        fprintf(stderr, "Moving to position %d: %f %f %f\n", i, target_x[i], target_y[i], target_z[i]);


        // ### USING OUR P CONTROLLER ###
        // Move in x axis
        P_position_control(smaract_system, CHANNELX, target_x[i]*scale/divider, ampl, freq);
        // Move in y axis
        P_position_control(smaract_system, CHANNELY, target_y[i]*scale/divider, ampl, freq);
        // Move in z axis
        P_position_control(smaract_system, CHANNELZ, target_z[i]/divider, ampl, freq);

        // Get position for all the axis
        smaract_get_pos(smaract_system, CHANNELX, &posX);
        smaract_get_pos(smaract_system, CHANNELY, &posY);
        smaract_get_pos(smaract_system, CHANNELZ, &posZ);

        fprintf(stdout, "%f\t %f\t %f\n", posX, posY, posZ);
    }

    return 0;
}


int32_t readfile(float* x, float* y, float *z, int* size){

    FILE *file;
    //float data[100][3];
    int i=0;

    fprintf(stderr, "Reading the file...\n");

    file=fopen("coords.dat", "r");

    while (!feof(file)) {
        fscanf(file, "%f%f%f", &x[i], &y[i], &z[i]);

        // Update size
        *size = i;
        i++;
    }

    fprintf(stderr, "Reading file: Done!\n");

    return 0;
}
