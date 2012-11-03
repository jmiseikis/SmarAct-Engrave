
/* Justinas Miseikis
 * Nina Sauthoff
 * Matthias Wueest */

#include "actuator.h"
#include "engrave.h"

// System level includes
#include <stdio.h>
#include <stdint.h>

int main()
{

	float speed;

	// Initialise the system
    if ( actuators_initialize(CONFIG) != 0 ) {
		fprintf(stderr, "ERROR: cannot initialise the device\n");
	}

	//Get the communication speed
    if ( actuators_get_comm_speed(SMARACT_SYSTEM, CHANNELX, &speed) != 0 ) {
		fprintf(stderr, "ERROR: cannot get comm speed\n");
    }
    fprintf(stderr, "Comm speed: %f Hz\n", speed);


	//Set zero position to all the channels
    if ( actuators_set_zero_pos(SMARACT_SYSTEM, CHANNELX) != 0 ) {
		fprintf(stderr, "ERROR: cannot set zero pos\n");
    }
    if ( actuators_set_zero_pos(SMARACT_SYSTEM, CHANNELY) != 0 ) {
		fprintf(stderr, "ERROR: cannot set zero pos\n");
    }
    if ( actuators_set_zero_pos(SMARACT_SYSTEM, CHANNELZ) != 0 ) {
		fprintf(stderr, "ERROR: cannot set zero pos\n");
	}

	//read file
	// Declare 3 arrays to store coordinates
	float *x, *y, *z;
	int size;
	float posX, posY, posZ;
	x = (float*)malloc(100*sizeof(float));
	y = (float*)malloc(100*sizeof(float));
	z = (float*)malloc(100*sizeof(float));

	readfile(x, y, z, &size);

    //Engrave the positions
    engrave_p(SMARACT_SYSTEM, x, y, z, size, AMPL_ENGRAVE, FREQ_ENGRAVE, 0);

    // Move the actuator to new position
    P_position_control(SMARACT_SYSTEM, CHANNELY, (float)3000/(float)1000000, AMPL_ENGRAVE, FREQ_ENGRAVE);
    P_position_control(SMARACT_SYSTEM, CHANNELZ, (float)150/(float)1000000, AMPL_ENGRAVE, FREQ_ENGRAVE);


    //Set zero position to all the channels
	if ( actuators_set_zero_pos(SMARACT_SYSTEM, CHANNELX) != 0 ) {
		fprintf(stderr, "ERROR: cannot set zero pos\n");
	}
	if ( actuators_set_zero_pos(SMARACT_SYSTEM, CHANNELY) != 0 ) {
		fprintf(stderr, "ERROR: cannot set zero pos\n");
	}
	if ( actuators_set_zero_pos(SMARACT_SYSTEM, CHANNELZ) != 0 ) {
		fprintf(stderr, "ERROR: cannot set zero pos\n");
	}

	// Get position for all the axis
	smaract_get_pos(SMARACT_SYSTEM, CHANNELX, &posX);
	smaract_get_pos(SMARACT_SYSTEM, CHANNELY, &posY);
	smaract_get_pos(SMARACT_SYSTEM, CHANNELZ, &posZ);

	fprintf(stdout, "%f\t %f\t %f\n", posX, posY, posZ);

	//Engrave the positions using scaling
	engrave_p(SMARACT_SYSTEM, x, y, z, size, AMPL_ENGRAVE, FREQ_ENGRAVE, 1);




	//Close the communication with the actuators
	actuators_close();


	return 0;
}
