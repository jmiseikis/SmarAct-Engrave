
/* Justinas Miseikis
 * Nina Sauthoff
 * Matthias Wueest */

// System includes
#include <stdio.h>
// Smaract micro-positioner
#include "actuator.h"
#include "smaract_util.h"
// Needed for data types
#include <stdint.h>

//Initialize system
int32_t 
actuators_initialize(uint32_t config)
{
  if (0 !=smaract_open(config) )
    return -1;
   
   return 0;
}

//Close system
int32_t
actuators_close()
{
  if (0 != smaract_close() )
    return 1;

    return 0;
}

//Set zero position
int32_t
actuators_set_zero_pos(uint32_t smaract_system, uint32_t smaract_chan)
{
    smaract_set_zero_pos(smaract_system, smaract_chan);

    return 0;
}


//Move steps, wait until the movement is completed and get the current position
int32_t
actuators_move_and_get_pos(uint32_t smaract_system, 
                           uint32_t smaract_chan, 
                           int32_t steps,
                           uint32_t ampl, 
                           uint32_t freq,
                           float* position)
{
	// Move steps and wait until the movement is completed. Use the get_status 
	// function to see when the position is finished
    unsigned int status;

    // Check if values are within the range
    if (steps < -30000 || steps > 30000)
    {
        fprintf(stderr, "ERROR: steps is out of range\n");
        return -1;
    }
    if (ampl < 150 || ampl > 1000)
    {
        fprintf(stderr, "ERROR: amplitude is out of range\n");
        return -1;
    }
    if (freq > 18500)
    {
        fprintf(stderr, "ERROR: frequency is out of range\n");
        return -1;
    }

    // Move steps
    smaract_move_steps(smaract_system,smaract_chan, steps, ampl, freq);


    // Check status
    do {
        smaract_get_status(smaract_system, smaract_chan, &status);
    } while(status != 0);
  
	// Get the position 
    smaract_get_pos(smaract_system, smaract_chan,position);


  return 0;
}
  
// Get the speed of communication for one command
int32_t
actuators_get_comm_speed(uint32_t smaract_system, uint32_t smaract_chan,float* speed)
{
    double t; // For timer
    uint16_t i, numTimes = 200;
    float pos;

    tic();
    for (i=0; i < numTimes; i++) {
        smaract_get_pos(smaract_system, smaract_chan, &pos);
    }

    t = toc(0);
    t /= numTimes; // Time for one read
    // Convert time to Hz
    *speed = 1/t;
   
   return 0;
}

int32_t P_position_control(uint32_t smaract_system, uint32_t smaract_chan, float desDispl, uint32_t ampl, uint32_t freq)
{
    // Define constants
    int32_t steps;
    uint32_t Kp = K_P; // Subject to change
    float errTol = ERROR_TOL;
    float err;
    float pos;

    uint32_t timestamp = 0;
    // Get current position
    smaract_get_pos(smaract_system, smaract_chan, &pos);
    // Calculate the error
    err = desDispl - pos;

    fprintf(stderr, "Error: %f", err);

    // While loop until convergence
    while ( (err > errTol) || (err < -errTol) ) {
        // Calculate steps
        steps = (int) Kp * err;

        // Move the actuator and get new position
        actuators_move_and_get_pos(smaract_system, smaract_chan, steps, ampl, freq, &pos);

        // Calculate new error
        err = desDispl - pos;

        // Write results to file
        //fprintf(stdout, "%d\t%f\t%f\n", timestamp, err, pos);

        // Increment timestamp
        timestamp++;
    }

    fprintf(stderr, "Woohoo! Target reached! :) CurrPos: %f\n", pos);

    return 0;
}

int32_t P_relative_position_control(uint32_t smaract_system, uint32_t smaract_chan, float desDispl)
{
    // Define constants
    uint32_t ampl = 750;
    uint32_t freq = 8000;
    int32_t steps;
    uint32_t Kp = 1000000; // Subject to change
    float errTol = 0.000001;
    float err;
    float pos;

    uint32_t timestamp = 0;
    // Get current position
    smaract_get_pos(smaract_system, smaract_chan, &pos);
    // Calculate the error
    err = desDispl - pos;

    //fprintf(stdout, "%s\t%s\t%s\n", "timestamp", "err", "pos");

    // While loop until convergence
    while ( (err > errTol) || (err < -errTol) ) {
        // Calculate steps
        steps = (int) Kp * err;

        // Move the actuator and get new position
        actuators_move_and_get_pos(smaract_system, smaract_chan, steps, ampl, freq, &pos);

        // Calculate new error
        err = desDispl - pos;

        // Write results to file
        //fprintf(stdout, "%d\t%f\t%f\n", timestamp, err, pos);

        // Increment timestamp
        timestamp++;
    }

    fprintf(stderr, "Woohoo! Target reached! :) CurrPos: %f\n", pos);

    return 0;
}

int32_t ThreeDfigure(uint32_t smaract_system)

{
    uint16_t i;

    // ### FOR ABSOLUTE POSITIONS
    //float desDispl = 0.005;     // 5 mm
    float desDispl = 0.00005;   // 5 um

    // ### FOR RELATIVE POSITIONS
    //float desDispl = 0.01;     // 5 mm
    //float desDispl = 0.0001;   // 5 um

    // Define positions

    // ### FOR ABSOLUTE POSITIONS

    float displ_x[9] = {desDispl, desDispl, -desDispl, -desDispl, desDispl, desDispl, -desDispl, -desDispl, desDispl};
    float displ_y[9] = {desDispl, -desDispl, -desDispl, -desDispl, -desDispl, desDispl, desDispl, desDispl, desDispl};
    float displ_z[9] = {desDispl, desDispl, desDispl, -desDispl, -desDispl, -desDispl, -desDispl, desDispl, desDispl};


    // ### FOR RELATIVE POSITIONS
    /*
    float displ_x[9] = {0.5*desDispl, 0, -desDispl, 0, desDispl, 0, -desDispl, 0, desDispl};
    float displ_y[9] = {0.5*desDispl, -desDispl, 0, 0, 0, desDispl, 0, 0, 0};
    float displ_z[9] = {0.5*desDispl, 0, 0, -desDispl, 0, 0, 0, desDispl, 0};
    */

    // Place the actuator to absolute zero
    /*
    // Move in x axis
    smaract_move_pos_abs (smaract_system, CHANNELX, 0.0f, 0);
    // Move in y axis
    smaract_move_pos_abs (smaract_system, CHANNELY, 0.0f, 0);
    // Move in z axis
    smaract_move_pos_abs (smaract_system, CHANNELZ, 0.0f, 0);
    */

    //usleep(3000000);

    // ### FOR RELATIVE POSITIONS - SET CURR POS TO ZERO
    actuators_set_zero_pos(SMARACT_SYSTEM, CHANNELX);
    actuators_set_zero_pos(SMARACT_SYSTEM, CHANNELY);
    actuators_set_zero_pos(SMARACT_SYSTEM, CHANNELZ);



    // Check that hte position is actually at zero
    float posX, posY, posZ;
    smaract_get_pos(smaract_system, CHANNELX, &posX);
    smaract_get_pos(smaract_system, CHANNELY, &posY);
    smaract_get_pos(smaract_system, CHANNELZ, &posZ);
    fprintf(stderr, "Zero position coords: %f %f %f\n", posX, posY, posZ);


    for (i=0; i < 9; i++) {
        fprintf(stderr, "Moving to position %d: %f %f %f\n", i, displ_x[i], displ_y[i], displ_z[i]);

        // ### USING BUILT-IN FUNCTIONS ###
        /*
        // Move in x axis
        smaract_move_pos_abs (smaract_system, CHANNELX, displ_x[i], 0);
        // Move in y axis
        smaract_move_pos_abs (smaract_system, CHANNELY, displ_y[i], 0);
        // Move in z axis
        smaract_move_pos_abs (smaract_system, CHANNELZ, displ_z[i], 0);
        usleep(3000000);
        */

        // ### USING OUR P CONTROLLER ###
        // Move in x axis
        P_relative_position_control(smaract_system, CHANNELX, displ_x[i]);
        // Move in y axis
        P_relative_position_control(smaract_system, CHANNELY, displ_y[i]);
        // Move in z axis
        P_relative_position_control(smaract_system, CHANNELZ, displ_z[i]);

        // Get position for all the axis
        smaract_get_pos(smaract_system, CHANNELX, &posX);
        smaract_get_pos(smaract_system, CHANNELY, &posY);
        smaract_get_pos(smaract_system, CHANNELZ, &posZ);
        //fprintf(stdout, "%f\t %f\t %f\n", posX, posY, posZ);
    }

    return 0;
}

// Part C: helix function
int32_t helix(uint32_t smaract_system)
{

    float radius, angle, length;

    // Get inputs from the user
    fprintf(stderr, "Enter helix radius in mm\n");
    scanf("%f",&radius);
    fprintf(stderr, "Enter helix angle in rad\n");
    scanf("%f",&angle);
    fprintf(stderr, "Enter helix length in mm\n");
    scanf("%f",&length);

    // Check for limits
    // On our device, the limits were -11 to + 11
    if (radius > 11) {
        fprintf(stderr, "ERROR: Radius is out of bounds, reduce the value\n");
        return 1;
    }
    if (length > 11) {
        fprintf(stderr, "ERROR: Length is out of bounds, reduce the value\n");
        return 1;
    }

    // Normalise radius and length to the range in meters
    radius /= 1000;
    length /= 1000;

    uint16_t i=0, N=0;
    float t=0.1;
    float actposX, actposY, actposZ;
    float pos_x[1000];
    float pos_y[1000];
    float pos_z[1000];

    float b = sin (angle)*4*radius/(2*M_PI);

    // Calculate theoretical positions of the helix
    while(pos_z[N-1]<length) {
        pos_x[N]= radius*cos(t);
        pos_y[N]= radius*sin(t);
        pos_z[N] = b*t;

        fprintf(stderr, "%f %f %f\n", pos_x[N], pos_y[N], pos_z[N]);

        N++;
        t += 0.1;
    }

    // Place the actuator to the absolute zero

    // Move in x axis
    smaract_move_pos_abs (smaract_system, CHANNELX, 0.0f, 0);
    // Move in y axis
    smaract_move_pos_abs (smaract_system, CHANNELY, 0.0f, 0);
    // Move in z axis
    smaract_move_pos_abs (smaract_system, CHANNELZ, 0.0f, 0);

	// Settle down time
    usleep(500000);



    for (i=0; i < N; i++) {
        fprintf(stderr, "Moving to position %d: %f %f %f\n", i, pos_x[i], pos_y[i], pos_z[i]);

        // Move in x axis
        smaract_move_pos_abs (smaract_system, CHANNELX, pos_x[i], 0);
        // Move in y axis
        smaract_move_pos_abs (smaract_system, CHANNELY, pos_y[i], 0);
        // Move in z axis
        smaract_move_pos_abs (smaract_system, CHANNELZ, pos_z[i], 0);
        usleep(80000);


        // Get position for all the axis
        smaract_get_pos(smaract_system, CHANNELX, &actposX);
        smaract_get_pos(smaract_system, CHANNELY, &actposY);
        smaract_get_pos(smaract_system, CHANNELZ, &actposZ);
        fprintf(stdout, "%f\t %f\t %f\n", actposX, actposY, actposZ);
    }

    return 0;
}
