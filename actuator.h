
/* Justinas Miseikis
 * Nina Sauthoff
 * Matthias Wueest */

/**
 * \author Dimitris Felekis & Brad Kratochvil
 * \author Copyright (C) 2010
 *
 * \brief This module controls a 3-axis micropositioning system based on 
 *        piezoelectric actuators from Smaract GmbH
 * 
 * \defgroup piezo_actuator Lab 04: Smaract piezo-actuator sample driver
 * \ingroup piezo_actuator
 * 
*/

// System includes
#include <stdio.h>
// Smaract positioner
#include <smaract_proxy_SCU.h>

// Needed for data types
#include <stdint.h>
#include <unistd.h>
#include <stdlib.h>
#include <math.h>

// Variables
#define SMARACT_SYSTEM  0
#define CHANNELX  0
#define CHANNELY  1
#define CHANNELZ  2
#define CONFIG  0
#define STATUS  1
#define STEPS  1000

// Constants for engraving
#define AMPL_ENGRAVE 700
#define FREQ_ENGRAVE 3500
#define K_P 100000
#define ERROR_TOL 0.00001
#define SCALE 0.5
#define OFFSET 3000


/** We must first initialize the 3-axis system before any other function
 *  \ingroup piezo_actuator
 *  \arg config This parameter selects the communication mode. Possible values 
 *              are SA_SYNCHRONOUS_COMMUNICATION, SA_ASYNCHRONOUS_COMMUNICATION
 *  \retval 0 if successful
 */
int32_t
actuators_initialize(uint32_t config);

/** We must close the 3-axis system 
 *  \ingroup piezo_actuator
 *  \arg config This parameter selects the communication mode. Possible values 
 *              are SA_SYNCHRONOUS_COMMUNICATION, SA_ASYNCHRONOUS_COMMUNICATION
 *  \retval 0 if successful
 */
int32_t
actuators_close();

/** Used to set the current position of the actuator as the zero position 
 *  \ingroup piezo_actuator
 *  This function sets the current position as zero.
 *  \arg smaract_system The system in which we refer to. The index is zero-based
 *  \arg smaract_chan The channel in which we refer to. The index is zero-based
 *  \retval 0 if successful
 */ 
int32_t 
actuators_set_zero_pos(uint32_t smaract_system, uint32_t smaract_chan);

/** Used to perform steps and wait until the movement is completed 
 *  \ingroup piezo_actuator
 *  This function instructs the actuator to perform a number of steps and to wait 
 *  until the movement is completed before the user is able to send the next 
 *  command. This is implemented by checking the status of the actuator.
 *  \arg smaract_system The system in which we refer to. The index is zero-based
 *  \arg smaract_chan The channel in which we refer to. The index is zero-based
 *  \arg steps Number and direction of steps to perform. The valid range is 
 *             between -30.000 and 30.000 steps
 *  \arg ampl  The amplitude that the steps are performed with. The valid range is
 *             from 150 to 1000. 150 corresponds to 15V and 1000 to 100V
 *  \arg freq   The frequency in Hz that the steps are performed with. The valid
 *              range is from 0Hz to 18.500 Hz  
 *  \retval 0 if successful
 */ 
int32_t 
actuators_move_steps_complete(uint32_t smaract_system, 
                           uint32_t smaract_chan, 
                           uint32_t steps,
                           uint32_t ampl, 
                           uint32_t freq);
                           
/** Used to perform steps, wait until the movement is completed and get the position
 *  \ingroup piezo_actuator
 *  This function instructs the actuator to perform a number of steps and to wait 
 *  until the movement is completed. This is implemented by checking the status
 *  of the actuator. After the movement is complete it gets the position 
 *  \arg smaract_system The system in which we refer to. The index is zero-based
 *  \arg smaract_chan The channel in which we refer to. The index is zero-based
 *  \arg steps Number and direction of steps to perform. The valid range is 
 *             between -30.000 and 30.000 steps
 *  \arg ampl  The amplitude that the steps are performed with. The valid range is
 *             from 150 to 1000. 150 corresponds to 15V and 1000 to 100V
 *  \arg freq   The frequency in Hz that the steps are performed with. The valid
 *              range is from 0Hz to 18.500 Hz  
 *  \arg position This is where the position of the actuator is stored
 *  \retval 0 if successful
 */ 
int32_t
actuators_move_and_get_pos(uint32_t smaract_system, 
                           uint32_t smaract_chan, 
                           int32_t steps,
                           uint32_t ampl, 
                           uint32_t freq,
                           float* position);

/** Used to test the communication speed with Smaract controler 
 *  \ingroup piezo_actuator
 *  This function tests the communication speed with Smaract controler by sending
 *  one command in Synchronous mode
 *  \arg smaract_system The system in which we refer to. The index is zero-based
 *  \arg smaract_chan The channel in which we refer to. The index is zero-based
 *  \arg speed The speed of communication for one command in Hz 
 *  \retval 0 if successful
 */ 
int32_t
actuators_get_comm_speed(uint32_t smaract_system, uint32_t smaract_chan, float* speed);

// Position control function
int32_t P_position_control(uint32_t smaract_system, uint32_t smaract_chan, float desDispl, uint32_t ampl, uint32_t freq);

// 3D position control
int32_t ThreeDfigure(uint32_t smaract_system);

// P controller for 3D position control
int32_t P_relative_position_control(uint32_t smaract_system, uint32_t smaract_chan, float desDispl);

// Helix function
int32_t helix(uint32_t smaract_system);
