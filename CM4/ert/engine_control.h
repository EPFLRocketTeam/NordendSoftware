/**
 * @file 		control.h
 * @brief 		Main control of the program through the global state machine
 *
 * @date 		20.01.2022
 * @author 		Iacopo Sprenger
 *
 * @ingroup 	engine_control
 * @{
 */

#ifndef ENGINE_CONTROL_H
#define ENGINE_CONTROL_H

/**********************
 *  INCLUDES
 **********************/

#include <stdint.h>
#include <util.h>
#include <solenoid.h>


/**********************
 *  CONSTANTS
 **********************/

/**********************
 *  MACROS
 **********************/

/**********************
 *  TYPEDEFS
 **********************/

/**
 * @brief State of the control FSM
 *
 * the description of the states don't make sense -> must be reviewed
 */




/**
 * The commands are processed through a message queue so that none can be missed.
 *
COMMANDE             	PARAMETRE
-----------------------------------------
COMMAND_NONE       	| 	aucun
COMMAND_ARM			|	aucun
COMMAND_DISARM		|	aucun
COMMAND_PRESSURE	|	aucun
COMMAND_IGNITE		|	aucun
COMMAND_RECOVER		|	aucun
COMMAND_VENT_N2O	|	ouvert/ferme
COMMAND_VENT_ETH	|	ouvert/ferme
COMMAND_MAN_PRESS	|	ouvert/ferme
COMMAND_MAN_PURGE	|	ouvert/ferme
COMMAND_VALVE_N2O	|	angle en degres
COMMAND_VALVE_ETH	|	angle en degres
COMMAND_ABORT 		|	aucun
 */



/**********************
 *  VARIABLES
 **********************/

/**********************
 *  PROTOTYPES
 **********************/

#ifdef __cplusplus
extern "C" {
#endif

void engine_control_command_push(control_command_t cmd, int32_t parameter);

void engine_control_thread(void *arg);

#ifdef __cplusplus
} // extern "C"
#endif /* __cplusplus */

#endif /* TEMPLATE_H */

/** @} */

/* END */
