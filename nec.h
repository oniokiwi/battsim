/*
 * Copyright © kiwipower 2017
 *
 * Header file to simulate battery charge/discharge profile
 */
#ifndef NEC_DOT_H
#define NEC_DOT_H

#include <stdint.h>
#include <modbus/modbus.h>
#include "typedefs.h"

#define enableDebugTrace              (100000L - 1)
#define realpoweroutput                   6
#define RealPowerSetPoint             14007
#define ReactivePowerSetPoint         14009
#define averagesoc                       10
#define SocRef                        14011
#define modecontrol                   14012
#define powerblockenablecontrol12H    14013
#define powerblockenablecontrol12L    14014
#define HeartbeatFromPGM              14017
#define dispatchmode                  14020
#define pslewrate                     14028
#define qslewrate                     14029
#define ackalarams                    14050

enum OperatingMode
{
    OperatingModeShutDown = 0,
	OperatingModeManual = 4,
	OperatingModeOperational = 32
};

enum DispatchMode
{
    DispatchModeIdle = 0,
	DispatchModeDispatch
};

void  nec_init(init_param_t* );
void  nec_dispose();
void* nec_thread_handler( void *ptr );
int   nec_process_single_register(uint16_t address, uint16_t data);
int   nec_write_multiple_addresses(uint16_t start_address, uint16_t quantity, uint8_t* pdata);

#endif
