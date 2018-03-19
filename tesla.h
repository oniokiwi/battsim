/*
 * Copyright © kiwipower 2017
 *
 * Header file to simulate battery charge/discharge profile
 */
#ifndef TESLA_DOT_H
#define TESLA_DOT_H

#include <stdint.h>
#include <modbus/modbus.h>
#include "typedefs.h"

#define enableDebug                   1
#define dumpMemory                    2
#define firmwareVersion               101
#define directRealTimeout             1023
#define directRealHeartbeat           1022
#define statusFullChargeEnergy        205
#define statusNorminalEnergy          207
#define directPower                   1020
#define realMode                      1000
#define powerBlock                    1002


//
// Public functions
//
void  tesla_init(init_param_t* );
void  tesla_dispose();
void* tesla_thread_handler( void *ptr );
int   tesla_process_single_register(uint16_t address, uint16_t data);
int   tesla_write_multiple_addresses(uint16_t start_address, uint16_t quantity, uint8_t* pdata);

#endif
