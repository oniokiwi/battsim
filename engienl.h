/*
 * Copyright © kiwipower 2017
 *
 * Header file to simulate battery charge/discharge profile
 */
#ifndef ENGIENL_DOT_H
#define ENGIENL_DOT_H

#include <stdint.h>
#include <modbus/modbus.h>
#include "typedefs.h"

#define PowerToDeliver     1
#define StateOfCharge      2


void engienl_init(init_param_t* modbus_mapping);
void engienl_dispose();
void engienl_disconnect();
int  engienl_process_single_register(uint16_t address, uint16_t data);
int  engienl_write_multiple_addresses(uint16_t start_address, uint16_t quantity, uint8_t* pdata);

#endif
