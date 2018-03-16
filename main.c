/*
 * Copyright © 2008-2014 Stéphane Raimbault <stephane.raimbault@gmail.com>
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <modbus/modbus.h>
#include <getopt.h>
#include <sys/socket.h>
#include <stdbool.h>
#include <sys/syscall.h>
#include "curl_handler.h"
#include "typedefs.h"
#include "nec.h"
#include "tesla.h"
#include "engienl.h"


#define POWER_TO_DELIVER_URL_DEFAULT "http://localhost:1880"
#define SUBMIT_READINGS_URL_DEFAULT  "http://localhost:1880/testpoint"

const uint16_t UT_REGISTERS_NB = 100000;
static uint8_t query[MODBUS_TCP_MAX_ADU_LENGTH];

#define MODBUS_DEFAULT_PORT 1502

static init_param_t param;

static void (*init)(init_param_t *);
static void (*dispose)();
static int (*thread_handler)( void *ptr );
static int (*process_handler)(uint16_t address, uint16_t data);
static int (*process_write_multiple_addresses)(uint16_t start_address, uint16_t quantity, uint8_t* pdata);


static void usage(const char *app_name)
{
    printf("Usage:\n");
    printf("%s [option <value>] ...\n", app_name);
    printf("\nOptions:\n");
    printf(" -p \t\t # Set Modbus port to listen on for incoming requests (Default 1502)\n");
    printf(" -? \t\t # Print this help menu\n");
    printf("\nExamples:\n");
    printf("%s -p 1502  \t # Change the listen port to 1502\n", app_name);
    exit(1);
}

static void init_default(init_param_t * param)
{
	const char *app_name = "mbservsim";
    usage(app_name);
    exit(1);
}

static void modbus_mem_init()
{
	param.modbus_mapping = modbus_mapping_new_start_address(
       0, 0,
       0, 0,
       0, UT_REGISTERS_NB,
       0, 0);

    if (param.modbus_mapping == NULL)
    {
        printf("Failed to allocate the mapping: %s\n", modbus_strerror(errno));
        exit(1); // all bets are off
    }
}


static void scan_options(int argc, char* argv[])
{
	int opt;

	// set some default options
	param.port = MODBUS_DEFAULT_PORT;
	strncpy(param.powerToDeliverURL, POWER_TO_DELIVER_URL_DEFAULT, strlen(POWER_TO_DELIVER_URL_DEFAULT));
	strncpy(param.submitReadingsURL, SUBMIT_READINGS_URL_DEFAULT, strlen(SUBMIT_READINGS_URL_DEFAULT));

    while ((opt = getopt(argc, argv, "p:u:k:t:")) != -1)
    {
        switch (opt)
        {
        case 'p':
            param.port = atoi(optarg);
            break;
        case 'u':
        	strncpy(param.powerToDeliverURL, optarg, strlen(optarg));
            break;
        case 'k':
        	strncpy(param.submitReadingsURL, optarg, strlen(optarg));
            break;

        case 't':
        	if (strncmp("TESLA", optarg, strlen(optarg)) == 0)
        	{
				init = tesla_init;
				dispose = tesla_dispose;
				process_handler = tesla_process_single_register;
				process_write_multiple_addresses = tesla_write_multiple_addresses;
				printf("starting tesla battery simulator application - port (%d)\n", param.port);
        	}
        	else if (strncmp("NEC", optarg, strlen(optarg)) == 0)
        	{
				init = nec_init;
				dispose = nec_dispose;
				process_handler = nec_process_single_register;
				process_write_multiple_addresses = nec_write_multiple_addresses;
				printf("starting nec battery simulator application - port (%d)\n", param.port);
        	}
        	else if (strncmp("ENGIENL", optarg, strlen(optarg)) == 0)
        	{
				init = engienl_init;
				dispose = engienl_dispose;
				process_handler = engienl_process_single_register;
				process_write_multiple_addresses = engienl_write_multiple_addresses;
				printf("starting engienl battery simulator application - port (%d) "
						"powerToDeliverURL (%s) "
						"submitReadingsURL (%s)\n",
						param.port, param.powerToDeliverURL, param.submitReadingsURL);
        	}
        	else if (strncmp("BASEPOWER", optarg, strlen(optarg)) == 0)
        	{
				//init = basepower_init;
				//dispose = basepower_dispose;
				//process_handler = basepower_process_single_register;
				printf("starting basepower battery simulator application - port (%d)\n", param.port);
        	}
        	else
        	{
        		usage(*argv);
        	}
            break;

        default:
            usage(*argv);
        }
    }
}

int main(int argc, char* argv[])
{
	void query_handler(modbus_pdu_t* mb);
    int rc, s = -1;
    bool done = FALSE;
    init = init_default;

    modbus_mem_init();
    scan_options(argc, argv);
    init(&param);

    for (;;)
    {
        param.ctx = modbus_new_tcp(NULL, param.port);
        if ( param.ctx == NULL )
        {
            printf("Failed creating modbus context\n");
            return -1;
        }
        s = modbus_tcp_listen(param.ctx, 1);
        modbus_tcp_accept(param.ctx, &s);
        done = FALSE;
        while (!done)
        {
            rc = modbus_receive(param.ctx, query);
            switch (rc)
            {
            case -1:
                close(s); // close the socket
                modbus_close(param.ctx);
                modbus_free(param.ctx);
                param.ctx = NULL;
                done = TRUE;
                break;

            case 0:
                // No data received
                break;

            default:
                query_handler((modbus_pdu_t*) query);
                break;
            }
        }
    } // for (;;)
    dispose();
    return 0;
}

/*
***************************************************************************************************************
 \fn      tesla_query_handler(modbus_pdu_t* mb)
 \brief   processess all incoming commands

 Process all input commands. The Modbus function code 0x17 which is not standard seems to exhibit non standaard
 data structure seen not belows.

 \note

      MODBUS_FC_READ_HOLDING_REGISTERS
      MODBUS_FC_WRITE_SINGLE_REGISTER - has the following data format
      ------------------------------------------------
      | TID | PID | LEN | UID | FC | [W|R]S | [W|R]Q |
      ------------------------------------------------
      0     1     3     5     7    8        11       13

      MODBUS_FC_WRITE_MULTIPLE_REGISTERS - has the following data format operation
      -------------------------------------------------------
      | TID | PID | LEN | UID | FC | WS | WQ | WC | WR x nn |
      -------------------------------------------------------
      0     1     3     5     7    8    11   13   14

      MODBUS_FC_WRITE_AND_READ_REGISTERS - has the following data format
      -----------------------------------------------------------------
      | TID | PID | LEN | UID | FC | RS | RQ | WS | WQ | WC | WR x nn |
      -----------------------------------------------------------------
      0     1     3     5     7    8    11   13   15   17   18

      TID = Transaction Id, PID = Protocol Id, LEN = Total length of message, UID = unit Id
      FC = Function Code, RS = Read start address, RQ = Read quantity, WS = Write start address,
      WQ = Write quantity, WC = Write count, WR = Write Register. nn => WQ x 2 bytes
**************************************************************************************************************
*/

void query_handler(modbus_pdu_t* mb)
{
    const int convert_bytes2word_value = 256;
    int i,j,retval = MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
    uint16_t address,value,count;
    int len = __bswap_16(mb->mbap.length) - 2; // len - fc - unit_id
    uint8_t fc;

   // for ( i = 0; i < len; i++ ) {
	fc = mb->fcode;
	switch ( fc ){
	case MODBUS_FC_READ_HOLDING_REGISTERS:
		printf("%s MODBUS_FC_READ_HOLDING_REGISTERS\n", __PRETTY_FUNCTION__);
		address = (mb->data[i++] * convert_bytes2word_value) + mb->data[i++]; // address
		value   = (mb->data[i++] * convert_bytes2word_value) + mb->data[i++]; // data
		printf("%s MODBUS_FC_READ_HOLDING_REGISTERS - address(%d), value(%d)\n", __PRETTY_FUNCTION__, address, value);
		retval  = process_handler(address, value);
		break;

	case MODBUS_FC_WRITE_SINGLE_REGISTER:
		printf("%s MODBUS_FC_WRITE_SINGLE_REGISTER\n", __PRETTY_FUNCTION__);
		address = (mb->data[i++] * convert_bytes2word_value) + mb->data[i++]; // address
		value   = (mb->data[i++] * convert_bytes2word_value) + mb->data[i++]; // data
		retval  = process_handler(address, value);
		break;

	case MODBUS_FC_WRITE_MULTIPLE_REGISTERS:
		printf("%s MODBUS_FC_WRITE_MULTIPLE_REGISTERS\n", __PRETTY_FUNCTION__);
		address = (mb->data[i++] * convert_bytes2word_value) + mb->data[i++]; // address
		count = (mb->data[i++] * convert_bytes2word_value) + mb->data[i++];   // register count
		i++;                                             // skip over byte count
		retval = process_write_multiple_addresses(address, count, &mb->data[i]);
		i += (count*2);
		break;

	case MODBUS_FC_WRITE_AND_READ_REGISTERS:
		//printf("%s MODBUS_FC_WRITE_AND_READ_REGISTERS\n", __PRETTY_FUNCTION__);
		//address = (mb->data[i++] * convert_bytes2word_value) + mb->data[i++]; // address
		//value   = (mb->data[i++] * convert_bytes2word_value) + mb->data[i++]; // data
		//retval  = process_handler(address, value);
		//address = (mb->data[i++] * convert_bytes2word_value) + mb->data[i++]; // address
		//count = (mb->data[i++] * convert_bytes2word_value) + mb->data[i++];   // register count
		//i++;                                             // skip over byte count
		//retval = process_write_multiple_addresses(address, count, &mb->data[i]);
		//i += (count*2);
		break;

	default:
		retval = MODBUS_EXCEPTION_ILLEGAL_FUNCTION;
		break;
        }
   // }
    if ( retval == MODBUS_SUCCESS)
    {
        modbus_reply(param.ctx, (uint8_t*)mb, sizeof(mbap_header_t) + sizeof(fc) + len, param.modbus_mapping); // subtract function code
    }
    else
    {
       modbus_reply_exception(param.ctx, (uint8_t*)mb, retval);
    }
}



