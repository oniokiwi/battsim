/*
 * Header files for typedefs
 *
*/
#ifndef TYPEDEFS_DOT_H
#define TYPEDEFS_DOT_H


#include <modbus/modbus.h>
#include "queue.h"

//typedef enum {false, true} bool;

#define MODBUS_SUCCESS  0

typedef enum CURL_MESSAGE_TYPE
{
    CURL_PLAIN_TEXT = 100,
    CURL_APPLICATION_JSON
}curl_message_type_t;


typedef struct process_table_struct
{
    uint16_t address;
    int (*handler)(uint16_t, uint16_t);
}process_table_t;


typedef struct optargs_struct
{
    unsigned int port;                           // port number for modbus server to listen
    unsigned int HoldingRegisters;               // Read - Write registers
    unsigned int NumberHoldingRegisters;         // Number of read - write registers
}optargs_t;

typedef struct init_param_struct
{
    uint8_t  *terminate;
    modbus_t *ctx;
    int   port;
    modbus_mapping_t *modbus_mapping;
    char powerToDeliverURL[128];                // powerToDeliverURL = ipaddress:port
    char submitReadingsURL[128];               // submitReadingsURL = ipaddress/endpoint
}init_param_t;

typedef struct thread_param_struct
{
    modbus_t *ctx;
    pthread_mutex_t* mutex;
    uint8_t *terminate;
}thread_param_t;

typedef struct tesla_params_struct
{
    modbus_t *ctx;
	uint8_t *terminate;
}tesla_thread_param_t;

typedef struct mhttpd_thread_param_struct
{
    modbus_t *ctx;
	uint8_t *terminate;
}mhttpd_thread_param_t;


typedef struct curl_thread_param_struct
{
    uint8_t *terminate;
    char powerToDeliverURL[128];                // powerToDeliverURL = ipaddress:port
    char submitReadingsURL[128];               // submitReadingsURL = ipaddress/endpoint
}curl_thread_param_t;

typedef struct mbap_header_struct
{
    uint16_t transport_id;
    uint16_t protocol_id;
    uint16_t length;
    uint8_t  unit_id;
}__attribute__((packed))mbap_header_t;

typedef struct modbus_pdu_struct
{
    mbap_header_t mbap;
    uint8_t  fcode;
    uint8_t  data[];
}__attribute__((packed))modbus_pdu_t;

typedef struct
{
    char status;
    char *buff;
}post_data_t;

typedef struct curl_data_struct
{
    link_t link;
    curl_message_type_t type;
    int length;
    char* payload;
}queue_item_t;

#endif
