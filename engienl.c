/*
     This file is part of libmicrohttpd
     Copyright (C) 2007 Christian Grothoff (and other contributing authors)

     This library is free software; you can redistribute it and/or
     modify it under the terms of the GNU Lesser General Public
     License as published by the Free Software Foundation; either
     version 2.1 of the License, or (at your option) any later version.

     This library is distributed in the hope that it will be useful,
     but WITHOUT ANY WARRANTY; without even the implied warranty of
     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
     Lesser General Public License for more details.

     You should have received a copy of the GNU Lesser General Public
     License along with this library; if not, write to the Free Software
     Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "engienl.h"

#include <sys/types.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <microhttpd.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <json.h>
#include <sys/syscall.h>
#include <stdbool.h>
#include <pthread.h>
#include "typedefs.h"
#include "curl_handler.h"

#define MAX_PATH 1024

// Private data
static modbus_mapping_t *mb_mapping;
static modbus_t* ctx;
static unsigned short stateOfCharge;
static unsigned short stateOfChargeDefault = 50;

static pthread_t thread1;
static uint8_t terminate1;

static pthread_t thread2;
static uint8_t terminate2;

// proclet
static int   _DebugEnable(uint16_t data);
static int   _getStateOfCharge ();
static int   _setPowerToDeliver (uint16_t );

static void  _remove_character(char *buffer, int character);
static void *_microhttpd_handler( void *ptr );
static void  _parse_json(const char* str);
static int   _ahc_echo(void * cls, struct MHD_Connection * connection, const char * url,
                       const char * method, const char * version, const char * upload_data,
                        size_t * upload_data_size, void ** ptr);

void _parse_json(const char* str)
{
    struct json_object *object, *tmp, *jobj;
    int length;
    char buf[128];
    float p,s;
    long int t;

    jobj = json_tokener_parse(str);

    // key and val don't exist outside of this bloc
    json_object_object_foreach(jobj, key, val)
    {
        switch (json_object_get_type(val))
        {
            case json_type_array:
                length = json_object_array_length(val);
                tmp = json_object_array_get_idx(val, length -1);
                strcpy(buf, json_object_to_json_string(tmp));
                sscanf(buf,"{ \"timestamp\": %ld, \"powerDeliveredkW\": %f, \"stateOfCharge\": %f }", &t, &p, &s);
                stateOfCharge = (uint16_t)s;
                break;
        }
    }
    json_object_put(jobj);
}

int _ahc_echo(void * cls,
            struct MHD_Connection * connection,
            const char * url,
            const char * method,
            const char * version,
            const char * upload_data,
            size_t * upload_data_size,
            void ** ptr)
{
    const char * page = cls;
    struct MHD_Response * response;
    int reply_status;
    int ret;
    post_data_t *post = NULL;

    if (0 != strcmp(method, "PUT"))
    {
        return MHD_NO; /* unexpected method */
    }
    post = (post_data_t*)*ptr;
    if(post == NULL)
    {
        post = malloc(sizeof(post_data_t));
        post->status = false;
        *ptr = post;
    }
    if(!post->status)
    {
        post->status = true;
        return MHD_YES;
    }
    else
    {
        static int length = 0;
        if(*upload_data_size != 0)
        {
            length = *upload_data_size + 1;         // add space for null character
            *upload_data_size = 0;
            post->buff = malloc(length);
            post->buff[length] = '\0';             // ensure null termination
            snprintf(post->buff, length,"%s",upload_data);
            return MHD_YES;
        }
        else
        {
            _parse_json((const char*)post->buff);
            curl_sendReadings((const char*)post->buff, length);
            free(post->buff);
        }
    }
    if(post != NULL)
    {
        free(post);
    }
    response = MHD_create_response_from_buffer (0, NULL,MHD_RESPMEM_PERSISTENT);
    ret = MHD_queue_response(connection, MHD_HTTP_OK, response);
    MHD_destroy_response(response);
    return ret;
}

int _DebugEnable(uint16_t data)
{
    bool val =  data & 0x0001;
    printf("%s - %s\n", __PRETTY_FUNCTION__, val?"TRUE":"FALSE");
    modbus_set_debug(ctx, val);
}

int _getStateOfCharge ()
{
    uint16_t *address;
    uint16_t address_offset;
    int retval = MODBUS_SUCCESS; // need to figure out what this constant is

    address_offset = mb_mapping->start_registers + StateOfCharge;
    address = mb_mapping->tab_registers + address_offset;
    *address =  stateOfCharge;

    return retval;
}

int _setPowerToDeliver (uint16_t data )
{
    int retval = MODBUS_SUCCESS;

    curl_sendPowerToDeliver(data);
    return retval;
}


void *_microhttpd_handler( void *ptr )
{
    char *terminate;
    mhttpd_thread_param_t* param = (mhttpd_thread_param_t*) ptr;
    terminate = param->terminate;
    free(param);
    struct MHD_Daemon *d;

    d = MHD_start_daemon (MHD_USE_AUTO | MHD_USE_INTERNAL_POLLING_THREAD ,
                          8888,
                          NULL, NULL, &_ahc_echo, NULL,
                          MHD_OPTION_CONNECTION_TIMEOUT, (unsigned int) 120,
                          MHD_OPTION_STRICT_FOR_CLIENT, (int) 1,
                          MHD_OPTION_END);

    if (NULL == daemon)
    {
        printf("unable to start microhttpd server\n");
        exit(1);
    }

    while (*terminate == false)
    ;
    MHD_stop_daemon (d);
    return 0;
}

void engienl_init(init_param_t* param)
{
    mhttpd_thread_param_t* mhttpd_thread_param;
    curl_thread_param_t* curl_thread_param;

    engienl_disconnect();
    mb_mapping = param->modbus_mapping;
    terminate1 = FALSE;
    mhttpd_thread_param = malloc(sizeof (mhttpd_thread_param_t));
    mhttpd_thread_param -> terminate = &terminate1;
    pthread_create( &thread1, NULL, _microhttpd_handler, mhttpd_thread_param);

    terminate2 = FALSE;
    curl_thread_param = malloc(sizeof (curl_thread_param_t));
    curl_thread_param -> terminate = &terminate2;
    strcpy(curl_thread_param->powerToDeliverURL, param->powerToDeliverURL);
    strcpy(curl_thread_param->submitReadingsURL, param->submitReadingsURL);
    pthread_create( &thread2, NULL, curl_handler, curl_thread_param);
}

void engienl_dispose()
{
    printf("%s entry\n", __PRETTY_FUNCTION__ );
    terminate1 = true;
    pthread_join(thread1, NULL);
    printf("%s exit\n", __PRETTY_FUNCTION__ );
}

void engienl_disconnect()
{
	stateOfCharge = stateOfChargeDefault;
}


int  engienl_process_single_register(uint16_t address, uint16_t data)
{
    switch (address)
    {
    case enableDebugTrace:
        _DebugEnable(data);
        break;

    case StateOfCharge:
        _getStateOfCharge();
        break;

    case PowerToDeliver:
        _setPowerToDeliver(data);
        break;
    }

    return 0;
}

int engienl_write_multiple_addresses(uint16_t start_address, uint16_t quantity, uint8_t* pdata)
{
    int retval = MODBUS_SUCCESS;

    int i;
    uint16_t *address;
    uint16_t address_offset;

    address_offset = mb_mapping->start_registers + start_address;
    address = mb_mapping->tab_registers + address_offset;
    for ( i = 0; i < quantity; i++ )
    {
        if ( address < (mb_mapping->tab_registers + mb_mapping-> nb_registers) )
        {
            *address++  = (*pdata++ << 8) | *pdata++;
        }
    }

    return retval;
}


