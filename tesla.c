#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <byteswap.h>
#include "tesla.h"
#include "typedefs.h"
#include <unistd.h>
#include <modbus/modbus.h>
#include <string.h>
#include <time.h>
#include <pthread.h>

#define BATTERY_POWER_RATING            230           // kW
#define TIME_CHARGE_FROM_0_TO_100       3000          // seconds
#define TIME_DISCHARGE_FROM_100_TO_0    2800          // seconds
#define HEARTBEAT_TIMEOUT_DEFAULT       60
#define STATE_OF_CHARGET_DEFAULT        50.0

// Private data
static modbus_t* ctx;
static modbus_mapping_t *mb_mapping;

static pthread_t thread1;
static uint8_t terminate1;
static bool debug = false;

static uint16_t heartbeatTimeout = HEARTBEAT_TIMEOUT_DEFAULT;
static uint16_t heartbeat = 0;

static int32_t StatusFullChargeEnergy = 100;
static int32_t StatusNorminalEnergy   = 50;

static float battery_discharge_decrement = 0.0;
static float battery_charge_increment = 0.0;
static bool battery_charging = false;
static bool battery_discharging = false;

static const uint16_t POWER_BLOCK_ALL = 2;
static const uint32_t sign_bit_mask             = 0x80000000;

static const float battery_charge_resolution    = 100.00 / (BATTERY_POWER_RATING * TIME_CHARGE_FROM_0_TO_100);     // % increase in charge per sec
static const float battery_discharge_resolution = 100.00 / (BATTERY_POWER_RATING * TIME_DISCHARGE_FROM_100_TO_0);  // % decrease in charge per sec
static const float battery_fully_charged        = 100.00;
static const float battery_fully_discharged     = 0.0;
static float state_of_charge = STATE_OF_CHARGET_DEFAULT;

//
// Lookup table for process functions
//
const process_table_t tesla_process_table[] =
{
    {enableDebug,                       process_enableDebug},  // 16 bits
    {dumpMemory,                         process_dumpMemory},  // 32 bits
    {firmwareVersion,               process_firmwareVersion},  // 16 bits
    {directRealTimeout,           process_directRealTimeout},  // 16 bits
    {directRealHeartbeat,       process_directRealHeartbeat},  // 16 bits
    {statusFullChargeEnergy, process_statusFullChargeEnergy},  // 32 bits
    {statusNorminalEnergy,     process_statusNorminalEnergy},
    {directPower,                       process_directPower},
    {realMode,                             process_realMode},
    {powerBlock,                         process_powerBlock},
    { 0,                                               NULL}
};


int process_dumpMemory (uint16_t index, uint16_t value)
{
    int retval = MODBUS_SUCCESS;
    static uint32_t mem;
    static uint32_t readings;

    //printf("%s - 0x%04X, index = %d \n", __PRETTY_FUNCTION__, value, index);

    //if (debug) {
    if ( index ) {
        mem =  (mem << 16) + __bswap_16(value);
        printf("%s - memory size %d \n", __PRETTY_FUNCTION__, mem);
    }

    return retval;
}

//
// Acks and dismisses alarms
//
int process_enableDebug (uint16_t index, uint16_t value)
{
    int retval = MODBUS_SUCCESS;
    printf("%s - %s\n", __PRETTY_FUNCTION__, (value & 0x0001)?"TRUE":"FALSE");
    debug = value?TRUE:FALSE;
    return retval;
}

//
// report dummy version number
//
int process_firmwareVersion (uint16_t unused, uint16_t count )
{
    uint16_t *address;
    uint16_t address_offset;
    int i, retval = MODBUS_SUCCESS; // need to figure out what this constant is
    const char version[] = "V0.1.3";
    const char *p = version;

    address_offset = mb_mapping->start_registers + firmwareVersion;
    address = mb_mapping->tab_registers + address_offset;
    for ( i = 0; i < count; i++ )
    {
        uint16_t value  =  *p++;
        address[i] = (value << 8) | *p++;
    }

    if (debug) {
        printf("%s Version = %s \n", __PRETTY_FUNCTION__, version);
    }
    return retval;
}

//
// Acks and dismisses alarms
//
int process_realMode (uint16_t index, uint16_t value)
{
    int retval = MODBUS_SUCCESS;
    if (debug) {
       printf("%s\n", __PRETTY_FUNCTION__);
    }
    return retval;
}
//
// Acks and dismisses alarms
//process_directRealTimeout
int process_directRealTimeout (uint16_t unused, uint16_t value)
{
    int retval = MODBUS_SUCCESS;
    heartbeatTimeout = value;
    //if(debug) {
        printf("%s heartbeatTimeout = %d\n", __PRETTY_FUNCTION__, heartbeatTimeout );
    //}
    heartbeat = 0;
    return retval;
}

//
// Heartbeat signal. Expected to toggle heartbeat bit very PGM HB Period
//
int process_directRealHeartbeat (uint16_t index, uint16_t value)
{
    int retval = MODBUS_SUCCESS;
    static uint16_t previous_value = 0;

    if (debug) {
        printf("%s - value:%04x \n", __PRETTY_FUNCTION__, value);
    }

    if ( previous_value == value )
    {
        heartbeat = 0;
    }
    previous_value = ~value;
    return retval;
}


int process_statusFullChargeEnergy(uint16_t index, uint16_t number_register)
{
    uint16_t *address;
    uint16_t address_offset;
    int retval = MODBUS_SUCCESS; // need to figure out what this constant is

    address_offset = mb_mapping->start_registers + statusFullChargeEnergy;
    address = mb_mapping->tab_registers + address_offset;
    if ( address < (mb_mapping->tab_registers + mb_mapping-> nb_registers) )
    {
        *address         = StatusFullChargeEnergy >> 16;
        *(address+1)     = StatusFullChargeEnergy;
    }
    if (debug) {
        printf("%s StatusFullChargeEnergy = %d\n", __PRETTY_FUNCTION__, StatusFullChargeEnergy );
    }
    return retval;
}

int process_statusNorminalEnergy(uint16_t index, uint16_t number_register)
{
    uint16_t *address;
    uint16_t address_offset;
    int retval = MODBUS_SUCCESS;

    address_offset = mb_mapping->start_registers + statusNorminalEnergy;
    address = mb_mapping->tab_registers + address_offset;
    if ( address < (mb_mapping->tab_registers + mb_mapping-> nb_registers) )
    {
        *address         = StatusNorminalEnergy >> 16;
        *(address+1)     = StatusNorminalEnergy;

    }
    if (debug) {
        printf("%s StatusNorminalEnergy = %d\n", __PRETTY_FUNCTION__, StatusNorminalEnergy );
    }

    return retval;
}


//
// Total real power being delivered in kW: range(-32768  to 32767)
//
int process_directPower(uint16_t index, uint16_t value)
{
    int retval = MODBUS_SUCCESS;
    static uint32_t val;

    if ( index == 0 )
    {
        val = value << 16;
    }
    else
    {
        val  += value;                             // store set point value
        if ( val & sign_bit_mask )
        {
            val = ((~val) + 1);                     // get 2nd complement value
            if (debug) {
                printf("%s - battery charging val(-%d)\n", __PRETTY_FUNCTION__, val);
            }
            battery_charging = true;
            battery_discharging = false;
            battery_charge_increment = ( val * battery_charge_resolution);  ;
        }
        else if (val > 0)
        {
            if (debug) {
                printf("%s - battery discharging val(%d)\n", __PRETTY_FUNCTION__, val);
            }
            battery_discharging = true;
            battery_charging = false;
            battery_discharge_decrement = (val * battery_discharge_resolution);
        }
        else
        {
            if (debug) {
                printf("%s - not charging val(%d)\n", __PRETTY_FUNCTION__, val);
            }
            battery_discharging = false;
            battery_charging = false;
        }
    }

    return retval;
}

int process_powerBlock(uint16_t index, uint16_t value)
{
    int retval = MODBUS_SUCCESS;

    printf("%s - value(%d)\n", __PRETTY_FUNCTION__, value);
    //if ( value != POWER_BLOCK_ALL )
    //{
    //    retval = MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE;
    //}
    return retval;
}


int tesla_process_single_register(uint16_t address, uint16_t data)
{
    int retval = MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS;

    printf("%s -address(%d), data(%d)\n", __PRETTY_FUNCTION__, address, data);
    for ( const process_table_t *p = tesla_process_table; p->handler != 0; p++ )
    {
        if ( address == p->address )
        {
        	printf("%s -address found\n", __PRETTY_FUNCTION__);
            retval = p->handler(address, data);
            break;
        }
    }

    return retval;
}


int tesla_write_multiple_addresses(uint16_t start_address, uint16_t quantity, uint8_t* pdata)
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

void tesla_init(init_param_t* param)
{
	printf("%s entry\n", __PRETTY_FUNCTION__ );

    tesla_thread_param_t* tesla_thread_param;
    setvbuf(stdout, NULL, _IONBF, 0);                          // disable stdout buffering
    mb_mapping = param->modbus_mapping;
    terminate1 = FALSE;
    tesla_thread_param = (tesla_thread_param_t*) malloc(sizeof (tesla_thread_param_t));
    tesla_thread_param -> terminate = &terminate1;
    pthread_create( &thread1, NULL, tesla_thread_handler, tesla_thread_param);
	printf("%s exit\n", __PRETTY_FUNCTION__ );
}

void tesla_dispose()
{
	printf("%s entry\n", __PRETTY_FUNCTION__ );
    terminate1 = true;
    pthread_join(thread1, NULL);
	printf("%s exit\n", __PRETTY_FUNCTION__ );
}

//
// Thread handler
//
void *tesla_thread_handler( void *ptr )
{
	printf("%s entry\n", __PRETTY_FUNCTION__ );
    uint8_t *terminate;
    char status[12] = "idle";
    tesla_thread_param_t* param = (tesla_thread_param_t*) ptr;
    terminate = param->terminate;
    free(param);
    modbus_set_debug(ctx, debug);

    while ( *terminate == false )
    {
        sleep(1);
        if ( heartbeat > heartbeatTimeout )
        {
            if ( debug ) {
                printf("%s: heartbeat expired, current timeout = %d\n", __PRETTY_FUNCTION__, heartbeatTimeout );
            }
            heartbeat = 0;
        }

        if (battery_charging)
        {
            if ( debug ) {
                strcpy(status,"charging");
            }
            if ( (state_of_charge + battery_charge_increment) <= battery_fully_charged )
            {
                state_of_charge += battery_charge_increment;
            }
            else
            {
                state_of_charge = battery_fully_charged;
                battery_charging = false;
            }
        }
        else if (battery_discharging)
        {
            if ( debug ) {
                strcpy(status,"discharging");
            }
            if ( (state_of_charge - battery_discharge_decrement) >= battery_fully_discharged )
            {
                state_of_charge -= battery_discharge_decrement;
            }
            else
            {
                state_of_charge = battery_fully_discharged;
                battery_discharging = false;
            }
        }
        else
        {
            if ( debug ) {
                strcpy(status,"idle");
            }
        }
        //update_json_file(state_of_charge, (const char*)status);
        heartbeat++;
    }
	printf("%s exit\n", __PRETTY_FUNCTION__ );
}

