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

// proclet
static int _enableDebugTrace (uint16_t );
static int _dumpMemory (uint16_t, uint16_t );
static int _firmwareVersion (uint16_t );
static int _directRealTimeout (uint16_t );
static int _directRealHeartbeat(uint16_t );
static int _statusFullChargeEnergy();
static int _statusNorminalEnergy ();
static int _directPower( uint16_t, uint16_t  );
static int _realMode(uint16_t  );
static int _powerBlock(uint16_t);

int tesla_process_single_register(uint16_t address, uint16_t data)
{
    int retval;

    switch ( address )
    {
        case enableDebugTrace: // Not a modbus register
            retval = _enableDebugTrace(data);
            break;

        case firmwareVersion:
            retval = _firmwareVersion(data);
            break;

        case directRealTimeout:
            retval = _directRealTimeout(data);
            break;

        case directRealHeartbeat:
            retval = _directRealHeartbeat(data);
            break;

        case statusFullChargeEnergy:
            retval = _statusFullChargeEnergy();
            break;

        case statusNorminalEnergy:
            retval = _statusNorminalEnergy();
            break;

        case directPower:
            retval = _realMode(data);
            break;

        case realMode:
            retval = _realMode(data);
            break;

        case powerBlock:
            retval = _powerBlock(data);
            break;

        default:
            retval = MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
            break;

    }
    return retval;
}


//
// Acks and dismisses alarms
//
int _enableDebugTrace (uint16_t value)
{
    debug =  value & 0x0001;
    uint16_t *address;
    uint16_t address_offset;

    if (debug) printf("%s - %s\n", __PRETTY_FUNCTION__, debug?"TRUE":"FALSE");
    address_offset = mb_mapping->start_registers + enableDebugTrace;
    address = mb_mapping->tab_registers + address_offset;
    if ( address < (mb_mapping->tab_registers + mb_mapping-> nb_registers) )
    {
       *address = debug;
    }
    return MODBUS_SUCCESS;
}

//
// report dummy version number
//
int _firmwareVersion (uint16_t count)
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

    if (debug) printf("%s Version = %s \n", __PRETTY_FUNCTION__, version);

    return retval;
}

int _realMode (uint16_t value)
{
    if (debug) printf("%s\n", __PRETTY_FUNCTION__);

    return MODBUS_SUCCESS;
}


int _directRealTimeout (uint16_t value)
{
    int retval = MODBUS_SUCCESS;
    heartbeatTimeout = value;
    if(debug) printf("%s heartbeatTimeout = %d\n", __PRETTY_FUNCTION__, heartbeatTimeout );
    heartbeat = 0;
    return retval;
}

//
// Heartbeat signal. Expected to toggle heartbeat bit very PGM HB Period
//
int _directRealHeartbeat (uint16_t value)
{
    int retval = MODBUS_SUCCESS;
    static uint16_t previous_value = 0;

    if (debug) printf("%s - value:%04x \n", __PRETTY_FUNCTION__, value);

    if ( previous_value == value )
    {
        heartbeat = 0;
    }
    previous_value = ~value;
    return retval;
}


int _statusFullChargeEnergy()
{
    uint16_t *address;
    uint16_t address_offset;

    address_offset = mb_mapping->start_registers + statusFullChargeEnergy;
    address = mb_mapping->tab_registers + address_offset;
    if ( address < (mb_mapping->tab_registers + mb_mapping-> nb_registers) )
    {
        *address         = StatusFullChargeEnergy >> 16;
        *(address+1)     = StatusFullChargeEnergy;
    }
    if (debug) printf("%s StatusFullChargeEnergy = %d\n", __PRETTY_FUNCTION__, StatusFullChargeEnergy );

    return MODBUS_SUCCESS;
}

int _statusNorminalEnergy()
{
    uint16_t *address;
    uint16_t address_offset;

    address_offset = mb_mapping->start_registers + statusNorminalEnergy;
    address = mb_mapping->tab_registers + address_offset;
    if ( address < (mb_mapping->tab_registers + mb_mapping-> nb_registers) )
    {
        *address         = StatusNorminalEnergy >> 16;
        *(address+1)     = StatusNorminalEnergy;

    }
    if (debug) printf("%s StatusNorminalEnergy = %d\n", __PRETTY_FUNCTION__, StatusNorminalEnergy );

    return MODBUS_SUCCESS;
}


//
// Total real power being delivered in kW: range(-32768  to 32767)
//
int _directPower(uint16_t index, uint16_t value)
{
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
            if (debug) printf("%s - battery charging val(-%d)\n", __PRETTY_FUNCTION__, val);
            battery_charging = true;
            battery_discharging = false;
            battery_charge_increment = ( val * battery_charge_resolution);  ;
        }
        else if (val > 0)
        {
            if (debug) printf("%s - battery discharging val(%d)\n", __PRETTY_FUNCTION__, val);
            battery_discharging = true;
            battery_charging = false;
            battery_discharge_decrement = (val * battery_discharge_resolution);
        }
        else
        {
            if (debug) printf("%s - not charging val(%d)\n", __PRETTY_FUNCTION__, val);
            battery_discharging = false;
            battery_charging = false;
        }
    }

    return MODBUS_SUCCESS;
}

int _powerBlock(uint16_t value)
{
    if (debug) printf("%s - value(%d)\n", __PRETTY_FUNCTION__, value);

    return MODBUS_SUCCESS;
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
    tesla_thread_param_t* tesla_thread_param;
    setvbuf(stdout, NULL, _IONBF, 0);                          // disable stdout buffering
    mb_mapping = param->modbus_mapping;
    terminate1 = FALSE;
    tesla_thread_param = (tesla_thread_param_t*) malloc(sizeof (tesla_thread_param_t));
    tesla_thread_param -> terminate = &terminate1;
    pthread_create( &thread1, NULL, tesla_thread_handler, tesla_thread_param);
}

void tesla_dispose()
{
    terminate1 = true;
    pthread_join(thread1, NULL);
}

//
// Thread handler
//
void *tesla_thread_handler( void *ptr )
{
    uint8_t *terminate;
    tesla_thread_param_t* param = (tesla_thread_param_t*) ptr;
    terminate = param->terminate;
    free(param);

    while ( *terminate == false )
    {
        sleep(1);
        if ( heartbeat > heartbeatTimeout )
        {
            if ( debug ) printf("%s: heartbeat expired, current timeout = %d\n", __PRETTY_FUNCTION__, heartbeatTimeout );
            heartbeat = 0;
        }

        if (battery_charging)
        {
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
        heartbeat++;
    }
}

