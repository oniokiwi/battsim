#include <stdio.h>
#include <stdlib.h>
#include "nec.h"
#include "typedefs.h"
#include <unistd.h>
#include <signal.h>
#include <error.h>
#include <errno.h>
#include <modbus/modbus.h>
#include <string.h>
#include <time.h>
#include <stdbool.h>
#include <pthread.h>

#define BATTERY_POWER_RATING            230           // kW
#define TIME_CHARGE_FROM_0_TO_100       3000          // seconds
#define TIME_DISCHARGE_FROM_100_TO_0    2800          // seconds

// Private data
static modbus_t* ctx;
static modbus_mapping_t *mb_mapping;
static uint16_t debug = 0;
static init_param_t *param;

static pthread_t thread1;
static uint8_t terminate1;

static uint16_t averagesoc_multiplier = 10;
static float battery_discharge_decrement = 0.0;
static float battery_charge_increment = 0.0;
static bool battery_charging = false;
static bool battery_discharging = false;
static uint16_t heartbeat = 0;
static uint16_t dispatch_mode_enable = 0;
static uint16_t real_power_output = 0;
static float state_of_charge;

static const uint16_t sign_bit_mask             = 0x8000;
static const int HeartbeatFromPGMask            = 1;
static const int HeartBeatIntervalInSeconds     = 5;
static const float state_of_charge_default      = 50.00;
static const float battery_charge_resolution    = 100.00 / (BATTERY_POWER_RATING * TIME_CHARGE_FROM_0_TO_100);     // % increase in charge per sec
static const float battery_discharge_resolution = 100.00 / (BATTERY_POWER_RATING * TIME_DISCHARGE_FROM_100_TO_0);  // % decrease in charge per sec
static const float battery_fully_charged        = 100.00;
static const float battery_fully_discharged     = 0.0;

// private functions
static int _enableDebugTrace(uint16_t);
static int _ackalarams (uint16_t);
static int _averagesoc();
static int _dispatchmode(uint16_t);
static int _HeartbeatFromPGM(uint16_t);
static int _modecontrol(uint16_t);
static int _powerblockenablecontrol12H (uint16_t);
static int _powerblockenablecontrol12L(uint16_t);
static int _realpoweroutput();
static int _ReactivePowerSetPoint(uint16_t);
static int _RealPowerSetPoint(uint16_t);
static int _SocRef(uint16_t);
static int _pslewrate(uint16_t);
static int _qslewrate(uint16_t);

const char* OperatingModecontrolName(uint16_t val);
//
// Lookup table for process functions
//
int nec_process_single_register(uint16_t address, uint16_t data)
{
    int retval;

    switch ( address )
    {
        case enableDebugTrace: // Not a modbus register
            retval = _enableDebugTrace(data);
            break;

        case realpoweroutput:
            retval = _realpoweroutput();
            break;

        case averagesoc:
            retval = _averagesoc();
            break;

        case RealPowerSetPoint:
            retval = _RealPowerSetPoint(data);
            break;

        case ReactivePowerSetPoint:
            retval = _ReactivePowerSetPoint(data);
            break;

        case modecontrol:
            retval = _modecontrol(data);
            break;

        case powerblockenablecontrol12H:
            retval = _powerblockenablecontrol12H(data);
            break;

        case powerblockenablecontrol12L:
            retval = _powerblockenablecontrol12L(data);
            break;

        case HeartbeatFromPGM:
            retval = _HeartbeatFromPGM(data);
            break;

        case SocRef:
            retval = _SocRef(data);
            break;

        case dispatchmode:
            retval = _dispatchmode(data);
            break;

        case ackalarams:
            retval = _ackalarams(data);
            break;

        case pslewrate:
            retval = _pslewrate(data);
            break;

        case qslewrate:
            _qslewrate(data);
            break;

        default:
            retval = MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
            break;

    }
    return retval;
}


int _enableDebugTrace(uint16_t value)
{
    debug =  value & 0x0001;
    uint16_t *address;
    uint16_t address_offset;

    printf("%s - %s\n", __PRETTY_FUNCTION__, debug?"TRUE":"FALSE");
    address_offset = mb_mapping->start_registers + enableDebugTrace;
    address = mb_mapping->tab_registers + address_offset;
    if ( address < (mb_mapping->tab_registers + mb_mapping-> nb_registers) )
    {
       *address = debug;
    }
    return MODBUS_SUCCESS;
}

//
// Acks and dismisses alarms
//
int _ackalarams (uint16_t value)
{
    int retval = MODBUS_SUCCESS;
    if (debug) printf("%s\n", __PRETTY_FUNCTION__);
    //alarm(val);
    return retval;
}

//
// Average SOC currently online
//
int _averagesoc()
{
    uint16_t *address;
    uint16_t address_offset;
    int retval = MODBUS_SUCCESS; // need to figure out what this constant is

    address_offset = mb_mapping->start_registers + averagesoc;
    address = mb_mapping->tab_registers + address_offset;
    if ( address < (mb_mapping->tab_registers + mb_mapping-> nb_registers) )
    {
        *address = state_of_charge * averagesoc_multiplier;
    }
    if (debug) printf("%s - soc(%d) \n", __PRETTY_FUNCTION__, *address );
    return retval;
}


//
// Enable dispatch mode.
//     0: idle
//     1: dispatch
//
int _dispatchmode(uint16_t value)
{
    int retval = MODBUS_SUCCESS;
    if (debug) printf("%s val(%d)\n", __PRETTY_FUNCTION__, value );

    switch (value)
    {
    case DispatchModeIdle:
    case DispatchModeDispatch:
        dispatch_mode_enable = value;
        break;

    default:
        retval = MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE;
        break;
    }
    return retval;
}

//
// Heartbeat signal. Expected to toggle heartbeat bit very PGM HB Period
//
int _HeartbeatFromPGM (uint16_t value)
{
    static uint16_t toggle = 0;
    uint16_t val = value;

    if (debug) printf("%s val(%d)\n", __PRETTY_FUNCTION__, value );
    val &= HeartbeatFromPGMask;   // mask off unwanted bits
    if ( toggle ^ val )
    {
        toggle = val;
        heartbeat = 0;
    }
    return MODBUS_SUCCESS;
}

//
// Operating mode of GBS.
//     0: Shutdown
//     4: Manual
//    32: Operational
//
int _modecontrol(uint16_t value)
{
    int retval = MODBUS_SUCCESS; // need to figure out what this constant is
    if (debug) printf("%s %s, value(%d)\n", __PRETTY_FUNCTION__, OperatingModecontrolName(value), value);

    switch (value)
    {
    case OperatingModeShutDown:
    case OperatingModeManual:
    case OperatingModeOperational:
        break;

    default:
        retval = MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE;
        break;
    }
    return retval;
}

//
// Controls which power blocks will be active for the Control Group
//
int _powerblockenablecontrol12H (uint16_t value)
{
    int retval = MODBUS_SUCCESS;

    if (debug) printf("%s val(%d)\n", __PRETTY_FUNCTION__, value );

    return retval;
}


//
// Controls which power blocks will be active for the Control Group
//
int _powerblockenablecontrol12L (uint16_t value)
{
    int retval = MODBUS_SUCCESS;

    if (debug) printf("%s val(%d)\n", __PRETTY_FUNCTION__, value );

    return retval;
}



//
// Total real power being delivered in kW: range(-32768  to 32767)
//
int _realpoweroutput()
{
    int retval = MODBUS_SUCCESS;
    int val;
    uint16_t *address;
    uint16_t address_offset;

    if (debug) printf("%s \n", __PRETTY_FUNCTION__ );

    if (battery_charging)
    {
          val = (int) (state_of_charge == battery_fully_charged) ? 0 : real_power_output;
    }
    else if (battery_discharging)
    {
        val = (int) (state_of_charge == battery_fully_discharged) ? 0 : real_power_output;
    }
    else
    {
        val = real_power_output;
    }
    address_offset = mb_mapping->start_registers + realpoweroutput;
    address = mb_mapping->tab_registers + address_offset;
    if ( address < (mb_mapping->tab_registers + mb_mapping-> nb_registers) )
    {
        *address = val;
    }

    return retval;
}


//
// Real power command in kW: range(-32768  to 32767)
//
int _RealPowerSetPoint(uint16_t value)
{

    int retval = MODBUS_SUCCESS;
    uint16_t val = value;

    real_power_output  = val;                             // store set point value
    if ( val & sign_bit_mask )
    {
        val = ((~val) + 1);                            // get 2nd complement value
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
    return retval;
}


//
// Reactive power command in kW: range(-32768  to 32767)
//
int _ReactivePowerSetPoint(uint16_t value)
{
    int retval = MODBUS_SUCCESS;

    if (debug) printf("%s value %d\n", __PRETTY_FUNCTION__, value);

    return retval;
}

//
// pslewrate
//
int _pslewrate(uint16_t value)
{
    int retval = MODBUS_SUCCESS;

    if (debug) printf("%s val(%d)\n", __PRETTY_FUNCTION__, value );

    return retval;
}

//
// qslewrate
//
int _qslewrate(uint16_t value)
{
    int retval = MODBUS_SUCCESS;

    if (debug) printf("%s val(%d)\n", __PRETTY_FUNCTION__, value );
    return retval;
}


//
// Returns the operating mode as a string
//
const char* OperatingModecontrolName(uint16_t val)
{
    switch (val)
    {
    case OperatingModeShutDown:
        return "OperatingModeShutDown";

    case OperatingModeManual:
        return "OperatingModeManual";

    case OperatingModeOperational:
        return "OperatingModeOperational";
    }
    return "unknown operating mode";
}

//
// Acks and dismisses alarms
//
int _SocRef (uint16_t value)
{
    int retval = MODBUS_SUCCESS;
    if (debug) printf("%s val(%d)\n", __PRETTY_FUNCTION__, value );
    return retval;
}


void nec_init(init_param_t* init_param)
{
    thread_param_t* nec_thread_param;
    nec_disconnect();                                           // set default SoC
    setvbuf(stdout, NULL, _IONBF, 0);                          // disable stdout buffering
    mb_mapping = init_param->modbus_mapping;
    terminate1 = FALSE;
    nec_thread_param = (thread_param_t*) malloc(sizeof (thread_param_t));
    nec_thread_param -> terminate = &terminate1;
    pthread_create( &thread1, NULL, nec_thread_handler, nec_thread_param);
}

void nec_dispose()
{
    terminate1 = true;
    pthread_join(thread1, NULL);
}

void nec_disconnect()
{
    state_of_charge = state_of_charge_default;
}


int nec_write_multiple_addresses(uint16_t start_address, uint16_t quantity, uint8_t* pdata)
{
    uint16_t *address;
    uint16_t address_offset;
    uint16_t i, data;

    address_offset = mb_mapping->start_registers + start_address;
    address = mb_mapping->tab_registers + address_offset;
    for ( i = 0; i < quantity; i++ )
    {
        if ( address < (mb_mapping->tab_registers + mb_mapping-> nb_registers) )
        {
            data = (*pdata++ << 8) | *pdata++;
            *address++  = data;
            nec_process_single_register(start_address + i, data);
        }
    }
    return MODBUS_SUCCESS;
}

//
// Thread handler
//
void *nec_thread_handler( void *ptr )
{
    uint8_t *terminate;
    thread_param_t* param = (thread_param_t*) ptr;
    ctx = param->ctx;
    terminate = param->terminate;
    free(param);

    while ( *terminate == false )
    {
        sleep(1);
        if ( heartbeat > HeartBeatIntervalInSeconds )
        {
            heartbeat = 0;
            if (debug) printf("heartbeat not received\n");
        }
        if ( dispatch_mode_enable == DispatchModeDispatch )
        {
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
        }
        heartbeat++;
    }
}

