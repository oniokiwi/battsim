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
static bool debug = false;
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
static const int ByteToWordConversionValue      = 256;
static const int FunctionCodeIndex              = 7;
static const int ReadWriteOperation             = 0x17;
static const int msbDataIndex                   = 0x00;
static const int lsbDataIndex                   = 0x01;

// private functions
//static int _enableDebugTrace(uint16_t, uint16_t);
static int _ackalarams (uint16_t, uint16_t);
static int _averagesoc(uint16_t, uint16_t);
static int _dispatchmode(uint16_t, uint16_t);
static int _HeartbeatFromPGM (uint16_t, uint16_t);
static int _modecontrol(uint16_t, uint16_t);
static int _powerblockenablecontrol12H (uint16_t, uint16_t);
static int _powerblockenablecontrol12L(uint16_t, uint16_t);
static int _realpoweroutput(uint16_t, uint16_t);
static int _ReactivePowerSetPoint(uint16_t, uint16_t);
static int _RealPowerSetPoint(uint16_t, uint16_t);
static int _SocRef(uint16_t, uint16_t);
static int _pslewrate(uint16_t, uint16_t);
static int _qslewrate(uint16_t, uint16_t);

const char* OperatingModecontrolName(uint16_t val);
//
// Lookup table for process functions
//
const process_table_t nec_process_table[] =
{
	//{enableDebugTrace,                      _enableDebugTrace},
    {realpoweroutput,                        _realpoweroutput},
    {averagesoc,                                  _averagesoc},
    {RealPowerSetPoint,                    _RealPowerSetPoint},
    {ReactivePowerSetPoint,            _ReactivePowerSetPoint},
    {modecontrol,                                _modecontrol},
    {powerblockenablecontrol12H,  _powerblockenablecontrol12H},
    {powerblockenablecontrol12L,  _powerblockenablecontrol12L},
    {HeartbeatFromPGM,                      _HeartbeatFromPGM},
    {SocRef,                                          _SocRef},
    {dispatchmode,                              _dispatchmode},
    {ackalarams,                                  _ackalarams},
    {pslewrate,                                    _pslewrate},
    {qslewrate,                                    _qslewrate},
    {0,                                                  NULL}
};


//int _enableDebugTrace(uint16_t index, uint16_t value)
//{
//	bool val =  value & 0x0001;
//	printf("%s - %s\n", __PRETTY_FUNCTION__, val?"TRUE":"FALSE");
//	modbus_set_debug(param->ctx, val);
//}
//
// Acks and dismisses alarms
//
int _ackalarams (uint16_t index, uint16_t value)
{
    int retval = MODBUS_SUCCESS;
    printf("%s\n", __PRETTY_FUNCTION__);
    //alarm(val);
    return retval;
}

//
// Average SOC currently online
//
int _averagesoc(uint16_t index, uint16_t value)
{
    uint16_t *address;
    uint16_t address_offset;
    int retval = MODBUS_SUCCESS; // need to figure out what this constant is

    printf("%s \n", __PRETTY_FUNCTION__ );
    address_offset = mb_mapping->start_registers + averagesoc;
    address = mb_mapping->tab_registers + address_offset;
    if ( address < (mb_mapping->tab_registers + mb_mapping-> nb_registers) )
    {
        *address = state_of_charge * averagesoc_multiplier;
    }

    return retval;
}


//
// Enable dispatch mode.
//     0: idle
//     1: dispatch
//
int _dispatchmode(uint16_t index, uint16_t value)
{
    int retval = MODBUS_SUCCESS;
    printf("%s val(%d)\n", __PRETTY_FUNCTION__, value );

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
int _HeartbeatFromPGM (uint16_t index, uint16_t value)
{
    int retval = MODBUS_SUCCESS;
    printf("%s val(%d)\n", __PRETTY_FUNCTION__, value );
    static int toggle = 0;
    uint16_t val = value;

    val &= HeartbeatFromPGMask;   // mask off unwanted bits
    if ( toggle ^ val )
    {
        toggle = val;
        heartbeat = 0;
    }
    return retval;
}


//
// Operating mode of GBS.
//     0: Shutdown
//     4: Manual
//    32: Operational
//
int _modecontrol(uint16_t index, uint16_t value)
{
    int retval = MODBUS_SUCCESS; // need to figure out what this constant is
    printf("%s %s, value(%d)\n", __PRETTY_FUNCTION__, OperatingModecontrolName(value), value);

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
int _powerblockenablecontrol12H (uint16_t index, uint16_t value)
{
    int retval = MODBUS_SUCCESS;

    printf("%s val(%d)\n", __PRETTY_FUNCTION__, value );

    return retval;
}


//
// Controls which power blocks will be active for the Control Group
//
int _powerblockenablecontrol12L (uint16_t index, uint16_t value)
{
    int retval = MODBUS_SUCCESS;

    printf("%s val(%d)\n", __PRETTY_FUNCTION__, value );

    return retval;
}



//
// Total real power being delivered in kW: range(-32768  to 32767)
//
int _realpoweroutput(uint16_t index, uint16_t value)
{
    int retval = MODBUS_SUCCESS;
    int val;
    uint16_t *address;
    uint16_t address_offset;

    printf("%s \n", __PRETTY_FUNCTION__ );

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
int _RealPowerSetPoint(uint16_t index, uint16_t value)
{

    int retval = MODBUS_SUCCESS;
    uint16_t val = value;

    real_power_output  = val;                             // store set point value
    if ( val & sign_bit_mask )
    {
        val = ((~val) + 1);                            // get 2nd complement value
        printf("%s - battery charging val(-%d)\n", __PRETTY_FUNCTION__, val);
        battery_charging = true;
        battery_discharging = false;
        battery_charge_increment = ( val * battery_charge_resolution);  ;
    }
    else if (val > 0)
    {
        printf("%s - battery discharging val(%d)\n", __PRETTY_FUNCTION__, val);
        battery_discharging = true;
        battery_charging = false;
        battery_discharge_decrement = (val * battery_discharge_resolution);
    }
    else
    {
        printf("%s - not charging val(%d)\n", __PRETTY_FUNCTION__, val);
        battery_discharging = false;
        battery_charging = false;
    }
    return retval;
}


//
// Reactive power command in kW: range(-32768  to 32767)
//
int _ReactivePowerSetPoint(uint16_t index, uint16_t value)
{
    int retval = MODBUS_SUCCESS;

    printf("%s value %d\n", __PRETTY_FUNCTION__, value);

    return retval;
}

//
// pslewrate
//
int _pslewrate(uint16_t index, uint16_t value)
{
    int retval = MODBUS_SUCCESS;

    printf("%s val(%d)\n", __PRETTY_FUNCTION__, value );

    return retval;
}

//
// qslewrate
//
int _qslewrate(uint16_t index, uint16_t value)
{
    int retval = MODBUS_SUCCESS;

    printf("%s val(%d)\n", __PRETTY_FUNCTION__, value );
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
int _SocRef (uint16_t index, uint16_t value)
{
    int retval = MODBUS_SUCCESS;
    printf("%s val(%d)\n", __PRETTY_FUNCTION__, value );
    return retval;
}


void nec_init(init_param_t* init_param)
{
    printf("%s entry\n", __PRETTY_FUNCTION__ );

    thread_param_t* nec_thread_param;
    setvbuf(stdout, NULL, _IONBF, 0);                          // disable stdout buffering
    mb_mapping = init_param->modbus_mapping;
    terminate1 = FALSE;
    nec_thread_param = (thread_param_t*) malloc(sizeof (thread_param_t));
    nec_thread_param -> terminate = &terminate1;
    pthread_create( &thread1, NULL, nec_thread_handler, nec_thread_param);
    param = init_param;
    printf("%s exit\n", __PRETTY_FUNCTION__ );
}

void nec_dispose()
{
    printf("%s entry\n", __PRETTY_FUNCTION__ );
    terminate1 = true;
    pthread_join(thread1, NULL);
    printf("%s exit\n", __PRETTY_FUNCTION__ );
}


int nec_process_single_register(uint16_t address, uint16_t data)
{
    int retval = MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS;

    printf("%s -address(%d), data(%d)\n", __PRETTY_FUNCTION__, address, data);
    for ( const process_table_t *p = nec_process_table; p->handler != 0; p++ )
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

int nec_write_multiple_addresses(uint16_t start_address, uint16_t quantity, uint8_t* pdata)
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

//
// Thread handler
//
void *nec_thread_handler( void *ptr )
{
    char *terminate;
    char status[12] = "idle";
    printf("nec_thread_handler\n");

    thread_param_t* param = (thread_param_t*) ptr;
    ctx = param->ctx;
    terminate = param->terminate;
    free(param);
    state_of_charge = state_of_charge_default; // set to default state of charge value

    while ( *terminate == false )
    {
        sleep(1);
        if ( heartbeat > HeartBeatIntervalInSeconds )
        {
            heartbeat = 0;
            printf("heatbeat not received\n");
        }
        if ( dispatch_mode_enable == DispatchModeDispatch )
        {
            if (battery_charging)
            {
                strcpy(status,"charging");
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
                strcpy(status,"discharging");
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
                strcpy(status,"idle");
            }
        }
        heartbeat++;
    }
    //printf("exiting thread handler\n");
}

