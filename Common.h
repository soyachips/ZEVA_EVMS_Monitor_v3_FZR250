// Common header file for EVMS3, BMS12i, BMS16, Monitor
// Has defines, enums, variables and utility functions common to EVMS and Monitor.
// Ideally just keep one copy of this file somewhere and link to it from each separate project

// Open Source version, released under MIT License (see Readme file)
// Last modified by Ian Hooper (ZEVA), August 2021

#define CAN_BAUD        250
#define USE_29BIT_IDS	1 // Or 11-bit IDs if set to 0

#define CAN_BASE_ID		30

#define PACK_CAPACITY_MULTIPLIER	5 // Ah steps in settings, also affects max capacity, normally 5
#define PACK_VOLTAGE_MULTIPLIER		1 // 1 for normal range of 0-400V systems, 2 for double range up to 800V or so
#define BALANCE_TOLERANCE	10 // i.e shunt if this many millivolts above average

#define EEPROM_OFFSET	4
enum { EEPROM_BLANK, EEPROM_CORRUPT, EEPROM_CORRECT };
#define EEPROM_DISPLAY_BRIGHTNESS	120

// CAN PACKET IDs
enum { CORE_BROADCAST_STATUS = CAN_BASE_ID,
	CORE_SET_STATE,
	CORE_RECEIVE_CONFIG1, 
	CORE_RECEIVE_CONFIG2, 
	CORE_RECEIVE_CONFIG3, 
	CORE_RECEIVE_CONFIG4, 
	CORE_RECEIVE_CELL_NUMS, 
	CORE_ACKNOWLEDGE_ERROR, 
	CORE_RESET_SOC,
    POWER_OFF };

#define CAN_CURRENT_SENSOR_ID	40
#define CAN_ZERO_CURRENT		41
#define CAN_EVSE_INTERFACE		45

enum { CORE_REQUEST_CONFIG = 51,
	CORE_SEND_CONFIG1,
	CORE_SEND_CONFIG2,
	CORE_SEND_CONFIG3,
	CORE_SEND_CONFIG4,
	CORE_SEND_CELL_NUMS };

#define BMS_BASE_ID	300
enum { BMS_REQUEST_DATA, BMS_REPLY1, BMS_REPLY2, BMS_REPLY3, BMS_REPLY4 };

// Charger stuff
#define TC_CHARGER1_RX_ID	0x1806E5F4
#define TC_CHARGER1_TX_ID	0x18FF50E5
#define TC_CHARGER2_RX_ID	0x1806E7F4
#define TC_CHARGER2_TX_ID	0x18FF50E7
#define TC_CHARGER3_RX_ID	0x1806E8F4
#define TC_CHARGER3_TX_ID	0x18FF50E8

// Motor Controller stuff
enum { NO_MC, MC600C, MC1000C };
enum { MC_STATUS_PACKET_ID = 50, MC_SET_THROTTLE_ID, MC_RECEIVE_SETTINGS_ID, MC_SEND_SETTINGS_ID };

enum { // MC status / errors
	MC_NO_ERROR,
	MC_SLEEPING,
	MC_DESAT_ERROR,
	MC_CURRENT_SENSOR_FAULT,
	MC_TEMP_SENSOR_FAULT,
	MC_UNDER_VOLTAGE,
	MC_OVER_VOLTAGE,
	MC_LOW_LOGIC_VOLTAGE,
	MC_THROTTLE_ERROR,
	MC_THERMAL_CUTBACK_ERROR,
	MC_THERMAL_SHUTDOWN_ERROR,
	MC_NUM_ERRORS };

enum { // MC settings
	MC_MIN_BATT_VOLTAGE,
	MC_MAX_MOTOR_VOLTAGE,
	MC_MAX_MOTOR_CURRENT,
	MC_MAX_BATT_CURRENT,
	MC_RAMP_RATE,
	MC_SPEED_CONTROL_TYPE,
	MC_TORQUE_CONTROL_TYPE,
	MC_THROTTLE_TYPE,
	MC_IDLE_VOLTAGE,
	MC_IDLE_CURRENT,
	MC_NUM_SETTINGS };
unsigned char mcSettings[MC_NUM_SETTINGS];
unsigned char mcMinimums[MC_NUM_SETTINGS] = { 8, 1, 5, 5, 0, 0, 0, 1, 0, 0 };
unsigned char mcMaximums[MC_NUM_SETTINGS] = { 150, 180, 100, 100, 4, 3, 3, 3, 12, 20 };

#ifdef MONITOR
	char* mcErrors[MC_NUM_ERRORS] = {
		"     Status: OK     ",
		"  Status: Sleeping  ",
		"     Desat fault     ",
		"Current sensor fault",
		"  Temp sensor fault  ", 
		"    Undervoltage    ",
		"     Overvoltage     ",
		"   Low 12v supply   ",
		"   Throttle error   ",
		"   Thermal cutback   ",
		"  Thermal shutdown  " };

	char* mcUnits[MC_NUM_SETTINGS] = { "V", "V", "A", "A", "", "", "", "", "V", "A" };
	char* mcNames[MC_NUM_SETTINGS] = {
		" Min Batt Volts ",
		" Max Motor Volts ",
		"Max Motor Current",
		" Max Batt Current ",
		" Thrtl Ramp Rate ",
		" Speed Control ",
		" Torque Control ",
		" Throttle Type ",
		" Idle Voltage ",
		" Idle Current " };

	char* mcThrottleTypes[4] = { "", "     0-5V     ", "   0-5kohm   ", "     HEPA     " };
	char* mcControlTypes[4] = { "    Linear    ", "Semiquadratic", "  Quadratic  ", "   Off   " };
#endif


enum { IDLE, PRECHARGING, RUNNING, CHARGING, STOPPED, SETUP };
char state;

enum { CORE_IDLE_STATE, CORE_SETUP_STATE, CORE_SETUP_EDIT_FUEL_GAUGE, CORE_SETUP_EDIT_TEMP_GAUGE };
#ifdef MONITOR
	const char cs1[] PROGMEM = "Idle";
	const char cs2[] PROGMEM = "Precharging";
	const char cs3[] PROGMEM = "Running";
	const char cs4[] PROGMEM = "Charging";
	const char cs5[] PROGMEM = "Stopped";
	const char cs6[] PROGMEM = "Setup";
	PROGMEM const char* const coreStatuses[] = { cs1, cs2, cs3, cs4, cs5, cs6 };
#endif

short ticksInCurrentState = 0;

enum { NO_ERROR, 
	CORRUPT_EEPROM_ERROR,
	OVERCURRENT_WARNING, 
	OVERCURRENT_SHUTDOWN, 
	BMS_LOW_WARNING, 
	SHUTDOWN_BY_BMS_ERROR, 
	BMS_HIGH_WARNING,
	BMS_ENDED_CHARGE_ERROR, 
	BMS_OVERTEMP, 
	BMS_UNDERTEMP, 
	LOW_SOC_ERROR, 
	OVERTEMP_ERROR, 
	ISOLATION_ERROR, 
	LOW_12V_ERROR, 
	PRECHARGE_FAILED_ERROR, 
	CONTACTOR_SW_FAULT,
	BMS_COMMS_ERROR, 
	CORE_COMMS_ERROR,
	NUM_ERRORS };

#ifdef MONITOR
	const char e0[] PROGMEM = "No error";
	const char e1[] PROGMEM = "Corrupt settings";
	const char e2[] PROGMEM = "Overcurrent warning";
	const char e3[] PROGMEM = "Overcurrent shutdown";
	const char e4[] PROGMEM = "BMS - Low cell";
	const char e5[] PROGMEM = "Shutdown by BMS";
	const char e6[] PROGMEM = "BMS - High cell";
	const char e7[] PROGMEM = "Charge ended by BMS";
	const char e8[] PROGMEM = "BMS - Overtemp";
	const char e9[] PROGMEM = "BMS - Undertemp";
	const char e10[] PROGMEM = "Low battery charge";
	const char e11[] PROGMEM = "Over-temperature";
	const char e12[] PROGMEM = "Isolation fault";
	const char e13[] PROGMEM = "Low 12V battery";
	const char e14[] PROGMEM = "Precharge failed";
	const char e15[] PROGMEM = "Contactor fault";
	const char e16[] PROGMEM = "BMS - Comms error";
	const char e17[]	PROGMEM = "No comms to EVMS";
	PROGMEM const char* const errorStrings[] = { e0,e1,e2,e3,e4,e5,e6,e7,e8,e9,e10,e11,e12,e13,e14,e15,e16,e17 };
#endif

enum { // MPI functions
	MPI_WAKE_UP = 0,
	MPI_DUAL_CHARGE_RATE,
	MPI_HEADLIGHT_SENSE,
	MPI_CTR_AUX_SWITCH,
	MPI_NUM_FUNCTIONS };

#ifdef MONITOR
	const char mpi1[] PROGMEM = "Wake up";
	const char mpi2[] PROGMEM = "Alt charge";
	const char mpi3[] PROGMEM = "Hdlight in";
	const char mpi4[] PROGMEM = "Ctr aux sw";
	PROGMEM const char* const mpiStrings[] = { mpi1, mpi2, mpi3, mpi4 };
#endif

enum { // MPO functions
	MPO_GROUND,
	MPO_TEMP_GAUGE,
	MPO_LOW_SOC_SIGNAL,
	MPO_OVERTEMP_SIGNAL,
	MPO_UNDERTEMP_SIGNAL,
	MPO_ERROR_BUZZER,
	MPO_STATUS_LIGHT,
	MPO_NUM_FUNCTIONS };

#ifdef MONITOR
	const char mpo1[] PROGMEM = "Ground";
	const char mpo2[] PROGMEM = "Temp gauge";
	const char mpo3[] PROGMEM = "Low SoC";
	const char mpo4[] PROGMEM = "Overtemp";
	const char mpo5[] PROGMEM = "Undertemp";
	const char mpo6[] PROGMEM = "Error";
	const char mpo7[] PROGMEM = "Status";
	PROGMEM const char* const mpoStrings[] = { mpo1, mpo2, mpo3, mpo4, mpo5, mpo6, mpo7 };
#endif

enum { SOC_PERCENT, SOC_AMPHOURS };

enum { // Settings
	PACK_CAPACITY,
	SOC_WARNING, 
	FULL_VOLTAGE, 
	CURRENT_WARNING, 
	CURRENT_TRIP, 
	OVER_TEMP,
	MIN_AUX_VOLTAGE, 
	MIN_ISOLATION,
	TACHO_PPR, 
	FUEL_GAUGE_FULL, 
	FUEL_GAUGE_EMPTY, 
	TEMP_GAUGE_HOT, 
	TEMP_GAUGE_COLD, 
	BMS_MIN_VOLTAGE, 
	BMS_MAX_VOLTAGE, 
	BALANCE_VOLTAGE,
	BMS_HYSTERESIS, 
	BMS_MIN_TEMP, 
	BMS_MAX_TEMP, 
//	LOW_TEMP_CHARGE_RESTRICT,
	CHARGER_VOLTAGE, 
	CHARGER_CURRENT, 
	CHARGER_VOLTAGE2, 
	CHARGER_CURRENT2, 
	CAN_POWER_DOWN_DELAY,
	MPI_FUNCTION,
	MPO1_FUNCTION,
	MPO2_FUNCTION,
	NUM_PARALLEL_STRINGS,
	ENABLE_PRECHARGE, 
	STATIONARY_VERSION, 
	REVERSE_CURRENT_DISPLAY,
	NIGHT_BRIGHTNESS,
	BUZZER_ON,
	USE_FAHRENHEIT,
	SOC_DISPLAY,
	NUM_SETTINGS };

unsigned char settings[NUM_SETTINGS] = {
	20,		// Pack capacity (Ah x 5)
	20,		// Soc warning (%)
	80,		// Full voltage (x2V)
	121,	// Current warning (A x10)
	121,	// Current trip (A x10)
	151,	// Over temp (degC)
	10,		// Min aux voltage (V)
	50,		// Min isolation (% of whatever full scale is)
	2,		// Tacho PPR
	80,		// Fuel gauge full
	20,		// Fuel gauge empty
	80,		// Temp gauge hot
	20,		// Temp gauge cold
	100,	// BMS min voltage (1.50 + 0.01N V)
	180,	// BMS max voltage (2.00 + 0.01N V)
	251,	// Balance voltage (2.00 + 0.01N V,     251 = dynamic)
	20,		// BMS hysteresis (stationary mode only 0.01V resolution)
	0,		// BMS min temp (-40)
	141,	// BMS max temp (-40)
//	0,		// Low temp charge restrict
	100,	// Charger voltage (mostly, also needs 9th bit stored in next byte)
	10,		// Charger current
	100,	// Charger voltage 2
	20,		// Charger current 2
	5,		// CAN power down delay
	0,		// MPI function
	0,		// MPO1 function
	0,		// MPO2 function
	1,		// Number of parallel strings
	1,		// Enable precharge
	0, 		// Stationary version (true/false)
	0,		// Reverse current display
	10,		// Night brightness
	1,		// Buzzer on
	0,		// Use fahrenheit
	0,		// Percentage or Amp-hours
};

unsigned char minimums[NUM_SETTINGS] = {
	1,		// Pack capacity (Ah x 5)
	0,		// Soc warning (%)
	5,		// Full voltage (x2V)
	1,		// Current warning (A x10)
	1,		// Current trip (A x10)
	0,		// Over temp (degC)
	8,		// Min aux voltage (V)
	0,		// Min isolation (% of whatever full scale is)
	1,		// Tacho PPR
	0,		// Fuel gauge full
	0,		// Fuel gauge empty
	0,		// Temp gauge hot
	0,		// Temp gauge cold
	0,		// BMS min voltage (1.50 + 0.01N V)
	0,		// BMS max voltage (2.00 + 0.01N V)
	0,		// Balance voltage (2.00 + 0.01N V)
	0,		// BMS hysteresis (stationary mode only 0.01V resolution)
	0,		// BMS min temp (-40)
	0,		// BMS max temp (-40)
//	0,		// Low temp charge restrict
	0,		// Charger voltage (mostly, also needs 9th bit stored in next byte)
	0,		// Charger current
	0,		// Charger voltage 2
	0,		// Charger current 2
	1,		// CAN power down delay
	0,		// MPI function
	0,		// MPO1 function
	0,		// MPO2 function
	1,		// Number of parallel strings
	0,		// Enable precharge
	0, 		// Stationary version (true/false)
	0,		// Reverse current display
	0,		// Night brightness
	0,		// Buzzer on
	0,		// Use fahrenheit
	0,		// SoC percent or amp-hours
};

unsigned char maximums[NUM_SETTINGS] = {
	250,		// Pack capacity (Ah x 5)
	99,		// Soc warning (%)
	251,		// Full voltage (x2V)
	121,	// Current warning (A x10)
	121,	// Current trip (A x10)
	151,	// Over temp (degC)
	16,		// Min aux voltage (V)
	99,		// Min isolation (% of whatever full scale is)
	6,		// Tacho PPR
	100,		// Fuel gauge full
	100,		// Fuel gauge empty
	100,		// Temp gauge hot
	100,		// Temp gauge cold
	250,	// BMS min voltage (1.50 + 0.01N V)
	250,	// BMS max voltage (2.00 + 0.01N V)
	252,	// Balance voltage (2.00 + 0.02N V, max two values are DYNAMIC or OFF)
	50,		// BMS hysteresis (stationary mode only 0.01V resolution)
	141,	// BMS min temp (-40)
	141,	// BMS max temp (-40)
//	1,		// Low temp charge restrict
	255,	// Charger voltage (mostly, also needs 9th bit stored in next byte)
	255,	// Charger current
	255,	// Charger voltage 2
	255,	// Charger current 2
	6,		// CAN power down delay
	MPI_NUM_FUNCTIONS-1,
	MPO_NUM_FUNCTIONS-1,
	MPO_NUM_FUNCTIONS-1,
	20,		// Number of parallel strings
	1,		// Enable precharge
	1, 		// Stationary version (true/false)
	1,		// Reverse current display
	10,		// Night brightness
	1,		// Buzzer on
	1,		// Use fahrenheit
	1,		// SoC percent or amp hours
};

unsigned char bms16maximums[NUM_SETTINGS] = {
    250,		// Pack capacity (Ah x 5)
    99,		// Soc warning (%)
    70,		// Full voltage (x2V normally, x1V for BMS16
    121,	// Current warning (A x10)
    121,	// Current trip (A x10)
    0,	// Over temp (degC)
    16,		// Min aux voltage (V) - REUSE FOR NUM CELLS
    3,		// Min isolation - REUSE FOR CURRENT SHUNT SIZE, 3 = 500A
    0,		// Tacho PPR
    0,		// Fuel gauge full
    0,		// Fuel gauge empty
    0,		// Temp gauge hot
    0,		// Temp gauge cold
    250,	// BMS min voltage (1.50 + 0.01N V)
    250,	// BMS max voltage (2.00 + 0.01N V)
    252,	// Balance voltage (2.00 + 0.02N V, max two values are DYNAMIC or OFF)
    50,		// BMS hysteresis (stationary mode only 0.01V resolution)
    141,	// BMS min temp (-40)
    141,	// BMS max temp (-40)
    //	1,		// Low temp charge restrict
    70,	// Charger voltage (mostly, also needs 9th bit stored in next byte)
    255,	// Charger current
    0,	// Charger voltage 2
    0,	// Charger current 2
    0,		// CAN power down delay
    0,  // MPI functions
    0,  // MPO1 functions
    0,  // MPO2 functions
    0,		// Number of parallel strings
    0,		// Enable precharge
    1, 		// Stationary version (true/false)
    1,		// Reverse current display
    10,		// Night brightness
    1,		// Buzzer on
    1,		// Use fahrenheit
    1,		// SoC percent or amp hours
};

// Reassign a couple of settings that the BMS16 needs to be different
#define NUM_CELLS   MIN_AUX_VOLTAGE
#define SHUNT_SIZE  MIN_ISOLATION

#ifdef MONITOR
	const char s0[] PROGMEM  = " Pack Capacity ";
	const char s1[] PROGMEM  = "  SoC Warning  ";
	const char s2[] PROGMEM  = " Full Voltage ";
	const char s3[] PROGMEM  = " Warn Current ";
	const char s4[] PROGMEM  = "   Trip Current   ";
	const char s5[] PROGMEM  = "EVMS Temp Warning";
	const char s6[] PROGMEM  = " Min Aux Voltage ";
	const char s7[] PROGMEM  = " Min Isolation ";
	const char s8[] PROGMEM  = "   Tacho PPR   ";
	const char s9[] PROGMEM  = " Fuel Gauge Full ";
	const char s10[] PROGMEM = "Fuel Gauge Empty";
	const char s11[] PROGMEM = " Temp Gauge Hot ";
	const char s12[] PROGMEM = " Temp Gauge Cold ";
	const char s14[] PROGMEM = " BMS Min Voltage ";
	const char s15[] PROGMEM = " BMS Max Voltage ";
	const char s18b[] PROGMEM= " Balance Voltage ";
	const char s16[] PROGMEM = "  BMS Hysteresis  ";
	const char s17[] PROGMEM = "  BMS Min Temp  ";
	const char s18[] PROGMEM = "  BMS Max Temp  ";
//	const char s19[] PROGMEM = "Low Temp Chg Rest.";
	const char s21[] PROGMEM = "Max Charge Voltage";
	const char s22[] PROGMEM = "Max Charge Current";
	const char s23[] PROGMEM = "Alt Charge Voltage";
	const char s24[] PROGMEM = "Alt Charge Current";
	const char s25[] PROGMEM = "    Sleep Delay    ";
	const char s26[] PROGMEM = "  MPI Function  ";
	const char s27[] PROGMEM = "  MPO1 Function  ";
	const char s28[] PROGMEM = "  MPO2 Function  ";
	const char s29[] PROGMEM = " Parallel Strings ";
	const char s13[] PROGMEM = "Enable Precharge";
	const char s20[] PROGMEM = "  Stationary Mode  ";
	const char s30[] PROGMEM = "Rev. Current Disp";
	const char s31[] PROGMEM = "Night Brightness";
	const char s32[] PROGMEM =  "    Buzzer On    ";
	const char s33[] PROGMEM =  " Use Fahrenheit ";
	const char s34[] PROGMEM =  "   SoC Display   ";
	PROGMEM const char* const generalSettingsLabels[] = { s0,s1,s2,s3,s4,s5,s6,s7,s8,s9,s10,
		s11,s12,s14,s15,s18b,s16,s17,s18,s21,s22,s23,s24,s25,s26,s27,s28,s29,s13,s20,s30,s31,s32,s33,s34 };
	char* allSettingsUnits[] = { "Ah", "%", "V", "A", "A", "C", "V", "%", "", "%", "%", "%", "%", // temp gauge cold
		"V", "V", "V", "V", "C", "C", "V", "A", "V", "A", "min", "", "", "", "", "", "", "", "%", "", "", "" };
#endif

unsigned char bmsCellCounts[16] = { 12, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
//unsigned char bmsCellCounts[24] = { 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12 };

// Utility functions
static inline short Abs(short num)
{
	if (num < 0) return -num;
	return num;
}

static inline short Cap(short val, short min, short max)
{
	if (val < min) return min;
	if (val > max) return max;
	return val;
}

static inline long CapLong(long val, long min, long max)
{
	if (val < min) return min;
	if (val > max) return max;
	return val;
}
