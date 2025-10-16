#include "include/mqtt.h"
#include "esp_log.h"
#include <stdio.h>

static const char *TAG = "HPC_MQTT";

// // Topic names (from HeishaMon)
// static const char topics[][MAX_TOPIC_LEN] = {
//     "Heatpump_State",          // TOP0
//     "Pump_Flow",               // TOP1
//     "Force_DHW_State",         // TOP2
//     "Quiet_Mode_Schedule",     // TOP3
//     "Operating_Mode_State",    // TOP4
//     "Main_Inlet_Temp",         // TOP5
//     "Main_Outlet_Temp",        // TOP6
//     "Main_Target_Temp",        // TOP7
//     "Compressor_Freq",         // TOP8
//     "DHW_Target_Temp",         // TOP9
//     "DHW_Temp",                // TOP10
//     "Operations_Hours",        // TOP11
//     "Operations_Counter",      // TOP12
//     "Main_Schedule_State",     // TOP13
//     "Outside_Temp",            // TOP14
//     "Heat_Power_Production",   // TOP15
//     "Heat_Power_Consumption",  // TOP16
//     "Powerful_Mode_Time",      // TOP17
//     "Quiet_Mode_Level",        // TOP18
//     "Holiday_Mode_State",      // TOP19
//     "ThreeWay_Valve_State",    // TOP20
//     "Outside_Pipe_Temp",       // TOP21
//     "DHW_Heat_Delta",          // TOP22
//     "Heat_Delta",              // TOP23
//     "Cool_Delta",              // TOP24
//     "DHW_Holiday_Shift_Temp",  // TOP25
//     "Defrosting_State",        // TOP26
//     "Z1_Heat_Request_Temp",    // TOP27
//     "Z1_Cool_Request_Temp",    // TOP28
//     "Z1_Heat_Curve_Target_High_Temp",      // TOP29
//     "Z1_Heat_Curve_Target_Low_Temp",       // TOP30
//     "Z1_Heat_Curve_Outside_High_Temp",     // TOP31
//     "Z1_Heat_Curve_Outside_Low_Temp",      // TOP32
//     "Room_Thermostat_Temp",    // TOP33
//     "Z2_Heat_Request_Temp",    // TOP34
//     "Z2_Cool_Request_Temp",    // TOP35
//     "Z1_Water_Temp",           // TOP36
//     "Z2_Water_Temp",           // TOP37
//     "Cool_Power_Production",   // TOP38
//     "Cool_Power_Consumption",  // TOP39
//     "DHW_Power_Production",    // TOP40
//     "DHW_Power_Consumption",   // TOP41
//     "Z1_Water_Target_Temp",    // TOP42
//     "Z2_Water_Target_Temp",    // TOP43
//     "Error",                   // TOP44
//     "Room_Holiday_Shift_Temp", // TOP45
//     "Buffer_Temp",             // TOP46
//     "Solar_Temp",              // TOP47
//     "Pool_Temp",               // TOP48
//     "Main_Hex_Outlet_Temp",    // TOP49
//     "Discharge_Temp",          // TOP50
//     "Inside_Pipe_Temp",        // TOP51
//     "Defrost_Temp",            // TOP52
//     "Eva_Outlet_Temp",         // TOP53
//     "Bypass_Outlet_Temp",      // TOP54
//     "Ipm_Temp",                // TOP55
//     "Z1_Temp",                 // TOP56
//     "Z2_Temp",                 // TOP57
//     "DHW_Heater_State",        // TOP58
//     "Room_Heater_State",       // TOP59
//     "Internal_Heater_State",   // TOP60
//     "External_Heater_State",   // TOP61
//     "Fan1_Motor_Speed",        // TOP62
//     "Fan2_Motor_Speed",        // TOP63
//     "High_Pressure",           // TOP64
//     "Pump_Speed",              // TOP65
//     "Low_Pressure",            // TOP66
//     "Compressor_Current",      // TOP67
//     "Force_Heater_State",      // TOP68
//     "Sterilization_State",     // TOP69
//     "Sterilization_Temp",      // TOP70
//     "Sterilization_Max_Time",  // TOP71
//     "Z1_Cool_Curve_Target_High_Temp",      // TOP72
//     "Z1_Cool_Curve_Target_Low_Temp",       // TOP73
//     "Z1_Cool_Curve_Outside_High_Temp",     // TOP74
//     "Z1_Cool_Curve_Outside_Low_Temp",      // TOP75
//     "Heating_Mode",            // TOP76
//     "Heating_Off_Outdoor_Temp",// TOP77
//     "Heater_On_Outdoor_Temp",  // TOP78
//     "Heat_To_Cool_Temp",       // TOP79
//     "Cool_To_Heat_Temp",       // TOP80
//     "Cooling_Mode",            // TOP81
//     "Z2_Heat_Curve_Target_High_Temp",      // TOP82
//     "Z2_Heat_Curve_Target_Low_Temp",       // TOP83
//     "Z2_Heat_Curve_Outside_High_Temp",     // TOP84
//     "Z2_Heat_Curve_Outside_Low_Temp",      // TOP85
//     "Z2_Cool_Curve_Target_High_Temp",      // TOP86
//     "Z2_Cool_Curve_Target_Low_Temp",       // TOP87
//     "Z2_Cool_Curve_Outside_High_Temp",     // TOP88
//     "Z2_Cool_Curve_Outside_Low_Temp",      // TOP89
//     "Room_Heater_Operations_Hours",        // TOP90
//     "DHW_Heater_Operations_Hours",         // TOP91
//     "Heat_Pump_Model",         // TOP92
//     "Pump_Duty",               // TOP93
//     "Zones_State",             // TOP94
//     "Max_Pump_Duty",           // TOP95
//     "Heater_Delay_Time",       // TOP96
//     "Heater_Start_Delta",      // TOP97
//     "Heater_Stop_Delta",       // TOP98
//     "Buffer_Installed",        // TOP99
//     "DHW_Installed",           // TOP100
//     "Solar_Mode",              // TOP101
//     "Solar_On_Delta",          // TOP102
//     "Solar_Off_Delta",         // TOP103
//     "Solar_Frost_Protection",  // TOP104
//     "Solar_High_Limit",        // TOP105
//     "Pump_Flowrate_Mode",      // TOP106
//     "Liquid_Type",             // TOP107
//     "Alt_External_Sensor",     // TOP108
//     "Anti_Freeze_Mode",        // TOP109
//     "Optional_PCB",            // TOP110
//     "Z1_Sensor_Settings",      // TOP111
//     "Z2_Sensor_Settings",      // TOP112
//     "Buffer_Tank_Delta",       // TOP113
//     "External_Pad_Heater",     // TOP114
//     "Water_Pressure",          // TOP115
//     "Second_Inlet_Temp",       // TOP116
//     "Economizer_Outlet_Temp",  // TOP117
//     "Second_Room_Thermostat_Temp",// TOP118
//     "External_Control",        // TOP119
//     "External_Heat_Cool_Control", // TOP120
//     "External_Error_Signal",   // TOP121
//     "External_Compressor_Control", // TOP122
//     "Z2_Pump_State",           // TOP123
//     "Z1_Pump_State",           // TOP124
//     "TwoWay_Valve_State",      // TOP125
//     "ThreeWay_Valve_State2",   // TOP126
//     "Z1_Valve_PID",            // TOP127
//     "Z2_Valve_PID",            // TOP128
//     "Bivalent_Control",        // TOP129
//     "Bivalent_Mode",           // TOP130
//     "Bivalent_Start_Temp",     // TOP131
//     "Bivalent_Advanced_Heat",  // TOP132
//     "Bivalent_Advanced_DHW",   // TOP133
//     "Bivalent_Advanced_Start_Temp",// TOP134
//     "Bivalent_Advanced_Stop_Temp",// TOP135
//     "Bivalent_Advanced_Start_Delay",// TOP136
//     "Bivalent_Advanced_Stop_Delay",// TOP137
//     "Bivalent_Advanced_DHW_Delay",// TOP138
// };

// // Extra topic names
// static const char xtopics[][MAX_TOPIC_LEN] = {
//     "Heat_Power_Consumption_Extra", // XTOP0
//     "Cool_Power_Consumption_Extra", // XTOP1
//     "DHW_Power_Consumption_Extra",  // XTOP2
//     "Heat_Power_Production_Extra",  // XTOP3
//     "Cool_Power_Production_Extra",  // XTOP4
//     "DHW_Power_Production_Extra",   // XTOP5
// };

// // Optional topic names
// static const char optTopics[][20] = {
//     "Z1_Water_Pump",           // OPT0
//     "Z1_Mixing_Valve",         // OPT1
//     "Z2_Water_Pump",           // OPT2
//     "Z2_Mixing_Valve",         // OPT3
//     "Pool_Water_Pump",         // OPT4
//     "Solar_Water_Pump",        // OPT5
//     "Alarm_State",             // OPT6
// };

// Description arrays (from HeishaMon)
static const char *DisabledEnabled[] = {"2", "Disabled", "Enabled"};
static const char *BlockedFree[] = {"2", "Blocked", "Free"};
static const char *OffOn[] = {"2", "Off", "On"};
static const char *InactiveActive[] = {"2", "Inactive", "Active"};
static const char *PumpFlowRateMode[] = {"2", "DeltaT", "Max flow"};
static const char *HolidayState[] = {"3", "Off", "Scheduled", "Active"};
static const char *OpModeDesc[] = {"9", "Heat", "Cool", "Auto(heat)", "DHW", "Heat+DHW", "Cool+DHW", "Auto(heat)+DHW", "Auto(cool)", "Auto(cool)+DHW"};
static const char *Powerfulmode[] = {"4", "Off", "30min", "60min", "90min"};
static const char *Quietmode[] = {"4", "Off", "Level 1", "Level 2", "Level 3"};
static const char *Valve[] = {"2", "Room", "DHW"};
static const char *Valve2[] = {"2", "Cool", "Heat"};
static const char *MixingValve[] = {"4", "Off", "Decrease","Increase","Invalid"};
static const char *LitersPerMin[] = {"0", "l/min"};
static const char *RotationsPerMin[] = {"0", "r/min"};
static const char *Bar[] = {"0", "Bar"};
static const char *Pressure[] = {"0", "Kgf/cm2"};
static const char *Celsius[] = {"0", "°C"};
static const char *Kelvin[] = {"0", "K"};
static const char *Hertz[] = {"0", "Hz"};
static const char *Counter[] = {"0", "count"};
static const char *Hours[] = {"0", "hours"};
static const char *Watt[] = {"0", "Watt"};
static const char *ErrorState[] = {"0", "Error"};
static const char *Ampere[] = {"0", "Ampere"};
static const char *Minutes[] = {"0", "Minutes"};
static const char *Duty[] = {"0", "Duty"};
static const char *ZonesState[] = {"3", "Zone1 active", "Zone2 active", "Zone1 and zone2 active"};
static const char *HeatCoolModeDesc[] = {"2", "Comp. Curve", "Direct"};
static const char *SolarModeDesc[] = {"3", "Disabled", "Buffer", "DHW"};
static const char *ZonesSensorType[] = {"4", "Water Temperature", "External Thermostat", "Internal Thermostat", "Thermistor"};
static const char *LiquidType[] = {"2", "Water", "Glycol"};
static const char *ExtPadHeaterType[] = {"3", "Disabled", "Type-A","Type-B"};
static const char *Bivalent[] = {"3", "Alternative", "Parallel", "Advanced Parallel"};
static const char *Percent[] = {"0", "%"};
static const char *Model[] = {"0", "Model"};

static const char **topicDescription[] = {
    OffOn,           // TOP0
    LitersPerMin,    // TOP1
    DisabledEnabled, // TOP2
    DisabledEnabled, // TOP3
    OpModeDesc,      // TOP4
    Celsius,         // TOP5
    Celsius,         // TOP6
    Celsius,         // TOP7
    Hertz,           // TOP8
    Celsius,         // TOP9
    Celsius,         // TOP10
    Hours,           // TOP11
    Counter,         // TOP12
    DisabledEnabled, // TOP13
    Celsius,         // TOP14
    Watt,            // TOP15
    Watt,            // TOP16
    Powerfulmode,    // TOP17
    Quietmode,       // TOP18
    HolidayState,    // TOP19
    Valve,           // TOP20
    Celsius,         // TOP21
    Kelvin,          // TOP22
    Kelvin,          // TOP23
    Kelvin,          // TOP24
    Kelvin,          // TOP25
    DisabledEnabled, // TOP26
    Celsius,         // TOP27
    Celsius,         // TOP28
    Celsius,         // TOP29
    Celsius,         // TOP30
    Celsius,         // TOP31
    Celsius,         // TOP32
    Celsius,         // TOP33
    Celsius,         // TOP34
    Celsius,         // TOP35
    Celsius,         // TOP36
    Celsius,         // TOP37
    Watt,            // TOP38
    Watt,            // TOP39
    Watt,            // TOP40
    Watt,            // TOP41
    Celsius,         // TOP42
    Celsius,         // TOP43
    ErrorState,      // TOP44
    Kelvin,          // TOP45
    Celsius,         // TOP46
    Celsius,         // TOP47
    Celsius,         // TOP48
    Celsius,         // TOP49
    Celsius,         // TOP50
    Celsius,         // TOP51
    Celsius,         // TOP52
    Celsius,         // TOP53
    Celsius,         // TOP54
    Celsius,         // TOP55
    Celsius,         // TOP56
    Celsius,         // TOP57
    BlockedFree,     // TOP58
    BlockedFree,     // TOP59
    InactiveActive,  // TOP60
    InactiveActive,  // TOP61
    RotationsPerMin, // TOP62
    RotationsPerMin, // TOP63
    Pressure,        // TOP64
    RotationsPerMin, // TOP65
    Pressure,        // TOP66
    Ampere,          // TOP67
    InactiveActive,  // TOP68
    InactiveActive,  // TOP69
    Celsius,         // TOP70
    Minutes,         // TOP71
    Celsius,         // TOP72
    Celsius,         // TOP73
    Celsius,         // TOP74
    Celsius,         // TOP75
    HeatCoolModeDesc,// TOP76
    Celsius,         // TOP77
    Celsius,         // TOP78
    Celsius,         // TOP79
    Celsius,         // TOP80
    HeatCoolModeDesc,// TOP81
    Celsius,         // TOP82
    Celsius,         // TOP83
    Celsius,         // TOP84
    Celsius,         // TOP85
    Celsius,         // TOP86
    Celsius,         // TOP87
    Celsius,         // TOP88
    Celsius,         // TOP89
    Hours,           // TOP90
    Hours,           // TOP91
    Model,           // TOP92
    Duty,            // TOP93
    ZonesState,      // TOP94
    Duty,            // TOP95
    Minutes,         // TOP96
    Kelvin,          // TOP97
    Kelvin,          // TOP98
    DisabledEnabled, // TOP99
    DisabledEnabled, // TOP100
    SolarModeDesc,   // TOP101
    Kelvin,          // TOP102
    Kelvin,          // TOP103
    Celsius,         // TOP104
    Celsius,         // TOP105
    PumpFlowRateMode,// TOP106
    LiquidType,      // TOP107
    DisabledEnabled, // TOP108
    DisabledEnabled, // TOP109
    DisabledEnabled, // TOP110
    ZonesSensorType, // TOP111
    ZonesSensorType, // TOP112
    Kelvin,          // TOP113
    ExtPadHeaterType,// TOP114
    Bar,             // TOP115
    Celsius,         // TOP116
    Celsius,         // TOP117
    Celsius,         // TOP118
    DisabledEnabled, // TOP119
    DisabledEnabled, // TOP120
    DisabledEnabled, // TOP121
    DisabledEnabled, // TOP122
    OffOn,           // TOP123
    OffOn,           // TOP124
    Valve2,          // TOP125
    Valve,           // TOP126
    Percent,         // TOP127
    Percent,         // TOP128
    DisabledEnabled, // TOP129
    Bivalent,        // TOP130
    Celsius,         // TOP131
    DisabledEnabled, // TOP132
    DisabledEnabled, // TOP133
    Celsius,         // TOP134
    Celsius,         // TOP135
    Minutes,         // TOP136
    Minutes,         // TOP137
    Minutes,         // TOP138
};

static const char **xtopicDescription[] = {
    Watt,           // XTOP0
    Watt,           // XTOP1
    Watt,           // XTOP2
    Watt,           // XTOP3
    Watt,           // XTOP4
    Watt,           // XTOP5
};

static const char **opttopicDescription[] = {
    OffOn,          // OPT0
    MixingValve,    // OPT1
    OffOn,          // OPT2
    MixingValve,    // OPT3
    OffOn,          // OPT4
    OffOn,          // OPT5
    OffOn,          // OPT6
};



static struct {
    char base_topic[128];
} s_mqtt = {0};

esp_err_t hpc_mqtt_init(const hpc_mqtt_config_t *config) {
    if (config == NULL || config->broker_uri == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    const char *base = config->base_topic != NULL ? config->base_topic : "heishamon";
    snprintf(s_mqtt.base_topic, sizeof(s_mqtt.base_topic), "%s", base);
    ESP_LOGI(TAG, "MQTT init: %s base=%s", config->broker_uri, s_mqtt.base_topic);
    return ESP_OK;
}

esp_err_t hpc_mqtt_start(void) {
    ESP_LOGI(TAG, "MQTT started");
    return ESP_OK;
}

static int publish(const char *subtopic, const char *payload, int qos, bool retain) {
    (void)qos; (void)retain;
    if (!subtopic || !payload) return -1;
    char topic[256];
    int n = snprintf(topic, sizeof(topic), "%s/%s", s_mqtt.base_topic, subtopic);
    if (n <= 0 || n >= (int)sizeof(topic)) return -1;
    ESP_LOGI(TAG, "MQTT PUBLISH %s = %s", topic, payload);
    return 0;
}

esp_err_t hpc_mqtt_publish_str(const char *subtopic, const char *payload, int qos, bool retain) {
    return publish(subtopic, payload, qos, retain) >= 0 ? ESP_OK : ESP_FAIL;
}

esp_err_t hpc_mqtt_publish_int(const char *subtopic, int value, int qos, bool retain) {
    char buf[32];
    snprintf(buf, sizeof(buf), "%d", value);
    return hpc_mqtt_publish_str(subtopic, buf, qos, retain);
}


