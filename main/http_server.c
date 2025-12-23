/**
 * @file http_server.c
 * @brief HTTP server implementation for viewing heat pump parameters
 * @version 1.0.0
 * @date 2025
 */

#include "include/http_server.h"
#include "include/modbus_params.h"
#include "include/wifi_connect.h"
#include "include/mqtt_pub.h"
#include "esp_log.h"
#include "esp_http_server.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "cJSON.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "HTTP_SERVER";
static httpd_handle_t server_handle = NULL;

// Helper function to format temperature value
static void format_temp_value(char *buf, size_t len, int16_t value, bool is_x100) {
    if (value == INT16_MIN || value == 0) {
        snprintf(buf, len, "--");
    } else {
        if (is_x100) {
            snprintf(buf, len, "%.2f", value / 100.0f);
        } else {
            snprintf(buf, len, "%d", value);
        }
    }
}

// Helper function to format value with unit
static void format_value_with_unit(char *buf, size_t len, int16_t value, const char *unit) {
    if (value == INT16_MIN || value == 0) {
        snprintf(buf, len, "--");
    } else {
        snprintf(buf, len, "%d %s", value, unit);
    }
}

// Get formatted value for a register
static void get_formatted_value(char *buf, size_t len, uint16_t reg_addr, mqtt_subtopic_t subtopic) {
    int16_t value = mb_input_registers[reg_addr];
    
    if (value == INT16_MIN) {
        snprintf(buf, len, "--");
        return;
    }
    
    switch (subtopic) {
        case MQTT_SUB_TEMP:
            // Check if it's a temperature stored as *100 (main inlet/outlet)
            if (reg_addr == MB_INPUT_MAIN_INLET_TEMP || reg_addr == MB_INPUT_MAIN_OUTLET_TEMP) {
                format_temp_value(buf, len, value, true);
            } else {
                format_temp_value(buf, len, value, false);
            }
            break;
        case MQTT_SUB_POWER:
            snprintf(buf, len, "%d", value);
            break;
        case MQTT_SUB_FREQ:
            snprintf(buf, len, "%d", value);
            break;
        case MQTT_SUB_FLOW:
            snprintf(buf, len, "%d", value);
            break;
        case MQTT_SUB_SPEED:
            snprintf(buf, len, "%d", value);
            break;
        case MQTT_SUB_PRESS:
            snprintf(buf, len, "%d", value);
            break;
        case MQTT_SUB_CURRENT:
            snprintf(buf, len, "%d", value);
            break;
        case MQTT_SUB_DUTY:
            snprintf(buf, len, "%d", value);
            break;
        case MQTT_SUB_HOUR:
            snprintf(buf, len, "%d", value);
            break;
        case MQTT_SUB_COUNT:
            snprintf(buf, len, "%d", value);
            break;
        default:
            snprintf(buf, len, "%d", value);
            break;
    }
}

// Root handler - HTML page
static esp_err_t root_handler(httpd_req_t *req) {
    ESP_LOGI(TAG, "HTML page requested");
    
    // Start building HTML page
    const char* html_start = 
        "<!DOCTYPE html>"
        "<html><head>"
        "<title>Panasonic Heat Pump Monitor</title>"
        "<meta charset='UTF-8'>"
        "<meta name='viewport' content='width=device-width, initial-scale=1'>"
        "<style>"
        "body{font-family:Arial,sans-serif;margin:0;padding:20px;background:#f5f5f5;color:#333;}"
        ".container{max-width:1400px;margin:0 auto;background:white;border-radius:10px;box-shadow:0 2px 10px rgba(0,0,0,0.1);overflow:hidden;}"
        ".header{background:linear-gradient(135deg,#2196F3,#1976D2);color:white;padding:20px;text-align:center;}"
        ".header h1{margin:0;font-size:28px;font-weight:300;}"
        ".status-bar{background:#E3F2FD;padding:10px 20px;border-bottom:1px solid #BBDEFB;display:flex;justify-content:space-between;align-items:center;flex-wrap:wrap;gap:10px;}"
        ".status-item{display:flex;align-items:center;gap:8px;}"
        ".status-dot{width:8px;height:8px;border-radius:50%;background:#4CAF50;}"
        ".status-dot.offline{background:#F44336;}"
        ".content{padding:20px;}"
        ".grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(300px,1fr));gap:20px;margin-bottom:20px;}"
        ".card{background:white;border:1px solid #E0E0E0;border-radius:8px;overflow:hidden;box-shadow:0 1px 3px rgba(0,0,0,0.1);}"
        ".card-header{background:#F8F9FA;padding:15px;border-bottom:1px solid #E0E0E0;font-weight:600;color:#424242;}"
        ".card-body{padding:15px;max-height:400px;overflow-y:auto;}"
        ".param-row{display:flex;justify-content:space-between;align-items:center;padding:8px 0;border-bottom:1px solid #F5F5F5;}"
        ".param-row:last-child{border-bottom:none;}"
        ".param-name{color:#666;font-size:14px;}"
        ".param-value{font-weight:600;color:#2196F3;font-size:16px;}"
        ".temp-value{color:#FF5722;}"
        ".power-value{color:#4CAF50;}"
        ".controls{padding:20px;background:#F8F9FA;border-top:1px solid #E0E0E0;text-align:center;}"
        ".btn{background:#2196F3;color:white;border:none;padding:10px 20px;border-radius:5px;cursor:pointer;font-size:14px;margin:0 5px;}"
        ".btn:hover{background:#1976D2;}"
        ".last-update{color:#999;font-size:12px;margin-top:10px;}"
        "</style>"
        "</head><body>"
        "<div class='container'>"
        "<div class='header'>"
        "<h1>🏠 Panasonic Heat Pump Monitor</h1>"
        "</div>"
        "<div class='status-bar'>"
        "<div class='status-item'>"
        "<div class='status-dot' id='statusDot'></div>"
        "<span id='statusText'>Loading...</span>"
        "</div>"
        "<div class='status-item'>"
        "<span>IP: <strong id='ip'>--</strong></span>"
        "<span>WiFi: <strong id='wifi'>--</strong></span>"
        "<span>Free memory: <strong id='memory'>--</strong></span>"
        "<span>Uptime: <strong id='uptime'>--</strong></span>"
        "<span>Last update: <strong id='lastUpdate'>--</strong></span>"
        "</div>"
        "</div>"
        "<div class='controls'>"
        "<button class='btn' onclick='loadData()'>🔄 Refresh Data</button>"
        "<button class='btn' onclick='toggleAutoRefresh()' id='autoBtn'>⏸️ Pause Auto-refresh</button>"
        "</div>"
        "<div class='content'>"
        "<div class='grid' id='dataGrid'>"
        "Loading data..."
        "</div>"
        "</div>"
        "</div>"
        "<script>"
        "var autoRefresh=true;"
        "var refreshInterval;"
        "function setElementText(id,text){var el=document.getElementById(id);if(el)el.textContent=text;}"
        "function formatUptime(hours){"
        "  if(!hours||isNaN(hours)||hours<0)return'0m';"
        "  var totalMinutes=Math.floor(hours*60);"
        "  var days=Math.floor(totalMinutes/(24*60));"
        "  var remainingMinutes=totalMinutes%(24*60);"
        "  var hours_part=Math.floor(remainingMinutes/60);"
        "  var minutes_part=remainingMinutes%60;"
        "  var result='';"
        "  if(days>0)result+=days+'d ';"
        "  if(hours_part>0)result+=hours_part+'h ';"
        "  if(minutes_part>0||(days===0&&hours_part===0))result+=minutes_part+'m';"
        "  return result.trim()||'0m';"
        "}"
        "function updateDisplay(data){"
        "  setElementText('ip',data.device_ip||'--');"
        "  setElementText('wifi',data.wifi_rssi?data.wifi_rssi+' dBm':'--');"
        "  setElementText('memory',data.free_memory?data.free_memory.toFixed(1)+' kB':'--');"
        "  setElementText('uptime',formatUptime(data.uptime));"
        "  var status=document.getElementById('statusText');"
        "  var dot=document.getElementById('statusDot');"
        "  if(data.status==='online'){"
        "    setElementText('statusText','Online');"
        "    dot.className='status-dot';"
        "  }else{"
        "    setElementText('statusText','Offline');"
        "    dot.className='status-dot offline';"
        "  }"
        "  var grid=document.getElementById('dataGrid');"
        "  if(data.params&&data.params.length>0){"
        "    var html='';"
        "    var currentCategory='';"
        "    data.params.forEach(function(param){"
        "      if(param.category!==currentCategory){"
        "        if(currentCategory!=='')html+='</div></div>';"
        "        currentCategory=param.category;"
        "        html+='<div class=\"card\"><div class=\"card-header\">'+param.category+'</div><div class=\"card-body\">';"
        "      }"
        "      var valueClass='param-value';"
        "      if(param.unit==='°C')valueClass+=' temp-value';"
        "      else if(param.unit==='W')valueClass+=' power-value';"
        "      html+='<div class=\"param-row\"><span class=\"param-name\">'+param.name+'</span><span class=\"'+valueClass+'\">'+(param.value||'--')+(param.unit||'')+'</span></div>';"
        "    });"
        "    if(currentCategory!=='')html+='</div></div>';"
        "    grid.innerHTML=html;"
        "  }"
        "  setElementText('lastUpdate',new Date().toLocaleTimeString());"
        "}"
        "function loadData(){"
        "  var x=new XMLHttpRequest();"
        "  x.open('GET','/json',true);"
        "  x.onreadystatechange=function(){"
        "    if(x.readyState==4){"
        "      if(x.status==200){"
        "        try{"
        "          var data=JSON.parse(x.responseText);"
        "          updateDisplay(data);"
        "        }catch(e){"
        "          console.error('JSON parsing error:', e);"
        "        }"
        "      }"
        "    }"
        "  };"
        "  x.send();"
        "}"
        "function toggleAutoRefresh(){"
        "  autoRefresh=!autoRefresh;"
        "  var btn=document.getElementById('autoBtn');"
        "  if(autoRefresh){"
        "    btn.innerHTML='⏸️ Pause Auto-refresh';"
        "    refreshInterval=setInterval(loadData,5000);"
        "  }else{"
        "    btn.innerHTML='▶️ Resume Auto-refresh';"
        "    clearInterval(refreshInterval);"
        "  }"
        "}"
        "window.onload=function(){"
        "  loadData();"
        "  refreshInterval=setInterval(loadData,5000);"
        "};"
        "</script>"
        "</body></html>";
    
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, html_start, strlen(html_start));
    return ESP_OK;
}

// JSON API handler
static esp_err_t json_handler(httpd_req_t *req) {
    cJSON *json = cJSON_CreateObject();
    cJSON *params_array = cJSON_CreateArray();
    
    // Check if we have valid data
    bool data_valid = (mb_input_registers[MB_INPUT_STATUS] != 0 || 
                       mb_input_registers[MB_INPUT_MAIN_INLET_TEMP] != INT16_MIN);
    
    // Parameter definitions for HTTP server
    typedef struct {
        uint16_t reg_addr;
        const char *name;
        mqtt_subtopic_t subtopic;
    } http_param_t;
    
    // Define all parameters we want to display
    static const http_param_t http_params[] = {
        {MB_INPUT_STATUS, "Status", MQTT_SUB_SYS},
        {MB_INPUT_EXTENDED_DATA, "Extended Data", MQTT_SUB_SYS},
        {MB_INPUT_MAIN_INLET_TEMP, "Main Inlet", MQTT_SUB_TEMP},
        {MB_INPUT_MAIN_OUTLET_TEMP, "Main Outlet", MQTT_SUB_TEMP},
        {MB_INPUT_MAIN_TARGET_TEMP, "Main Target", MQTT_SUB_TEMP},
        {MB_INPUT_DHW_TEMP, "DHW", MQTT_SUB_TEMP},
        {MB_INPUT_DHW_TARGET_TEMP, "DHW Target", MQTT_SUB_TEMP},
        {MB_INPUT_OUTSIDE_TEMP, "Outside", MQTT_SUB_TEMP},
        {MB_INPUT_ROOM_THERMOSTAT_TEMP, "Room Thermostat", MQTT_SUB_TEMP},
        {MB_INPUT_BUFFER_TEMP, "Buffer", MQTT_SUB_TEMP},
        {MB_INPUT_SOLAR_TEMP, "Solar", MQTT_SUB_TEMP},
        {MB_INPUT_POOL_TEMP, "Pool", MQTT_SUB_TEMP},
        {MB_INPUT_MAIN_HEX_OUTLET_TEMP, "Main HEX Outlet", MQTT_SUB_TEMP},
        {MB_INPUT_DISCHARGE_TEMP, "Discharge", MQTT_SUB_TEMP},
        {MB_INPUT_INSIDE_PIPE_TEMP, "Inside Pipe", MQTT_SUB_TEMP},
        {MB_INPUT_DEFROST_TEMP, "Defrost", MQTT_SUB_TEMP},
        {MB_INPUT_EVA_OUTLET_TEMP, "EVA Outlet", MQTT_SUB_TEMP},
        {MB_INPUT_BYPASS_OUTLET_TEMP, "Bypass Outlet", MQTT_SUB_TEMP},
        {MB_INPUT_IPM_TEMP, "IPM", MQTT_SUB_TEMP},
        {MB_INPUT_OUTSIDE_PIPE_TEMP, "Outside Pipe", MQTT_SUB_TEMP},
        {MB_INPUT_Z1_ROOM_TEMP, "Z1 Room", MQTT_SUB_TEMP},
        {MB_INPUT_Z2_ROOM_TEMP, "Z2 Room", MQTT_SUB_TEMP},
        {MB_INPUT_Z1_WATER_TEMP, "Z1 Water", MQTT_SUB_TEMP},
        {MB_INPUT_Z2_WATER_TEMP, "Z2 Water", MQTT_SUB_TEMP},
        {MB_INPUT_Z1_WATER_TARGET_TEMP, "Z1 Water Target", MQTT_SUB_TEMP},
        {MB_INPUT_Z2_WATER_TARGET_TEMP, "Z2 Water Target", MQTT_SUB_TEMP},
        {MB_INPUT_SECOND_INLET_TEMP, "Second Inlet", MQTT_SUB_TEMP},
        {MB_INPUT_ECONOMIZER_OUTLET_TEMP, "Economizer Outlet", MQTT_SUB_TEMP},
        {MB_INPUT_SECOND_ROOM_THERMO_TEMP, "Second Room Thermo", MQTT_SUB_TEMP},
        {MB_INPUT_Z1_HEAT_REQUEST_TEMP, "Z1 Heat Request", MQTT_SUB_TEMP},
        {MB_INPUT_Z1_COOL_REQUEST_TEMP, "Z1 Cool Request", MQTT_SUB_TEMP},
        {MB_INPUT_Z2_HEAT_REQUEST_TEMP, "Z2 Heat Request", MQTT_SUB_TEMP},
        {MB_INPUT_Z2_COOL_REQUEST_TEMP, "Z2 Cool Request", MQTT_SUB_TEMP},
        {MB_INPUT_HEAT_POWER_PRODUCTION, "Heat Production", MQTT_SUB_POWER},
        {MB_INPUT_HEAT_POWER_CONSUMPTION, "Heat Consumption", MQTT_SUB_POWER},
        {MB_INPUT_COOL_POWER_PRODUCTION, "Cool Production", MQTT_SUB_POWER},
        {MB_INPUT_COOL_POWER_CONSUMPTION, "Cool Consumption", MQTT_SUB_POWER},
        {MB_INPUT_DHW_POWER_PRODUCTION, "DHW Production", MQTT_SUB_POWER},
        {MB_INPUT_DHW_POWER_CONSUMPTION, "DHW Consumption", MQTT_SUB_POWER},
        {MB_INPUT_COMPRESSOR_FREQ, "Compressor Frequency", MQTT_SUB_FREQ},
        {MB_INPUT_PUMP_FLOW, "Pump Flow", MQTT_SUB_FLOW},
        {MB_INPUT_OPERATIONS_HOURS, "Operations Hours", MQTT_SUB_HOUR},
        {MB_INPUT_OPERATIONS_COUNTER, "Operations Counter", MQTT_SUB_COUNT},
        {MB_INPUT_FAN1_MOTOR_SPEED, "Fan 1 Speed", MQTT_SUB_SPEED},
        {MB_INPUT_FAN2_MOTOR_SPEED, "Fan 2 Speed", MQTT_SUB_SPEED},
        {MB_INPUT_HIGH_PRESSURE, "High Pressure", MQTT_SUB_PRESS},
        {MB_INPUT_PUMP_SPEED, "Pump Speed", MQTT_SUB_SPEED},
        {MB_INPUT_LOW_PRESSURE, "Low Pressure", MQTT_SUB_PRESS},
        {MB_INPUT_COMPRESSOR_CURRENT, "Compressor Current", MQTT_SUB_CURRENT},
        {MB_INPUT_PUMP_DUTY, "Pump Duty", MQTT_SUB_DUTY},
        {MB_INPUT_MAX_PUMP_DUTY, "Max Pump Duty", MQTT_SUB_DUTY},
        {MB_INPUT_HEATPUMP_STATE, "Heat Pump State", MQTT_SUB_STATE},
        {MB_INPUT_FORCE_DHW_STATE, "Force DHW", MQTT_SUB_STATE},
        {MB_INPUT_OPERATING_MODE_STATE, "Operating Mode", MQTT_SUB_STATE},
        {MB_INPUT_QUIET_MODE_SCHEDULE, "Quiet Mode Schedule", MQTT_SUB_STATE},
        {MB_INPUT_POWERFUL_MODE_TIME, "Powerful Mode Time", MQTT_SUB_STATE},
        {MB_INPUT_QUIET_MODE_LEVEL, "Quiet Mode Level", MQTT_SUB_STATE},
        {MB_INPUT_HOLIDAY_MODE_STATE, "Holiday Mode", MQTT_SUB_STATE},
        {MB_INPUT_THREE_WAY_VALVE_STATE, "Three-Way Valve", MQTT_SUB_STATE},
        {MB_INPUT_DEFROSTING_STATE, "Defrosting", MQTT_SUB_STATE},
        {MB_INPUT_MAIN_SCHEDULE_STATE, "Main Schedule", MQTT_SUB_STATE},
        {MB_INPUT_ZONES_STATE, "Zones", MQTT_SUB_STATE},
        {MB_INPUT_DHW_HEATER_STATE, "DHW Heater", MQTT_SUB_STATE},
        {MB_INPUT_ROOM_HEATER_STATE, "Room Heater", MQTT_SUB_STATE},
        {MB_INPUT_INTERNAL_HEATER_STATE, "Internal Heater", MQTT_SUB_STATE},
        {MB_INPUT_EXTERNAL_HEATER_STATE, "External Heater", MQTT_SUB_STATE},
        {MB_INPUT_FORCE_HEATER_STATE, "Force Heater", MQTT_SUB_STATE},
        {MB_INPUT_STERILIZATION_STATE, "Sterilization", MQTT_SUB_STATE},
        {MB_INPUT_STERILIZATION_TEMP, "Sterilization Temp", MQTT_SUB_TEMP},
        {MB_INPUT_STERILIZATION_MAX_TIME, "Sterilization Max Time", MQTT_SUB_HOUR},
        {MB_INPUT_DHW_HEAT_DELTA, "DHW Heat Delta", MQTT_SUB_TEMP},
        {MB_INPUT_HEAT_DELTA, "Heat Delta", MQTT_SUB_TEMP},
        {MB_INPUT_COOL_DELTA, "Cool Delta", MQTT_SUB_TEMP},
        {MB_INPUT_DHW_HOLIDAY_SHIFT_TEMP, "DHW Holiday Shift", MQTT_SUB_TEMP},
        {MB_INPUT_ROOM_HOLIDAY_SHIFT_TEMP, "Room Holiday Shift", MQTT_SUB_TEMP},
        {MB_INPUT_BUFFER_TANK_DELTA, "Buffer Tank Delta", MQTT_SUB_TEMP},
        {MB_INPUT_HEATING_MODE, "Heating Mode", MQTT_SUB_STATE},
        {MB_INPUT_HEATING_OFF_OUTDOOR_TEMP, "Heating Off Outdoor", MQTT_SUB_TEMP},
        {MB_INPUT_HEATER_ON_OUTDOOR_TEMP, "Heater On Outdoor", MQTT_SUB_TEMP},
        {MB_INPUT_HEAT_TO_COOL_TEMP, "Heat to Cool", MQTT_SUB_TEMP},
        {MB_INPUT_COOL_TO_HEAT_TEMP, "Cool to Heat", MQTT_SUB_TEMP},
        {MB_INPUT_COOLING_MODE, "Cooling Mode", MQTT_SUB_STATE},
        {MB_INPUT_BUFFER_INSTALLED, "Buffer Installed", MQTT_SUB_SYS},
        {MB_INPUT_DHW_INSTALLED, "DHW Installed", MQTT_SUB_SYS},
        {MB_INPUT_SOLAR_MODE, "Solar Mode", MQTT_SUB_STATE},
        {MB_INPUT_SOLAR_ON_DELTA, "Solar On Delta", MQTT_SUB_TEMP},
        {MB_INPUT_SOLAR_OFF_DELTA, "Solar Off Delta", MQTT_SUB_TEMP},
        {MB_INPUT_SOLAR_FROST_PROTECTION, "Solar Frost Protection", MQTT_SUB_TEMP},
        {MB_INPUT_SOLAR_HIGH_LIMIT, "Solar High Limit", MQTT_SUB_TEMP},
        {MB_INPUT_PUMP_FLOWRATE_MODE, "Pump Flowrate Mode", MQTT_SUB_STATE},
        {MB_INPUT_LIQUID_TYPE, "Liquid Type", MQTT_SUB_SYS},
        {MB_INPUT_ALT_EXTERNAL_SENSOR, "Alt External Sensor", MQTT_SUB_SYS},
        {MB_INPUT_ANTI_FREEZE_MODE, "Anti-Freeze Mode", MQTT_SUB_STATE},
        {MB_INPUT_OPTIONAL_PCB, "Optional PCB", MQTT_SUB_SYS},
        {MB_INPUT_Z1_SENSOR_SETTINGS, "Z1 Sensor Settings", MQTT_SUB_SYS},
        {MB_INPUT_Z2_SENSOR_SETTINGS, "Z2 Sensor Settings", MQTT_SUB_SYS},
        {MB_INPUT_EXTERNAL_PAD_HEATER, "External Pad Heater", MQTT_SUB_STATE},
        {MB_INPUT_WATER_PRESSURE, "Water Pressure", MQTT_SUB_PRESS},
        {MB_INPUT_EXTERNAL_CONTROL, "External Control", MQTT_SUB_STATE},
        {MB_INPUT_EXTERNAL_HEAT_COOL_CONTROL, "External Heat/Cool", MQTT_SUB_STATE},
        {MB_INPUT_EXTERNAL_ERROR_SIGNAL, "External Error", MQTT_SUB_STATE},
        {MB_INPUT_EXTERNAL_COMPRESSOR_CONTROL, "External Compressor", MQTT_SUB_STATE},
        {MB_INPUT_Z2_PUMP_STATE, "Z2 Pump", MQTT_SUB_STATE},
        {MB_INPUT_Z1_PUMP_STATE, "Z1 Pump", MQTT_SUB_STATE},
        {MB_INPUT_TWO_WAY_VALVE_STATE, "Two-Way Valve", MQTT_SUB_STATE},
        {MB_INPUT_THREE_WAY_VALVE_STATE2, "Three-Way Valve 2", MQTT_SUB_STATE},
        {MB_INPUT_Z1_VALVE_PID, "Z1 Valve PID", MQTT_SUB_SYS},
        {MB_INPUT_Z2_VALVE_PID, "Z2 Valve PID", MQTT_SUB_SYS},
        {MB_INPUT_BIVALENT_CONTROL, "Bivalent Control", MQTT_SUB_STATE},
        {MB_INPUT_BIVALENT_MODE, "Bivalent Mode", MQTT_SUB_STATE},
        {MB_INPUT_BIVALENT_START_TEMP, "Bivalent Start Temp", MQTT_SUB_TEMP},
        {MB_INPUT_BIVALENT_ADVANCED_HEAT, "Bivalent Advanced Heat", MQTT_SUB_STATE},
        {MB_INPUT_BIVALENT_ADVANCED_DHW, "Bivalent Advanced DHW", MQTT_SUB_STATE},
        {MB_INPUT_BIVALENT_ADVANCED_START_TEMP, "Bivalent Advanced Start", MQTT_SUB_TEMP},
        {MB_INPUT_BIVALENT_ADVANCED_STOP_TEMP, "Bivalent Advanced Stop", MQTT_SUB_TEMP},
        {MB_INPUT_BIVALENT_ADVANCED_START_DELAY, "Bivalent Advanced Start Delay", MQTT_SUB_HOUR},
        {MB_INPUT_BIVALENT_ADVANCED_STOP_DELAY, "Bivalent Advanced Stop Delay", MQTT_SUB_HOUR},
        {MB_INPUT_BIVALENT_ADVANCED_DHW_DELAY, "Bivalent Advanced DHW Delay", MQTT_SUB_HOUR},
        {MB_INPUT_HEATER_DELAY_TIME, "Heater Delay Time", MQTT_SUB_HOUR},
        {MB_INPUT_HEATER_START_DELTA, "Heater Start Delta", MQTT_SUB_TEMP},
        {MB_INPUT_HEATER_STOP_DELTA, "Heater Stop Delta", MQTT_SUB_TEMP},
        {MB_INPUT_ERROR_TYPE, "Error Type", MQTT_SUB_ERROR},
        {MB_INPUT_ERROR_NUMBER, "Error Number", MQTT_SUB_ERROR},
        {MB_INPUT_ROOM_HEATER_OPS_HOURS, "Room Heater Ops Hours", MQTT_SUB_HOUR},
        {MB_INPUT_DHW_HEATER_OPS_HOURS, "DHW Heater Ops Hours", MQTT_SUB_HOUR},
        {MB_INPUT_Z1_WATER_PUMP, "Z1 Water Pump", MQTT_SUB_STATE},
        {MB_INPUT_Z1_MIXING_VALVE, "Z1 Mixing Valve", MQTT_SUB_STATE},
        {MB_INPUT_Z2_WATER_PUMP, "Z2 Water Pump", MQTT_SUB_STATE},
        {MB_INPUT_Z2_MIXING_VALVE, "Z2 Mixing Valve", MQTT_SUB_STATE},
        {MB_INPUT_POOL_WATER_PUMP, "Pool Water Pump", MQTT_SUB_STATE},
        {MB_INPUT_SOLAR_WATER_PUMP, "Solar Water Pump", MQTT_SUB_STATE},
        {MB_INPUT_ALARM_STATE, "Alarm State", MQTT_SUB_STATE},
        {MB_INPUT_ADC_AIN, "ADC AIN", MQTT_SUB_SYS},
        {MB_INPUT_ADC_NTC1, "ADC NTC1", MQTT_SUB_TEMP},
        {MB_INPUT_ADC_NTC2, "ADC NTC2", MQTT_SUB_TEMP},
        {MB_INPUT_DS18B20_TEMP,  "DS18B20 #1", MQTT_SUB_TEMP},
        {MB_INPUT_DS18B20_TEMP2, "DS18B20 #2", MQTT_SUB_TEMP},
        {MB_INPUT_DS18B20_TEMP3, "DS18B20 #3", MQTT_SUB_TEMP},
        {MB_INPUT_DS18B20_TEMP4, "DS18B20 #4", MQTT_SUB_TEMP},
        {MB_INPUT_DS18B20_TEMP5, "DS18B20 #5", MQTT_SUB_TEMP},
        {MB_INPUT_DS18B20_TEMP6, "DS18B20 #6", MQTT_SUB_TEMP},
        {MB_INPUT_DS18B20_TEMP7, "DS18B20 #7", MQTT_SUB_TEMP},
        {MB_INPUT_DS18B20_TEMP8, "DS18B20 #8", MQTT_SUB_TEMP},
        {0, NULL, 0} // End marker
    };
    
    // Category mapping
    const char* category_map[] = {
        [MQTT_SUB_SYS] = "🔧 System",
        [MQTT_SUB_TEMP] = "🌡️ Temperatures",
        [MQTT_SUB_FLOW] = "💧 Flow",
        [MQTT_SUB_STATE] = "⚙️ States",
        [MQTT_SUB_POWER] = "⚡ Power",
        [MQTT_SUB_FREQ] = "📊 Frequency",
        [MQTT_SUB_HOUR] = "⏱️ Hours",
        [MQTT_SUB_COUNT] = "🔢 Counters",
        [MQTT_SUB_SPEED] = "🌪️ Speed",
        [MQTT_SUB_PRESS] = "📊 Pressure",
        [MQTT_SUB_CURRENT] = "⚡ Current",
        [MQTT_SUB_DUTY] = "📈 Duty",
        [MQTT_SUB_ERROR] = "⚠️ Errors"
    };
    
    for (size_t i = 0; http_params[i].reg_addr != 0; i++) {
        uint16_t reg_addr = http_params[i].reg_addr;
        const char *name = http_params[i].name;
        mqtt_subtopic_t subtopic = http_params[i].subtopic;
        
        int16_t value = mb_input_registers[reg_addr];
        
        if (value == INT16_MIN) {
            continue; // Skip invalid values
        }
        
        cJSON *param = cJSON_CreateObject();
        cJSON_AddStringToObject(param, "name", name);
        
        char value_str[64];
        const char *unit = "";
        
        switch (subtopic) {
            case MQTT_SUB_TEMP: {
                bool is_x100 =
                    (reg_addr == MB_INPUT_MAIN_INLET_TEMP) ||
                    (reg_addr == MB_INPUT_MAIN_OUTLET_TEMP) ||
                    (reg_addr >= MB_INPUT_DS18B20_TEMP && reg_addr <= MB_INPUT_DS18B20_TEMP8) ||
                    (reg_addr == MB_INPUT_ADC_NTC1) ||
                    (reg_addr == MB_INPUT_ADC_NTC2);
                if (is_x100) {
                    snprintf(value_str, sizeof(value_str), "%.2f", value / 100.0f);
                } else {
                    snprintf(value_str, sizeof(value_str), "%d", value);
                }
                unit = "°C";
                break;
            }
            case MQTT_SUB_POWER:
                snprintf(value_str, sizeof(value_str), "%d", value);
                unit = "W";
                break;
            case MQTT_SUB_FREQ:
                snprintf(value_str, sizeof(value_str), "%d", value);
                unit = "Hz";
                break;
            case MQTT_SUB_FLOW:
                snprintf(value_str, sizeof(value_str), "%d", value);
                unit = "L/min";
                break;
            case MQTT_SUB_SPEED:
                snprintf(value_str, sizeof(value_str), "%d", value);
                unit = "rpm";
                break;
            case MQTT_SUB_PRESS:
                snprintf(value_str, sizeof(value_str), "%d", value);
                unit = "bar";
                break;
            case MQTT_SUB_CURRENT:
                snprintf(value_str, sizeof(value_str), "%d", value);
                unit = "A";
                break;
            case MQTT_SUB_DUTY:
                snprintf(value_str, sizeof(value_str), "%d", value);
                unit = "%";
                break;
            case MQTT_SUB_HOUR:
                snprintf(value_str, sizeof(value_str), "%d", value);
                unit = "H";
                break;
            default:
                snprintf(value_str, sizeof(value_str), "%d", value);
                break;
        }
        
        cJSON_AddStringToObject(param, "value", value_str);
        cJSON_AddStringToObject(param, "unit", unit);
        if (subtopic < sizeof(category_map) / sizeof(category_map[0])) {
            cJSON_AddStringToObject(param, "category", category_map[subtopic]);
        } else {
            cJSON_AddStringToObject(param, "category", "📊 Other");
        }
        
        cJSON_AddItemToArray(params_array, param);
    }
    
    cJSON_AddItemToObject(json, "params", params_array);
    cJSON_AddStringToObject(json, "status", data_valid ? "online" : "offline");
    
    // System information
    cJSON_AddNumberToObject(json, "uptime", (float)esp_timer_get_time() / (1000000.0f * 3600.0f));
    cJSON_AddNumberToObject(json, "free_memory", (float)(esp_get_free_heap_size() / 1024.0f));
    
    // WiFi information
    wifi_ap_record_t ap_info;
    if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
        cJSON_AddNumberToObject(json, "wifi_rssi", ap_info.rssi);
        cJSON_AddStringToObject(json, "wifi_ssid", (char*)ap_info.ssid);
    } else {
        cJSON_AddNumberToObject(json, "wifi_rssi", 0);
        cJSON_AddStringToObject(json, "wifi_ssid", "Not connected");
    }
    
    // IP address
    char ip_str[32];
    if (wifi_connect_get_ip(ip_str, sizeof(ip_str)) == ESP_OK) {
        cJSON_AddStringToObject(json, "device_ip", ip_str);
    } else {
        cJSON_AddStringToObject(json, "device_ip", "Not available");
    }
    
    char *json_string = cJSON_Print(json);
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_send(req, json_string, strlen(json_string));
    
    free(json_string);
    cJSON_Delete(json);
    
    return ESP_OK;
}

// Initialize HTTP server
esp_err_t http_server_init(void) {
    if (server_handle != NULL) {
        ESP_LOGW(TAG, "HTTP server already initialized");
        return ESP_OK;
    }
    
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 10;
    config.max_open_sockets = 7;
    
    ESP_LOGI(TAG, "Starting HTTP server on port: '%d'", config.server_port);
    
    if (httpd_start(&server_handle, &config) == ESP_OK) {
        // Register URI handlers
        httpd_uri_t root_uri = {
            .uri       = "/",
            .method    = HTTP_GET,
            .handler   = root_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server_handle, &root_uri);
        
        httpd_uri_t json_uri = {
            .uri       = "/json",
            .method    = HTTP_GET,
            .handler   = json_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server_handle, &json_uri);
        
        ESP_LOGI(TAG, "HTTP server started successfully");
        return ESP_OK;
    }
    
    ESP_LOGE(TAG, "Error starting HTTP server");
    return ESP_FAIL;
}

// Start HTTP server
esp_err_t http_server_start(void) {
    return http_server_init();
}

// Stop HTTP server
esp_err_t http_server_stop(void) {
    if (server_handle == NULL) {
        return ESP_OK;
    }
    
    httpd_stop(server_handle);
    server_handle = NULL;
    ESP_LOGI(TAG, "HTTP server stopped");
    return ESP_OK;
}

