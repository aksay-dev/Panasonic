# Отсутствующие поля в Modbus регистрах

## Анализ структуры hp_decoder_data_t vs Modbus регистры

### ❌ Температуры (int8_t) - ОТСУТСТВУЮТ:
- `main_target_temp` (TOP7) - целевая температура основного контура
- `dhw_target_temp` (TOP9) - целевая температура ГВС
- `room_thermostat_temp` (TOP33) - температура комнатного термостата
- `z1_water_target_temp` (TOP42) - целевая температура воды зоны 1
- `z2_water_target_temp` (TOP43) - целевая температура воды зоны 2
- `inside_pipe_temp` (TOP51) - температура внутренней трубы
- `defrost_temp` (TOP52) - температура разморозки
- `bypass_outlet_temp` (TOP54) - температура байпаса
- `second_inlet_temp` (TOP116) - вторичная входная температура
- `economizer_outlet_temp` (TOP117) - температура экономайзера
- `second_room_thermostat_temp` (TOP118) - вторичная комнатная температура

### ❌ Температуры зон и кривые нагрева/охлаждения (int8_t) - ВСЕ ОТСУТСТВУЮТ:
- `z1_heat_request_temp` (TOP27)
- `z1_cool_request_temp` (TOP28)
- `z2_heat_request_temp` (TOP34)
- `z2_cool_request_temp` (TOP35)
- `z1_heat_curve_target_high_temp` (TOP29)
- `z1_heat_curve_target_low_temp` (TOP30)
- `z1_heat_curve_outside_high_temp` (TOP31)
- `z1_heat_curve_outside_low_temp` (TOP32)
- `z1_cool_curve_target_high_temp` (TOP72)
- `z1_cool_curve_target_low_temp` (TOP73)
- `z1_cool_curve_outside_high_temp` (TOP74)
- `z1_cool_curve_outside_low_temp` (TOP75)
- `z2_heat_curve_target_high_temp` (TOP82)
- `z2_heat_curve_target_low_temp` (TOP83)
- `z2_heat_curve_outside_high_temp` (TOP84)
- `z2_heat_curve_outside_low_temp` (TOP85)
- `z2_cool_curve_target_high_temp` (TOP86)
- `z2_cool_curve_target_low_temp` (TOP87)
- `z2_cool_curve_outside_high_temp` (TOP88)
- `z2_cool_curve_outside_low_temp` (TOP89)

### ❌ Технические параметры - ОТСУТСТВУЮТ:
- `operations_hours` (TOP11) - часы работы
- `operations_counter` (TOP12) - счетчик операций
- `fan1_motor_speed` (TOP62) - скорость вентилятора 1
- `fan2_motor_speed` (TOP63) - скорость вентилятора 2
- `high_pressure` (TOP64, int16_t*100) - высокое давление
- `low_pressure` (TOP66, int16_t*100) - низкое давление
- `compressor_current` (TOP67, int16_t*100) - ток компрессора

### ❌ Нагреватели и стерилизация - ОТСУТСТВУЮТ:
- `external_heater_state` (TOP61)
- `force_heater_state` (TOP68)
- `sterilization_state` (TOP69)
- `sterilization_temp` (TOP70, int16_t*100)
- `sterilization_max_time` (TOP71)

### ❌ Дельты и смещения (int16_t*100) - ВСЕ ОТСУТСТВУЮТ:
- `dhw_heat_delta` (TOP22)
- `heat_delta` (TOP23)
- `cool_delta` (TOP24)
- `dhw_holiday_shift_temp` (TOP25)
- `room_holiday_shift_temp` (TOP45)
- `buffer_tank_delta` (TOP113)

### ❌ Режимы отопления/охлаждения - ВСЕ ОТСУТСТВУЮТ:
- `heating_mode` (TOP76)
- `heating_off_outdoor_temp` (TOP77, int16_t*100)
- `heater_on_outdoor_temp` (TOP78, int16_t*100)
- `heat_to_cool_temp` (TOP79, int16_t*100)
- `cool_to_heat_temp` (TOP80, int16_t*100)
- `cooling_mode` (TOP81)

### ❌ Солнечные и буферные настройки - ВСЕ ОТСУТСТВУЮТ:
- `buffer_installed` (TOP99)
- `dhw_installed` (TOP100)
- `solar_mode` (TOP101)
- `solar_on_delta` (TOP102, int16_t*100)
- `solar_off_delta` (TOP103, int16_t*100)
- `solar_frost_protection` (TOP104, int16_t*100)
- `solar_high_limit` (TOP105, int16_t*100)

### ❌ Настройки насоса и жидкости - ОТСУТСТВУЮТ:
- `liquid_type` (TOP107)
- `alt_external_sensor` (TOP108)
- `anti_freeze_mode` (TOP109)
- `optional_pcb` (TOP110)
- `z1_sensor_settings` (TOP111)
- `z2_sensor_settings` (TOP112)

### ❌ Внешние управления - ВСЕ ОТСУТСТВУЮТ:
- `external_pad_heater` (TOP114)
- `water_pressure` (TOP115, int16_t*100)
- `external_control` (TOP119)
- `external_heat_cool_control` (TOP120)
- `external_error_signal` (TOP121)
- `external_compressor_control` (TOP122)

### ❌ Клапаны - ОТСУТСТВУЮТ:
- `two_way_valve_state` (TOP125)
- `three_way_valve_state2` (TOP126)
- `z1_valve_pid` (TOP127, int16_t*100)
- `z2_valve_pid` (TOP128, int16_t*100)

### ❌ Бивалентные настройки - ВСЕ ОТСУТСТВУЮТ:
- `bivalent_control` (TOP129)
- `bivalent_mode` (TOP130)
- `bivalent_start_temp` (TOP131, int16_t*100)
- `bivalent_advanced_heat` (TOP132)
- `bivalent_advanced_dhw` (TOP133)
- `bivalent_advanced_start_temp` (TOP134, int16_t*100)
- `bivalent_advanced_stop_temp` (TOP135, int16_t*100)
- `bivalent_advanced_start_delay` (TOP136)
- `bivalent_advanced_stop_delay` (TOP137)
- `bivalent_advanced_dhw_delay` (TOP138)

### ❌ Настройки времени нагревателя - ОТСУТСТВУЮТ:
- `heater_delay_time` (TOP96)
- `heater_start_delta` (TOP97, int16_t*100)
- `heater_stop_delta` (TOP98, int16_t*100)

### ❌ Ошибки и модель - ОТСУТСТВУЮТ:
- `error_state` (TOP44, char[16])
- `heat_pump_model` (TOP92, char[32])

### ❌ Часы работы - ОТСУТСТВУЮТ:
- `room_heater_operations_hours` (TOP90)
- `dhw_heater_operations_hours` (TOP91)

### ❌ Дополнительные данные энергопотребления (XTOP) - ВСЕ ОТСУТСТВУЮТ:
- `heat_power_consumption_extra` (XTOP0)
- `cool_power_consumption_extra` (XTOP1)
- `dhw_power_consumption_extra` (XTOP2)
- `heat_power_production_extra` (XTOP3)
- `cool_power_production_extra` (XTOP4)
- `dhw_power_production_extra` (XTOP5)

### ❌ Дополнительные опциональные данные (OPT) - ОТСУТСТВУЮТ:
- `z1_mixing_valve` (OPT1)
- `z2_mixing_valve` (OPT3)
- `alarm_state` (OPT6)

### ❌ Другие состояния - ОТСУТСТВУЮТ:
- `quiet_mode_schedule` (TOP3)

---

## ✅ Поля, которые УЖЕ ЕСТЬ в Modbus (45 полей):

### Температуры (11):
- main_inlet_temp, main_outlet_temp
- dhw_temp, outside_temp, buffer_temp, solar_temp, pool_temp
- z1_temp, z2_temp, z1_water_temp, z2_water_temp
- discharge_temp, eva_outlet_temp, main_hex_outlet_temp, ipm_temp, outside_pipe_temp

### Мощность (6):
- heat_power_production, heat_power_consumption
- cool_power_production, cool_power_consumption
- dhw_power_production, dhw_power_consumption

### Состояния (11):
- heatpump_state, operating_mode_state, quiet_mode_level, powerful_mode_time
- holiday_mode_state, force_dhw_state, defrosting_state, three_way_valve_state
- zones_state, main_schedule_state
- dhw_heater_state, room_heater_state, internal_heater_state

### Технические (6):
- compressor_freq, pump_flow, pump_speed, pump_duty, max_pump_duty, pump_flowrate_mode

### Насосы и клапаны (6):
- z1_water_pump, z2_water_pump, pool_water_pump, solar_water_pump
- z1_pump_state, z2_pump_state

### Системные (5):
- FREE_MEMORY, UPTIME, STATUS, LAST_UPDATE (системные регистры, не из decoder)

---

## 📊 ИТОГО:

- **Всего полей в hp_decoder_data_t**: ~149 полей (без учета data_valid и last_update_time)
- **Представлено в Modbus**: ~40 полей из decoder + ~5 системных = 45 регистров
- **ОТСУТСТВУЕТ**: ~109 полей (**73% данных НЕ представлены в Modbus**)

---

## 🎯 Рекомендации:

1. **Приоритетные для добавления**:
   - Ошибки и модель: `error_state`, `heat_pump_model`
   - Операционные часы: `operations_hours`, `operations_counter`
   - Технические: `fan1_motor_speed`, `fan2_motor_speed`, `high_pressure`, `low_pressure`, `compressor_current`
   - Целевые температуры: `main_target_temp`, `dhw_target_temp`, `z1_water_target_temp`, `z2_water_target_temp`
   - Давление воды: `water_pressure`

2. **Расширенные (если нужны)**:
   - Кривые нагрева/охлаждения (20 параметров)
   - Настройки солнечных коллекторов (7 параметров)
   - Бивалентные настройки (10 параметров)
   - Дельты и смещения (6 параметров)

3. **Специфические**:
   - Дополнительные данные энергопотребления (XTOP, 6 параметров)
   - Настройки стерилизации (3 параметра)

