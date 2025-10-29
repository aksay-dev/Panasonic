# Modbus RTU Slave - Инструкция по настройке

## ✅ Реализованные функции

Modbus RTU slave интерфейс успешно реализован и интегрирован в проект на базе **ESP-IDF v5.5.1** с компонентом **esp_modbus**. Реализованы:

### Input Registers (0x0000-0x004F) - Только чтение
- **Системная информация**: статус, время работы
- **Температуры** (0x0010-0x001F): inlet, outlet, DHW, outside, zones, компрессор, испаритель и др.
- **Мощность и энергия** (0x0020-0x002D): производство/потребление тепла/холода/ГВС, частота компрессора, насос
- **Состояния** (0x0030-0x003F): тепловой насос, режимы работы, разморозка, клапаны, нагреватели
- **Зональные насосы** (0x0040-0x0046): состояния насосов зон, бассейна, солнечных панелей

### Holding Registers (0x1000-0x102F) - Чтение/Запись
- **Команды управления** (0x1000-0x100C): вкл/выкл, режимы, принудительные режимы
- **Температурные уставки** (0x1020-0x1024): зоны отопление/охлаждение, ГВС

## 📝 Настройка перед компиляцией

### 0. Установка зависимости esp-modbus

В ESP-IDF v5.5.1 компонент Modbus стал managed component. Он автоматически загрузится при первой компиляции благодаря файлу `main/idf_component.yml`:

```yaml
dependencies:
  espressif/esp-modbus: "^2.0.0"
```

Для принудительной загрузки компонента выполните:
```bash
idf.py reconfigure
```

### 1. Конфигурация через menuconfig

```bash
idf.py menuconfig
```

Перейдите в: **Component config → Modbus configuration → Modbus slave configuration**

#### Основные настройки:

```
[*] Enable Modbus slave support
    Modbus communication mode: Modbus Serial RTU
    Slave address: 7
    UART port number: 1
    UART baud rate: 9600
    UART data bits: 8
    UART stop bits: 1
    UART parity: None
    UART RXD pin: 26
    UART TXD pin: 25
```

#### Опциональные настройки RS485:

Если используется RS485 с автоматическим управлением направлением:
```
    UART RTS pin: 23  (или другой свободный GPIO)
    [*] Enable RS485 mode
```

### 2. Проверка GPIO пинов

**ВАЖНО**: Убедитесь, что выбранные GPIO не конфликтуют с другими функциями:

- **UART2** (тепловой насос): GPIO 16 (RX), GPIO 17 (TX) - **ЗАНЯТО**
- **UART1** (Modbus): GPIO 25 (TX), GPIO 26 (RX) - используется для Modbus

### 3. Компиляция

При первой компиляции IDF Component Manager автоматически загрузит esp-modbus:

```bash
idf.py build
```

Вы увидите сообщение о загрузке managed components:
```
Processing 1 dependencies:
[1/1] espressif/esp-modbus (2.x.x)
```

### 4. Прошивка

```bash
idf.py flash monitor
```

## 🔌 Подключение оборудования

### RS485 преобразователь (MAX485 / MAX3485)

```
ESP32 (UART1)          MAX485              MODBUS Master
GPIO 25 (TX)    ---->  DI                  
GPIO 26 (RX)    <----  RO                  
GPIO 23 (RTS)*  ---->  DE/RE               
GND             <----  GND                 
3.3V            ---->  VCC                 

                       A  <--------------> A (RS485 A)
                       B  <--------------> B (RS485 B)
```

*GPIO 23 (RTS) - опционально, только если включен RS485 mode в menuconfig

### Важные замечания:
- Используйте витую пару для линии RS485
- Максимальная длина линии: до 1200м
- Терминирующий резистор 120 Ом между A и B на концах линии (при длинных кабелях)

## 📊 Карта регистров

### Input Registers (чтение)

| Адрес | Параметр | Тип данных | Описание |
|-------|----------|------------|----------|
| 0x0006 | status | uint16 | Статус (0=офлайн, 1=онлайн) |
| 0x0010 | main_inlet_temp | int16 | Температура на входе, °C×100 |
| 0x0011 | main_outlet_temp | int16 | Температура на выходе, °C×100 |
| 0x0012 | dhw_temp | int16 | Температура ГВС, °C×100 |
| 0x0013 | outside_temp | int16 | Наружная температура, °C×100 |
| 0x0017 | z1_room_temp | int16 | Температура зоны 1, °C×100 |
| 0x0018 | z1_water_temp | int16 | Температура воды зоны 1, °C×100 |
| 0x0020 | heat_power_production | uint16 | Мощность производства тепла |
| 0x0021 | heat_power_consumption | uint16 | Мощность потребления |
| 0x0029 | compressor_freq | uint16 | Частота компрессора, Гц |
| 0x002A | pump_speed | uint16 | Скорость насоса, RPM |
| 0x0030 | heatpump_state | uint16 | Состояние (0/1) |
| 0x0032 | operation_mode | uint16 | Режим работы (0-6) |

### Holding Registers (чтение/запись)

| Адрес | Команда | Диапазон | Описание |
|-------|---------|----------|----------|
| 0x1000 | set_heatpump | 0/1 | Включить/выключить тепловой насос |
| 0x1001 | set_pump | 0/1 | Включить/выключить насос |
| 0x1002 | set_max_pump_duty | 0-100 | Макс. нагрузка насоса, % |
| 0x1003 | set_quiet_mode | 0-3 | Тихий режим |
| 0x1004 | set_powerful_mode | 0-90 | Мощный режим, минут |
| 0x1005 | set_operation_mode | 0-6 | Режим работы |
| 0x1006 | set_holiday_mode | 0/1 | Режим отпуска |
| 0x1007 | set_force_dhw | 0/1 | Принудительный нагрев ГВС |
| 0x1020 | set_z1_heat_temp | °C×100 | Уставка отопления зоны 1 |
| 0x1021 | set_z1_cool_temp | °C×100 | Уставка охлаждения зоны 1 |
| 0x1024 | set_dhw_temp | °C×100 | Уставка температуры ГВС |

## 🧪 Тестирование

### Python (pymodbus)

```python
from pymodbus.client import ModbusSerialClient

# Подключение
client = ModbusSerialClient(
    port='COM3',  # или '/dev/ttyUSB0' в Linux
    baudrate=9600,
    parity='N',
    stopbits=1,
    bytesize=8,
    timeout=1
)

client.connect()

# Чтение температуры на входе (регистр 0x0010)
result = client.read_input_registers(0x0010, 1, slave=7)
if not result.isError():
    temp = result.registers[0] / 100.0
    print(f"Температура на входе: {temp}°C")

# Чтение состояния теплового насоса (регистр 0x0030)
result = client.read_input_registers(0x0030, 1, slave=7)
if not result.isError():
    state = "ВКЛ" if result.registers[0] else "ВЫКЛ"
    print(f"Тепловой насос: {state}")

# Включение теплового насоса (регистр 0x1000)
result = client.write_register(0x1000, 1, slave=7)
print(f"Команда включения: {'OK' if not result.isError() else 'ERROR'}")

# Установка температуры ГВС 50°C (регистр 0x1024)
result = client.write_register(0x1024, 5000, slave=7)  # 50°C × 100
print(f"Установка температуры ГВС: {'OK' if not result.isError() else 'ERROR'}")

client.close()
```

### ModbusPoll / QModMaster

Настройки подключения:
- **Port**: COM3 (или /dev/ttyUSB0)
- **Baud**: 9600
- **Data bits**: 8
- **Parity**: None
- **Stop bits**: 1
- **Slave ID**: 7

Чтение Input Registers:
- Начальный адрес: 16 (0x0010)
- Количество: 16

Запись Holding Register:
- Адрес: 4096 (0x1000)
- Значение: 1 (включить тепловой насос)

## 🔍 Отладка

### Просмотр логов

```bash
idf.py monitor
```

Ищите сообщения с тегами:
- `MODBUS_SLAVE` - события Modbus slave
- `MODBUS_PARAMS` - обновление регистров и обработка команд
- `PROTOCOL` - связь с тепловым насосом
- `DECODER` - декодирование данных

### Типичные проблемы

1. **Modbus не отвечает**
   - Проверьте правильность подключения A/B линий RS485
   - Убедитесь, что slave адрес = 7
   - Проверьте скорость 9600 baud
   - Попробуйте другой терминал Modbus

2. **Неправильные данные**
   - Проверьте, что `g_decoded_data.data_valid = true` в логах
   - Убедитесь, что тепловой насос подключен и передает данные

3. **Команды не выполняются**
   - Смотрите логи `MODBUS_PARAMS` для ошибок валидации
   - Проверьте диапазоны значений команд

## 📚 Дополнительная информация

- Полная документация регистров: `MODBUS_README.md`
- Описание протокола теплового насоса: `README.md`
- Архитектура системы: `ARCHITECTURE.md`

## 🎯 Следующие шаги

1. Настроить menuconfig с правильными GPIO пинами
2. Скомпилировать и прошить проект
3. Подключить RS485 преобразователь
4. Протестировать с помощью ModbusPoll или Python скрипта
5. Интегрировать с вашей SCADA/ПЛК системой

