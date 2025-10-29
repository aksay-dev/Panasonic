<!-- e13fdb71-277e-4ce3-bd7c-8a221b68f7df 735513ee-1311-40f6-b528-3871b10fc2ea -->
# Modbus RTU Slave Implementation Plan

## Обзор

Реализовать Modbus RTU slave на основе ESP-IDF freemodbus компонента для интеграции теплового насоса с SCADA/ПЛК системами.

## Конфигурация

- **UART**: UART1 (UART2 занят тепловым насосом)
- **GPIO**: TX=25, RX=26
- **Скорость**: 9600 baud
- **Slave адрес**: 7
- **Режим**: RTU с поддержкой RS485

## Основные регистры для реализации

### Input Registers (0x0000-0x00FF) - Только чтение

- **0x0010-0x001F**: Температуры (inlet, outlet, DHW, outside, zones, etc.)
- **0x0020-0x002D**: Мощности и энергия (heat/cool production/consumption, compressor freq, pump)
- **0x0030-0x003F**: Состояния (heatpump, pump, operation mode, quiet, powerful, defrost)

### Holding Registers (0x1000-0x102F) - Чтение/Запись

- **0x1000-0x100F**: Команды управления (on/off, modes, force DHW/defrost/sterilization)
- **0x1020-0x1028**: Температурные уставки (zones heat/cool, DHW)

## Файлы для создания

### 1. `main/include/modbus_slave.h`

- Объявления функций инициализации и запуска Modbus
- Константы регистров
- Прототипы обработчиков

### 2. `main/modbus_slave.c`

- Инициализация ESP-IDF Modbus slave
- Настройка UART1 с GPIO 25/26
- Регистрация областей памяти регистров
- Запуск Modbus task
- Обработчики обновления Input Registers из `g_decoded_data`
- Обработчики Holding Registers с вызовом функций из `commands.h`

### 3. `main/include/modbus_params.h`

- Макросы адресов регистров
- Структуры маппинга регистров
- Размеры областей регистров

### 4. `main/modbus_params.c`

- Массивы регистров (input и holding)
- Таблица дескрипторов регистров для freemodbus
- Функции обновления Input Registers из `g_decoded_data`
- Функции обработки записи Holding Registers с вызовом команд

## Интеграция

### Обновить `main/hpc.c`

- Добавить `#include "modbus_slave.h"`
- В `hpc_init()`: вызвать `modbus_slave_init()`
- В `hpc_start()`: вызвать `modbus_slave_start()`

### Обновить `main/CMakeLists.txt`

- Добавить `modbus_slave.c` и `modbus_params.c`
- Добавить зависимость от `freemodbus`

### Создать `main/Kconfig.projbuild`

- Настройки Modbus: UART port, GPIO pins, baud rate, slave address
- Значения по умолчанию: UART1, TX=25, RX=26, 9600, addr=7

## Ключевые моменты реализации

1. **Синхронизация данных**: Input Registers обновляются из `g_decoded_data` периодически в Modbus task
2. **Преобразование единиц**: Температуры умножаются на 100 (25.5°C → 2550), мощности также
3. **Обработка команд**: При записи в Holding Register вызывается соответствующая функция из `commands.h`
4. **Защита**: Валидация диапазонов значений перед вызовом команд
5. **RS485**: RTS контроль через GPIO (если требуется), автоматически управляется драйвером

## Файлы для изменения

- `main/hpc.c` - добавить инициализацию Modbus
- `main/CMakeLists.txt` - добавить новые исходники

## Файлы для создания

- `main/include/modbus.h`
- `main/modbus.c`
- `main/include/modbus_params.h`
- `main/modbus_params.c`
- `main/Kconfig.projbuild`

### To-dos

- [ ] Создать Kconfig.projbuild с настройками Modbus
- [ ] Создать modbus_slave.h и modbus_params.h
- [ ] Реализовать modbus_params.c с маппингом регистров
- [ ] Реализовать modbus_slave.c с инициализацией и обработчиками
- [ ] Интегрировать Modbus в hpc.c и CMakeLists.txt