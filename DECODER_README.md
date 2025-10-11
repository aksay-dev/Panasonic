# Heat Pump Data Decoder

Этот модуль предоставляет функциональность для декодирования данных теплового насоса Panasonic Aquarea, основанную на реализации HeishaMon.

## Обзор

Decoder модуль состоит из двух основных файлов:
- `decoder.h` - заголовочный файл с определениями структур и функций
- `decoder.c` - реализация функций декодирования

## Основные возможности

### 1. Декодирование основных данных (Main Data)
- 139 топиков данных (TOP0-TOP138)
- Температуры, мощности, состояния работы
- Режимы работы, настройки зон
- Информация об ошибках и модели

### 2. Декодирование дополнительных данных (Extra Data)
- 6 дополнительных топиков (XTOP0-XTOP5)
- Дополнительные данные о потреблении и производстве энергии

### 3. Декодирование опциональных данных (Optional Data)
- 7 опциональных топиков (OPT0-OPT6)
- Состояния насосов и клапанов
- Состояние аварийной сигнализации

## Структура данных

### hp_decoder_data_t
Основная структура для хранения всех декодированных данных:

```c
typedef struct {
    // Температуры
    float main_inlet_temp;           // TOP5 - Температура входа основного контура
    float main_outlet_temp;          // TOP6 - Температура выхода основного контура
    float main_target_temp;          // TOP7 - Целевая температура основного контура
    float dhw_temp;                  // TOP10 - Температура ГВС
    float outside_temp;              // TOP14 - Наружная температура
    
    // Мощности
    uint16_t heat_power_production;  // TOP15 - Производство тепловой мощности
    uint16_t heat_power_consumption; // TOP16 - Потребление тепловой мощности
    
    // Состояния работы
    uint8_t operating_mode_state;    // TOP4 - Режим работы
    uint8_t heatpump_state;          // TOP0 - Состояние теплового насоса
    
    // ... и многие другие поля
} hp_decoder_data_t;
```

## API функции

### Инициализация
```c
esp_err_t decoder_init(void);
```

### Декодирование данных
```c
// Основные данные
esp_err_t decoder_decode_main_data(const uint8_t *data, size_t size, hp_decoder_data_t *decoded_data);

// Дополнительные данные
esp_err_t decoder_decode_extra_data(const uint8_t *data, size_t size, hp_decoder_data_t *decoded_data);

// Опциональные данные
esp_err_t decoder_decode_opt_data(const uint8_t *data, size_t size, hp_decoder_data_t *decoded_data);
```

### Получение значений топиков
```c
// Основные топики
esp_err_t decoder_get_topic_value(const uint8_t *data, uint8_t topic_number, char *value, size_t value_size);

// Дополнительные топики
esp_err_t decoder_get_extra_topic_value(const uint8_t *data, uint8_t topic_number, char *value, size_t value_size);

// Опциональные топики
esp_err_t decoder_get_opt_topic_value(const uint8_t *data, uint8_t topic_number, char *value, size_t value_size);
```

### Получение имен и описаний
```c
const char* decoder_get_topic_name(uint8_t topic_number);
const char* decoder_get_extra_topic_name(uint8_t topic_number);
const char* decoder_get_opt_topic_name(uint8_t topic_number);

const char* decoder_get_topic_description(uint8_t topic_number, int value);
const char* decoder_get_extra_topic_description(uint8_t topic_number, int value);
const char* decoder_get_opt_topic_description(uint8_t topic_number, int value);
```

## Интеграция с Protocol

Decoder автоматически интегрирован с модулем protocol.c:

1. При инициализации protocol вызывается `decoder_init()`
2. При получении данных от теплового насоса автоматически вызываются соответствующие функции декодирования
3. Результаты декодирования логируются в консоль

## Пример использования

```c
#include "include/decoder.h"

void example_usage(void) {
    // Инициализация
    esp_err_t ret = decoder_init();
    if (ret != ESP_OK) {
        ESP_LOGE("DECODER", "Failed to initialize decoder");
        return;
    }
    
    // Декодирование основных данных
    uint8_t main_data[203];
    // ... заполнение данными от теплового насоса
    
    hp_decoder_data_t decoded_data;
    ret = decoder_decode_main_data(main_data, sizeof(main_data), &decoded_data);
    if (ret == ESP_OK) {
        ESP_LOGI("DECODER", "Main Inlet Temp: %.2f°C", decoded_data.main_inlet_temp);
        ESP_LOGI("DECODER", "Heat Power Production: %dW", decoded_data.heat_power_production);
    }
    
    // Получение значения конкретного топика
    char value[32];
    ret = decoder_get_topic_value(main_data, 5, value, sizeof(value));
    if (ret == ESP_OK) {
        ESP_LOGI("DECODER", "TOP5 (%s): %s", decoder_get_topic_name(5), value);
    }
}
```

## Тестирование

Включен тестовый файл `test_decoder.c` который демонстрирует:
- Инициализацию decoder
- Декодирование всех типов данных
- Получение значений отдельных топиков
- Работу с именами и описаниями

Тест автоматически запускается при старте приложения.

## Константы

- `NUMBER_OF_TOPICS` = 139 - количество основных топиков
- `NUMBER_OF_TOPICS_EXTRA` = 6 - количество дополнительных топиков  
- `NUMBER_OF_OPT_TOPICS` = 7 - количество опциональных топиков
- `DATASIZE` = 203 - размер основных данных
- `EXTRADATASIZE` = 110 - размер дополнительных данных
- `OPTDATASIZE` = 20 - размер опциональных данных

## Основа

Реализация основана на анализе исходного кода HeishaMon (decode.h/cpp), адаптированного для ESP-IDF и архитектуры ESP32.
