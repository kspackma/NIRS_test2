// this is a test merge
#include "esp_mac.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "portmacro.h"
#include "registers.h"
#include "register_values.h"
#include "esp_rom_sys.h"
#include <string.h>

#define SCLK 6
#define SDOUT 2
#define SDIN 7
#define SEN_AFE1 9
#define SEN_AFE2 10
#define ADC_RDY 5
#define RESETZ 8