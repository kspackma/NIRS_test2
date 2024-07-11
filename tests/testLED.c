// testLED script
#include "esp_mac.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "portmacro.h"
#include "registers.h"
#include "register_values1.h"
#include "esp_rom_sys.h"
#include "sdkconfig.h"
#include <string.h>
#include <spi_types.h>

#define SCLK 6
#define SDOUT 2
#define SDIN 7
#define SEN_AFE1 9
#define SEN_AFE2 10
#define ADC_RDY 5
#define RESETZ 8

#define VSPI_HOST SPI3_HOST

// Initialize flags ---------------------------------------------------------------------------
bool useAFE1 = true;
bool useSequence1 = true;
int pdCount = 0;
bool dataReady = false;

// Function Prototypes ------------------------------------------------------------------------
void write_LED(spi_device_handle_t handle);
void init_pins(void);
void setup_AFE(spi_device_handle_t handle);
void select_AFE(bool useAFE1);
void writeRegisterBits(spi_device_handle_t handle, uint8_t address, uint32_t value, uint32_t mask);
uint32_t readRegisterBits(spi_device_handle_t handle, uint8_t address);
void IRAM_ATTR onDataReady(void* arg);
void executeSequence(spi_device_handle_t handle);
void updateCounters(void);
void reset_LED_times(spi_device_handle_t handle);
void reset_PD_times(spi_device_handle_t handle);
void reset_PD_select(spi_device_handle_t handle);
void setGain(spi_device_handle_t handle);

// Main application entry point ---------------------------------------------------------------
void app_main(void) {
    spi_device_handle_t handle;  // SPI device handle

    // Initialize GPIO pins
    init_pins();

    // Initialize SPI communication
    spi_bus_config_t buscfg = {
        .mosi_io_num = SDIN,   // SPI MOSI pin
        .miso_io_num = SDOUT,  // SPI MISO pin
        .sclk_io_num = SCLK,   // SPI Clock pin
        .quadwp_io_num = -1,   // Not used
        .quadhd_io_num = -1    // Not used
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1000000,  // Clock out at 1 MHz
        .mode = 0,                  // SPI mode 0
        .spics_io_num = -1,         // No CS pin, we'll control it manually
        .queue_size = 7             // Queue size
    };

    // Initialize SPI bus
    spi_bus_initialize(VSPI_HOST, &buscfg, 0);
    // Add SPI device
    spi_bus_add_device(VSPI_HOST, &devcfg, &handle);

    // Setup AFE
    select_AFE(true);    // Select AFE1
    setup_AFE(handle);   // Setup AFE1
    select_AFE(false);   // Select AFE2
    setup_AFE(handle);   // Setup AFE2



// Main Loop ---------------------------------------------------------

}

// Implementation of init_pins ----------------------------------------------------------------
void init_pins(void) {
    gpio_config_t io_conf;

    // Configure SEN_AFE1 and SEN_AFE2 as output
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << SEN_AFE1) | (1ULL << SEN_AFE2);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    // Configure ADC_RDY as input
    io_conf.intr_type = GPIO_INTR_POSEDGE; // Rising edge interrupt
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << ADC_RDY);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    // Configure RESETZ as output
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << RESETZ);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    // Install GPIO ISR handler
    gpio_install_isr_service(0);
    gpio_isr_handler_add(ADC_RDY, onDataReady, NULL);  // Add ISR handler
}
// Implementation of setup_AFE ----------------------------------------------------------------
void setup_AFE(spi_device_handle_t handle) { 
    // Write values to registers (Address, Value, Mask) 

    // Reset AFE modules: pulse resetz pin high for 30 microseconds (must be reset before valid operation)
    gpio_set_level(RESETZ, 1); 
    esp_rom_delay_us(30);
    gpio_set_level(RESETZ, 0);

    // Set LED Driver for 50mA (TX_REF = 0.15V)
    writeRegisterBits(handle, SETUP_4, 0x01 << 17, MASK_ILED_FS);    // ILED_FS = 1

    // Set current for each LED
    writeRegisterBits(handle, SETUP_3, 0x3F, MASK_ILED1_MSB);       // LED1 current 1111xx
    writeRegisterBits(handle, SETUP_3, 0x3F << 6, MASK_ILED2_MSB);  // LED2 current 1111xx
    writeRegisterBits(handle, SETUP_3, 0x3F << 12, MASK_ILED3_MSB); // LED3 current 1111xx
    writeRegisterBits(handle, SETUP_5, 0x3F << 11, MASK_ILED4_MSB); // LED4 current 1111xx

    writeRegisterBits(handle, SETUP_3, 0x03 << 18, MASK_ILED1_LSB); // LED1 current xxxx11
    writeRegisterBits(handle, SETUP_3, 0x03 << 20, MASK_ILED2_LSB); // LED2 current xxxx11
    writeRegisterBits(handle, SETUP_3, 0x03 << 22, MASK_ILED3_LSB); // LED3 current xxxx11
    writeRegisterBits(handle, SETUP_5, 0x03 << 9, MASK_ILED4_LSB);  // LED4 current xxxx11

    // Set default TIA gain
    setGain(handle); // Gain setting for PD1

    // LED, Sample, Convert times: 
    // LED 1 VALUES
    writeRegisterBits(handle, LED1LEDSTC, LED1LEDSTC_VAL, 0xFFFFFFFF);  // Initialize LED 1 with actual values (default)
    writeRegisterBits(handle, LED1LEDENDC, LED1LEDENDC_VAL, 0xFFFFFFFF);
    // LED 2 VALUES
    writeRegisterBits(handle, LED2LEDSTC, LED2LEDSTC_VAL, 0xFFFFFFFF);  // Initialize LED 2 with actual values (default)
    writeRegisterBits(handle, LED2LEDENDC, LED2LEDENDC_VAL, 0xFFFFFFFF); 
    // LED 3 VALUES
    writeRegisterBits(handle, LED3STC, 0x0, 0xFFFFFFFF);   // Initialize LED 3 times with cleared values 
    writeRegisterBits(handle, LED3ENDC, 0x0, 0xFFFFFFFF);
    // LED 4 VALUES
    writeRegisterBits(handle, LED4STC, 0x0, 0xFFFFFFFF);   // Initialize LED 4 times with cleared values
    writeRegisterBits(handle, LED4ENDC, 0x0, 0xFFFFFFFF);
    // LED 1 and 3 sample times
    writeRegisterBits(handle, LED1STC, LED1STC_VAL, 0xFFFFFFFF);    
    writeRegisterBits(handle, LED1ENDC, LED1ENDC_VAL, 0xFFFFFFFF);  
    // LED 1 and 3 convert times
    writeRegisterBits(handle, LED1CONVST, LED1CONVST_VAL, 0xFFFFFFFF);
    writeRegisterBits(handle, LED1CONVEND, LED1CONVEND_VAL, 0xFFFFFFFF);
    // LED 2 and 4 sample times
    writeRegisterBits(handle, LED2STC, LED2STC_VAL, 0xFFFFFFFF);  
    writeRegisterBits(handle, LED2ENDC, LED2ENDC_VAL, 0xFFFFFFFF);
    // LED 2 and 4 convert times
    writeRegisterBits(handle, LED2CONVST, LED2CONVST_VAL, 0xFFFFFFFF);
    writeRegisterBits(handle, LED2CONVEND, LED2CONVEND_VAL, 0xFFFFFFFF);
}
// Implementation of select_AFE ---------------------------------------------------------------
void select_AFE(bool useAFE1) {
    gpio_set_level(SEN_AFE1, !useAFE1); // Active low
    gpio_set_level(SEN_AFE2, useAFE1);
}

// Implementation of writeRegisterBits --------------------------------------------------------
void writeRegisterBits(spi_device_handle_t handle, uint8_t address, uint32_t value, uint32_t mask) {
    // SPI transaction code to write register bits
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));  // Zero out the transaction

    // Step 0: Write 0 to bit 0 of register 00h to disable SPI register read mode
    uint8_t disable_spi_reg_read[] = {0x40 | SETUP_1, 0x00, 0x00, 0x00};
    t.length = 8 * sizeof(disable_spi_reg_read);  // Command is 8-bit address + 24-bit data
    t.tx_buffer = disable_spi_reg_read;
    spi_device_transmit(handle, &t);

    // Step 1: Enable SPI register read mode by setting the SPI_REG_READ bit
    uint8_t enable_spi_reg_read[] = {0x40 | SETUP_1, 0x00, 0x00, MASK_SPI_REG_READ};
    t.length = 8 * sizeof(enable_spi_reg_read);  // Command is 8-bit address + 24-bit data
    t.tx_buffer = enable_spi_reg_read;
    spi_device_transmit(handle, &t);

    // Step 2: Read the current register value
    uint8_t read_cmd[] = {0x80 | address, 0x00, 0x00, 0x00};
    uint8_t read_data[3] = {0};
    t.length = 8 * sizeof(read_cmd);
    t.tx_buffer = read_cmd;
    t.rxlength = 8 * sizeof(read_data);
    t.rx_buffer = read_data;
    spi_device_transmit(handle, &t);
    uint32_t currentValue = (read_data[0] << 16) | (read_data[1] << 8) | read_data[2];

    // Step 3: Modify the bits in the register value
    currentValue = (currentValue & ~mask) | (value & mask);

    // Step 4: Write the new register value
    uint8_t write_data[] = {0x40 | address, (currentValue >> 16) & 0xFF, (currentValue >> 8) & 0xFF, currentValue & 0xFF};
    t.length = 8 * sizeof(write_data);
    t.tx_buffer = write_data;
    spi_device_transmit(handle, &t);
}

// Implementation of readRegisterBits ---------------------------------------------------------
uint32_t readRegisterBits(spi_device_handle_t handle, uint8_t address) {
    // SPI transaction code to read register bits
    uint32_t data = 0;  // Variable to store the incoming data

    // Step 0: Write 1 to bit 0 of register 00h to enable SPI register read mode
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));  // Zero out the transaction
    uint8_t enable_spi_reg_read[] = {0x40 | SETUP_1, 0x00, 0x00, 0x01};
    t.length = 8 * sizeof(enable_spi_reg_read);
    t.tx_buffer = enable_spi_reg_read;
    spi_device_transmit(handle, &t);

    // Step 1: Read the register value
    uint8_t read_cmd[] = {0x80 | address, 0x00, 0x00, 0x00};
    uint8_t read_data[3] = {0};
    t.length = 8 * sizeof(read_cmd);
    t.tx_buffer = read_cmd;
    t.rxlength = 8 * sizeof(read_data);
    t.rx_buffer = read_data;
    spi_device_transmit(handle, &t);

    data = (read_data[0] << 16) | (read_data[1] << 8) | read_data[2];

    return data;
}

// ISR for ADC_RDY
void IRAM_ATTR onDataReady(void* arg) {
    dataReady = true;
}

// Function to set the gain
void setGain(spi_device_handle_t handle) {
        // Write 0 (500kohm) to Rf register
    writeRegisterBits(handle, TIA_SETUP_2, 0x00, MASK_TIA_GAIN_LSB);
    // Write 4 (20pF) to Cf register
    writeRegisterBits(handle, TIA_SETUP_2, 0x04 << 3, MASK_TIA_CF);
}

// Implementation of reset_LED_times
void reset_LED_times(spi_device_handle_t handle) {
    // Reset LED timing registers
    writeRegisterBits(handle, LED1LEDSTC, 0x0, 0xFFFFFFFF);   // Reset LED 1 timing registers
    writeRegisterBits(handle, LED1LEDENDC, 0x0, 0xFFFFFFFF);
    writeRegisterBits(handle, LED2LEDSTC, 0x0, 0xFFFFFFFF);   // Reset LED 2 timing registers
    writeRegisterBits(handle, LED2LEDENDC, 0x0, 0xFFFFFFFF);
    writeRegisterBits(handle, LED3LEDSTC, 0x0, 0xFFFFFFFF);   // Reset LED 3 timing registers
    writeRegisterBits(handle, LED3LEDENDC, 0x0, 0xFFFFFFFF);
    writeRegisterBits(handle, LED4LEDSTC, 0x0, 0xFFFFFFFF);   // Reset LED 4 timing registers
    writeRegisterBits(handle, LED4LEDENDC, 0x0, 0xFFFFFFFF);  
}

// Implementation of reset_PD_times
void reset_PD_times(spi_device_handle_t handle) {
    // Reset PD timing registers
    writeRegisterBits(handle, TG_PD1STC, 0x0, 0xFFFFFFFF); // Reset PD 1 timing registers
    writeRegisterBits(handle, TG_PD1ENDC, 0x0, 0xFFFFFFFF);
    writeRegisterBits(handle, TG_PD2STC, 0x0, 0xFFFFFFFF); // Reset PD 2 timing registers
    writeRegisterBits(handle, TG_PD2ENDC, 0x0, 0xFFFFFFFF);
    writeRegisterBits(handle, TG_PD3STC, 0x0, 0xFFFFFFFF); // Reset PD 3 timing registers
    writeRegisterBits(handle, TG_PD3ENDC, 0x0, 0xFFFFFFFF);     
}

// Implementation of reset_PD_select
void reset_PD_select(spi_device_handle_t handle) {
    // Reset PD mode select registers
    writeRegisterBits(handle, PD_ENABLE, 0x0, MASK_DUAL_PD_ENABLE);
    writeRegisterBits(handle, PD_ENABLE, 0x0, MASK_TRIPLE_PD_ENABLE);
}

void write_LED(spi_device_handle_t handle) {
    reset_LED_times(handle); // Reset start and end times for all LEDs
    writeRegisterBits(handle, LED1LEDSTC, LED1STC_VAL, 0xFFFFFFFF);
    writeRegisterBits(handle, LED1LEDENDC, 0x22, 0xFFFFFFFF); // increased the on time from default
}