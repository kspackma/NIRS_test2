#ifndef REGISTER_VALUES1_H
#define REGISTER_VALUES1_H

// Clock Timing Values ------------------------------------------------------------------------
#define CLK_DIV 0x07

// LED Timing Values --------------------------------------------------------------------------
// **** Note timings for LED 1 = LED 3, and LED 2 = LED 4 since LEDs 3 and 4 replace 1 and 2 in sequence 2
// LED 1 Values
#define LED1STC_VAL       0x09  // Value for LED1 start time
#define LED1ENDC_VAL      0x15  // Value for LED1 end time
#define LED1LEDSTC_VAL    0x0B  // Value for sample LED1 start time
#define LED1LEDENDC_VAL   0x15  // Value for sample LED1 end time
#define LED1CONVST_VAL    0x16  // Value for convert LED1 start time
#define LED1CONVEND_VAL   0x1B  // Value for convert LED1 end time

// LED 2 Values
#define LED2STC_VAL       0x16  // Value for LED2 start time
#define LED2ENDC_VAL      0x22  // Value for LED2 end time
#define LED2LEDSTC_VAL    0x18  // Value for sample LED2 start time
#define LED2LEDENDC_VAL   0x22  // Value for sample LED2 end time
#define LED2CONVST_VAL    0x23  // Value for convert LED2 start time
#define LED2CONVEND_VAL   0x28  // Value for convert LED2 end time

// LED 3 Values
#define LED3STC_VAL       0x09  // Value for LED3 start time
#define LED3ENDC_VAL      0x15  // Value for LED3 end time
#define LED3LEDSTC_VAL    0x0B  // Value for sample LED3 start time
#define LED3LEDENDC_VAL   0x15  // Value for sample LED3 end time
#define LED3CONVST_VAL    0x16  // Value for convert LED3 start time
#define LED3CONVEND_VAL   0x1B  // Value for convert LED3 end time

// LED 4 Values
#define LED4STC_VAL       0x16  // Value for LED2 start time
#define LED4ENDC_VAL      0x22  // Value for LED2 end time
#define LED4LEDSTC_VAL    0x18  // Value for sample LED2 start time
#define LED4LEDENDC_VAL   0x22  // Value for sample LED2 end time
#define LED4CONVST_VAL    0x23  // Value for convert LED2 start time
#define LED4CONVEND_VAL   0x28  // Value for convert LED2 end time

// Ambient 1 Values
#define ALED1STC_VAL      0x25  // Value for sample Ambient1 start time
#define ALED1ENDC_VAL     0x2F  // Value for sample Ambient1 end time
#define ALED1CONVST_VAL   0x30  // Value for convert Ambient1 start time
#define ALED1CONVEND_VAL  0x35  // Value for convert Ambient1 end time

// Ambient 2 Values
#define ALED2STC_VAL      0x32  // Value for sample Ambient2 start time
#define ALED2ENDC_VAL     0x3C  // Value for sample Ambient2 end time
#define ALED2CONVST_VAL   0x3D  // Value for convert Ambient2 start time
#define ALED2CONVEND_VAL  0x42  // Value for convert Ambient2 end time


// Other Timing Values ----------------
#define DATA_RDY_STC_VAL      0x48  // Value for data ready start  
#define DATA_RDY_ENDC_VAL     0x49  
#define DYN_TIA_STC_VAL       0x00
#define DYN_TIA_ENDC_VAL      0x44
#define DYN_ADC_STC_VAL       0x00
#define DYN_ADC_ENDC_VAL      0x44
#define DYN_CLK_STC_VAL       0x00
#define DYN_CLK_ENDC_VAL      0x44
#define DEEP_SLEEP_STC_VAL    0x4E
#define DEEP_SLEEP_ENDC_VAL   0x635

#endif // REGISTER_VALUES1_H