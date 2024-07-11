#ifndef REGISTERS_H
#define REGISTERS_H

/*
Header file to define the addresses used in the Analog Front End
*/ 

// Define Set-up register addresses and masks -------------------------------------------------
#define SETUP_1 0x00  // address for 0x00 register
    #define MASK_FIFO_EN       0x40  // Mask for bit, bit 6
    #define MASK_ENABLE_ULP    0x20  // Mask for bit, bit 5
    #define MASK_RW_CONT       0x10  // Mask for bit, bit 4
    #define MASK_SW_RESET      0x08  // Mask for bit, bit 3
    #define MASK_TM_COUNT_RST  0x02  // Mask for bit, bit 1
    #define MASK_SPI_REG_READ  0x01  // Mask for bit 0
#define SETUP_2 0x42  // address for 0x42 register
    #define MASK_INT_MUX1           0x00000030  // Mask for bits 5-4
    #define MASK_INT_MUX2           0x00300000  // Mask for bits 21-20
    #define MASK_INT_MUX3           0x00C00000  // Mask for bits 23-22
    #define MASK_FIFO_EARLY         0x0007C000  // Mask for bits 18-14
    #define MASK_REG_FIFO_PERIOD    0x00003FC0  // Mask for bits 13-6
    #define MASK_FIFO_PARTITION     0x0000000F  // Mask for bits 3-0
#define SETUP_3 0x22  // address for 0x22 register
    #define MASK_ILED3_LSB 0xC00000  // Mask for bits 23-22
    #define MASK_ILED2_LSB 0x300000  // Mask for bits 21-20
    #define MASK_ILED1_LSB 0x0C000   // Mask for bits 19-18
    #define MASK_ILED3_MSB 0x03F000  // Mask for bits 17-12
    #define MASK_ILED2_MSB 0x00FC0   // Mask for bits 11-6
    #define MASK_ILED1_MSB 0x003F    // Mask for bits 5-0
#define SETUP_4 0x23  // address for 0x23 register
    #define MASK_CONTROL_DYN_TX     0x00100000  // Mask for bit 20: Control dynamic transmit
    #define MASK_ILED_FS            0x00020000  // Mask for bit 17: LED full-scale current programming
    #define MASK_ENSEPGAIN4         0x00008000  // Mask for bit 15: Enable separate gain settings
    #define MASK_CONTROL_DYN_BIAS   0x00004000  // Mask for bit 14: Control dynamic bias
    #define MASK_OSC_ENABLE         0x00000200  // Mask for bit 9: Oscillator enable
    #define MASK_CONTROL_DYN_TIA    0x00000010  // Mask for bit 4: Control dynamic TIA
    #define MASK_CONTROL_DYN_ADC    0x00000008  // Mask for bit 3: Control dynamic ADC
    #define MASK_PDN_RX             0x00000002  // Mask for bit 1: Power down RX
    #define MASK_PDN_AFE            0x00000001  // Mask for bit 0: Power down entire AFE
#define SETUP_5 0x24  // address for 0x24 register
    #define MASK_ILED4_MSB          0x0001F800  // Mask for bits 16-11
    #define MASK_ILED4_LSB          0x00000600  // Mask for bits 10-9

#define TIA_SETUP_1 0x20
    #define MASK_ENSEPGAIN          0x00008000  // Mask for bit 15: Enable separate gain settings
    #define MASK_TIA_GAIN_SEP_MSB   0x00007F80  // Mask for bits 14-7: MSB of the R_f control in phases 3 and 4
    #define MASK_TIA_CF_SEP         0x00000078  // Mask for bits 6-3: C_f control in phases 3 and 4
    #define MASK_TIA_GAIN_SEP_LSB   0x00000007  // Mask for bits 2-0: LSB of the R_f control in phases 3 and 4

#define TIA_SETUP_2 0x21           
    #define MASK_IFS_OFFDAC  0x00007C00  // Mask for bits 14-10: Programs the full-scale current range of the offset cancellation DAC
    #define MASK_FILTER_BW   0x00000200  // Mask for bit 9: Controls the bandwidth setting of the noise-reduction filter
    #define MASK_TIA_GAIN_MSB 0x00000040  // Mask for bit 6: MSB of the R_f control in phases 1 and 2
    #define MASK_TIA_CF      0x00000038  // Mask for bits 5-3: C_f control in phases 1 and 2
    #define MASK_TIA_GAIN_LSB 0x00000007  // Mask for bits 2-0: LSB of the R_f control in phases 1 and 2

#define PD_ENABLE 0x4E  // address for 0x4E register
    // #define MASK_AC_LEADOFF_FREQ    0x800000  // Mask for bit 23
    // #define MASK_PROG_VCOMP_HIGH    0x700000  // Mask for bits 22-20
    // #define MASK_PROG_VCOMP_LOW     0x0E0000  // Mask for bits 19-17
    // #define MASK_POL_ILEADOFFP      0x010000  // Mask for bit 16
    // #define MASK_POL_ILEADOFFM      0x008000  // Mask for bit 15
    // #define MASK_MAG_ILEADOFF       0x006000  // Mask for bits 14-13
    // #define MASK_CHOOSE_AC_LEADOFF  0x001000  // Mask for bit 12
    #define MASK_TRIPLE_PD_ENABLE   0x000010  // Mask for bit 4
    #define MASK_DUAL_PD_ENABLE     0x000008  // Mask for bit 3
    // #define MASK_ENABLE_PTT         0x000004  // Bit 2

// Define Timing register addresses -----------------------------------------------------------
#define PRPCT           0x1D  // Address for count value that sets PRF 
#define CLKDIV_TE       0x39  // Address for clock division ratio
#define DYN_TIA_STC     0x64  // Address for start of TIA active phase
#define DYN_TIA_ENDC    0x65  // Address for end of TIA active phase
#define DYN_ADC_STC     0x66  // Address for start of ADC active phase
#define DYN_ADC_ENDC    0x67  // Address for end of ADC active phase
#define DYN_CLK_STC     0x68  // Address for start of 4MHz oscillator active phase (for ADC)
#define DYN_CLK_ENDC    0x69  // Address for end of 4MHz oscillator active phase (for ADC)
#define DEEP_SLEEP_STC  0x6A  // Address for start of deep sleep phase
#define DEEP_SLEEP_ENDC 0x6B  // Address for end of deep sleep phase

// Define LED control register addresses ------------------------------------------------------
// LED1
#define LED1LEDSTC    0x03  // Address for LED1 start 
#define LED1LEDENDC   0x04  // Address for LED1 end 

// LED2
#define LED2LEDSTC    0x09  // Address for LED2 start 
#define LED2LEDENDC   0x0A  // Address for LED2 end 

// LED3
#define LED3LEDSTC    0x36  // Address for LED3 start
#define LED3LEDENDC   0x37  // Address for LED3 end 

// LED4
#define LED4LEDSTC    0x43  // Address for LED4 start
#define LED4LEDENDC   0x44  // Address for LED4 end 

// Sample1
#define LED1STC       0x07  // Address for sample LED1 start
#define LED1ENDC      0x08  // Address for sample LED1 end
#define LED3STC       0x07  // Address for sample LED3 start (same as LED1)
#define LED3ENDC      0x08  // Address for sample LED3 end (same as LED1)

// Sample2
#define LED2STC       0x01  // Address for sample LED2 start
#define LED2ENDC      0x02  // Address for sample LED2 end
#define LED4STC       0x01  // Address for sample LED4 start (same as LED2)
#define LED4ENDC      0x02  // Address for sample LED4 end (same as LED2)

// Ambient1
#define ALED1STC      0x0B  // Address for sample Ambient1 start
#define ALEDENDC      0x0C  // Address for sample Ambient1 end

// Ambient2
#define ALED2STC      0x05  // Address for sample Ambient2 start
#define ALED2ENDC     0x06  // Address for sample Ambient2 end

// Convert1
#define LED1CONVST    0x11  // Address for LED1 convert phase start
#define LED1CONVEND   0x12  // Address for LED1 convert phase end
#define LED3CONVST    0x11  // Address for LED3 convert phase start (same as LED1)
#define LED3CONVEND   0x12  // Address for LED3 convert phase end (same as LED1)

// Convert2
#define LED2CONVST    0x0D  // Address for LED2 convert phase start
#define LED2CONVEND   0x0E  // Address for LED2 convert phase end
#define LED4CONVST    0x0D  // Address for LED4 convert phase start (same as LED2)
#define LED4CONVEND   0x0E  // Address for LED4 convert phase end (same as LED2)

// AmbientConvert1
#define ALED1CONVST   0x13  // Address for ambient1 convert phase start
#define ALED1CONVEND  0x14  // Address for ambient1 convert phase end

// AmbientConvert2
#define ALED2CONVST   0x0F  // Address for ambient2 convert phase start
#define ALED2CONVEND  0x10  // Address for ambient2 convert phase end

// Define PD control address ------------------------------------------------------------------
#define TG_PD1STC     0x45  // Address for PD1 start count
#define TG_PD1ENDC    0x46  // Address for PD1 end count

#define TG_PD2STC     0x47  // Address for PD2 start count
#define TG_PD2ENDC    0x48  // Address for PD2 end count

#define TG_PD3STC     0x49  // Address for PD3 start count
#define TG_PD3ENDC    0x4A  // Address for PD3 end count

// Define LED output addresses ----------------------------------------------------------------
// Single phase outputs
#define LED1VAL       0x2A  // Address for phase 1 output (24-bit two's complement)
#define ALED1VAL      0x2B  // Addres for phase 2 output (24-bit two's complement)
#define LED2VAL       0x2C  // Address for phase 3 output (24-bit two's complement)
#define ALED2VAL      0x2D  // Address for phase 4 output (24-bit two's complement)

// Difference outputs
#define LED2_ALED2VAL 0x2E  // Address for LED2 output - Ambient2 output (24-bit two's complement)
#define LED1_ALED1VAL 0x2F  // Address for LED1 output - Ambient1 output (24-bit two's complement)

#endif // REGISTERS_H