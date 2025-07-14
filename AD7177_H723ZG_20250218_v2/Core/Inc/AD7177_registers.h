/*
 * AD7177_registers.h
 *
 *  Created on: June 4, 2025
 *      Author: Natalie
 */

#ifndef INC_AD7177_REGISTERS_H_
#define INC_AD7177_REGISTERS_H_

// Register Definitions
#define AD7177_REG_COMM           0x00 // Communications Register (W, 8-bit)
#define AD7177_REG_STATUS         0x00 // Status Register (R, 8-bit)
#define AD7177_REG_ADCMODE        0x01 // ADC Mode Register (RW, 16-bit)
#define AD7177_REG_IFMODE         0x02 // If Mode Register (RW, 16-bit)
#define AD7177_REG_REGCHECK       0x03 // Reg Check Register (R, 24-bit)
#define AD7177_REG_DATA           0x04 // Data Register (R, 32-bit)
#define AD7177_REG_GPIOCON        0x06 // GPIOCON Register (RW, 16-bit)
#define AD7177_REG_ID             0x07 // ID Register (R, 16-bit)

#define AD7177_REG_CH0            0x10 // CH0 Register setup (RW, 16-bit)
#define AD7177_REG_CH1            0x11 // CH1 Register setup (RW, 16-bit)
#define AD7177_REG_CH2            0x12 // CH2 Register (RW, 16-bit)
#define AD7177_REG_CH3            0x13 // CH3 Register (RW, 16-bit)

#define AD7177_REG_SETUPCON0      0x20 // 0 Setup Configuration Register (RW, 16-bit)
#define AD7177_REG_SETUPCON1      0x21 // 1 Setup Configuration Register (RW, 16-bit)
#define AD7177_REG_SETUPCON2      0x22 // 2 Setup Configuration Register (RW, 16-bit)
#define AD7177_REG_SETUPCON3      0x23 // 3 Setup Configuration Register (RW, 16-bit)

#define AD7177_REG_FILTCON0       0x28 // 0 Filter Configuration Register (RW, 16-bit)
#define AD7177_REG_FILTCON1       0x29 // 1 Filter Configuration Register (RW, 16-bit)
#define AD7177_REG_FILTCON2       0x2A // 2 Filter Configuration Register (RW, 16-bit)
#define AD7177_REG_FILTCON3       0x2B // 3 Filter Configuration Register (RW, 16-bit)

#define AD7177_REG_OFFSET0        0x30 // 0 Offset Register (RW, 24-bit)
#define AD7177_REG_OFFSET1        0x31 // 1 Offset Register (RW, 24-bit)
#define AD7177_REG_OFFSET2        0x32 // 2 Offset Register (RW, 24-bit)
#define AD7177_REG_OFFSET3        0x33 // 3 Offset Register (RW, 24-bit)

#define AD7177_REG_GAIN0          0x38 // 0 Gain Register (RW, 24-bit)
#define AD7177_REG_GAIN1          0x39 // 1 Gain Register (RW, 24-bit)
#define AD7177_REG_GAIN2          0x3A // 2 Gain Register (RW, 24-bit)
#define AD7177_REG_GAIN3          0x3B // 3 Gain Register (RW, 24-bit)


//still need to rewrite this part

// Operation Modes
#define AD7177_COMM_WRITE         0b00000000 // Write Operation (at channel 0 register)
#define AD7177_COMM_READ          0b01000000 // Read Operation (at channel 0 register)

//setup configuration for all channels
#define AD7177_SETUPCON0          0x1F20 //bipolar, 2.5 V internal reference enabled, AIN+ and AIN- buffers enabled
#define AD7177_READ_DATA_REG	  0x44

//channel setups
#define AD7177_CH0_SETUP0		  0x8004
#define AD7177_CH1_SETUP0		  0x8024
#define AD7177_CH2_SETUP0		  0x8044
#define AD7177_CH3_SETUP0		  0x8064

//GPIO Configuration
#define AD7177_GPIO				  0x0020

//Filter Configuration
#define AD7177_FILTCON0			  0x0009

//ADC Mode
#define AD7177_ADCMODE		  	  0x8000 //had this on 0x8000 to enable 2.5 V reference

//IF Mode
#define AD7177_IFMODE			  0x0040
#define AD7177_CONT_READ		  0x00C0

//to avoid hardcoding stuff
#define NUM_CH_ENABLED 			  3


#endif /* INC_AD7177_REGISTERS_H_ */
