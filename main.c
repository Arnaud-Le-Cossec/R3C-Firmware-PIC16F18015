/*
 * File:   main.c
 * Author: Nono
 *
 * Created on 20 November 2023, 07:50
 */


// PIC16F18015 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1
#pragma config FEXTOSC = OFF    // External Oscillator Selection bits (Oscillator not enabled)
#pragma config RSTOSC = HFINTOSC_1MHz// Reset Oscillator Selection bits (HFINTOSC (1MHz))
#pragma config CLKOUTEN = OFF   // Clock Out Enable bit (CLKOUT function is disabled; i/o or oscillator function on OSC2)
#pragma config VDDAR = HI       // VDD Range Analog Calibration Selection bit (Internal analog systems are calibrated for operation between VDD = 2.3 - 5.5V)

// CONFIG2
#pragma config MCLRE = EXTMCLR  // Master Clear Enable bit (If LVP = 0, MCLR pin is MCLR; If LVP = 1, RA3 pin function is MCLR)
#pragma config PWRTS = PWRT_OFF // Power-up Timer Selection bits (PWRT is disabled)
#pragma config WDTE = ON        // WDT Operating Mode bits (WDT enabled regardless of Sleep; SEN bit is ignored)
#pragma config BOREN = ON       // Brown-out Reset Enable bits (Brown-out Reset enabled, SBOREN bit is ignored)
#pragma config DACAUTOEN = OFF  // DAC Buffer Automatic Range Select Enable bit (DAC Buffer reference range is determined by the REFRNG bit)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection bit (Brown-out Reset Voltage (VBOR) set to 1.9V)
#pragma config ZCD = OFF        // ZCD Disable bit (ZCD module is disabled; ZCD can be enabled by setting the ZCDSEN bit of ZCDCON)
#pragma config PPS1WAY = ON     // PPSLOCKED One-Way Set Enable bit (The PPSLOCKED bit can be cleared and set only once after an unlocking sequence is executed; once PPSLOCKED is set, all future changes to PPS registers are prevented)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable bit (Stack Overflow or Underflow will cause a reset)

// CONFIG3

// CONFIG4
#pragma config BBSIZE = BB512   // Boot Block Size Selection bits (512 words boot block size)
#pragma config BBEN = OFF       // Boot Block Enable bit (Boot Block disabled)
#pragma config SAFEN = OFF      // Storage Area Flash (SAF) Enable bit (SAF disabled)
#pragma config WRTAPP = OFF     // Application Block Write Protection bit (Application Block is NOT write protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block is NOT write protected)
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration Register is NOT write protected)
#pragma config WRTD = OFF       // Data EEPROM Write-Protection bit (Data EEPROM is NOT write-protected)
#pragma config WRTSAF = OFF     // Storage Area Flash (SAF) Write Protection bit (SAF is NOT write protected)
#pragma config LVP = ON         // Low Voltage Programming Enable bit (Low Voltage programming enabled. MCLR/Vpp pin function is MCLR. MCLRE Configuration bit is ignored)

// CONFIG5
#pragma config CP = OFF         // Program Flash Memory Code Protection bit (Program Flash Memory code protection is disabled)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (EEPROM code protection is disabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <pic16f18015.h>

#define _XTAL_FREQ 1000000  // 32Mhz configuration de la carte

#include "watchdog_driver.h"
#include "eusart_driver.h"
#include "i2c_driver.h"
#include "analog_driver.h"
#include "lora_driver.h"

#define LED_PIN 4

#define SLEEP_COUNTER_THRESHOLD_DEFAULT 14

// Prototype


void main(void) {
    
    TRISA &= !(1<<LED_PIN);		// Set outputs
    
    ANSELA = 0x0;
    
    /*Setup I2C*/
    I2C_setup();
    
    /*Setup serial communication*/
    EUSART_setup();
    
    /*Setup ADC*/
    Analog_setup();
    
    /*Setup watchdog*/
    WDT_setup();
    
    /*Declare sleep counter*/
    uint8_t sleep_counter = 0;
    
    //PORTA |= (1<<LED_PIN);		// Set LED to 1
	
    float temp;
    float humidity;
    
    while(1){
           
        /*Start sleep, will be awaken by watchdog*/
        //SLEEP_start();
        
        /*increment sleep counter*/
        //sleep_counter++;
        
        //if(sleep_counter >= SLEEP_COUNTER_THRESHOLD_DEFAULT){
            /*Sleep*/
        //    sleep_counter = 0;
        EUSART_print("Hello ! I am a PIC ! ");
            
        //}
            
        //uint16_t a = Analog_read();
        //uint8_t r = a*(100.0/1023.0);  
        //EUSART_print_num(r);
        
        //EUSART_print_num(sleep_counter);
        //blink led for debugging

        //I2C_SHT4x_read(&temp, &humidity);
        //I2C_PCF8574_write();
        I2C_MCP23008_read();
        LoRa_setup();
        __delay_ms(1000);
    }
    return;
}










