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

#define _XTAL_FREQ 1000000  // 32Mhz configuration de la carte

#define LED_PIN 4

#define SLEEP_COUNTER_THRESHOLD_DEFAULT 14

// Prototype
void WDT_setup(void);
void SLEEP_start(void);

void EUSART_setup(void);
void EUSART_write(uint8_t txData);
void EUSART_print(const char* string);
void EUSART_print_num(uint8_t number);

void main(void) {
    
    TRISA &= !(1<<LED_PIN);		// Set outputs
    
    /*Setup serial communication*/
    EUSART_setup();
    
    /*Setup watchdog*/
    WDT_setup();
    
    /*Declare sleep counter*/
    uint8_t sleep_counter = 0;
    
    PORTA |= (1<<LED_PIN);		// Set LED to 1
	
    while(1){
           
        /*Start sleep, will be awaken by watchdog*/
        SLEEP_start();
        
        /*increment sleep counter*/
        sleep_counter++;
        
        if(sleep_counter >= SLEEP_COUNTER_THRESHOLD_DEFAULT){
            /*Sleep*/
            sleep_counter = 0;
        }
        
        PORTA &= !(1<<LED_PIN);		// Set LED to 0
        EUSART_print("Hello ! I am a PIC ! ");
        EUSART_print_num(sleep_counter);
        __delay_ms(1000);
        PORTA |= (1<<LED_PIN);		// Set LED to 1
        //EUSART_write('B');
        __delay_ms(1000);
    }
    return;
}

void WDT_setup(void){
    /*Setup watchdog for slowest interval : 256 sec*/
    WDTCONbits.PS = 0b10000;//1s
}

void SLEEP_start(void){
    asm("CLRWDT");
    asm("SLEEP");
}

void EUSART_setup(void){
    /* Transmit Enable */
    TX2STAbits.TXEN = 1;
    /* High Baud Rate Select */
    TX2STAbits.BRGH = 1;
    /* 16-bit Baud Rate Generator is used */
    BAUD2CONbits.BRG16 = 1;
    /* Serial Port Enable */
    RC2STAbits.SPEN = 1;
    /* Baud rate 9600 */
    SP2BRGL = 25;
    SP2BRGH = 0;
    /* RA5 is TX1 */
    RA5PPS = 0x11;
    /* Configure RA5 as output. */
    TRISAbits.TRISA5 = 0;
}

void EUSART_write(uint8_t txData){
    while(PIR3bits.TX2IF == 0){}
    TX2REG = txData;
}

void EUSART_print(const char* string){
    uint8_t c=0;
    while(string[c]!=0){
        EUSART_write(string[c]);
        c++;
    }
}

void EUSART_print_num(uint8_t number){
    uint8_t c = (number/100);
    uint8_t d = ((number-c)/10);
    uint8_t u = (number-c-d);
    EUSART_write(c+48);
    EUSART_write(d+48);
    EUSART_write(u+48);
}