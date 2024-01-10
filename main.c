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

#define LED_PIN 4

#define SLEEP_COUNTER_THRESHOLD_DEFAULT 14

// Prototype
void WDT_setup(void);
void SLEEP_start(void);

void I2C_setup(void);
void I2C_wait(void);
void I2C_start(void);
void I2C_RepeatedStart(void);
void I2C_stop(void);
void I2C_write(uint8_t data);
uint8_t I2C_read();
uint8_t I2C_write_query(uint8_t address, uint8_t data);
uint8_t I2C_read_query(uint8_t address, uint8_t *data, uint8_t number_of_bytes);
uint8_t I2C_SHT4x_read(float *t_degC, float *rh_pRH);
void I2C_MCP23008_write(void);
void I2C_MCP23008_read(void);

void EUSART_setup(void);
void EUSART_write(uint8_t txData);
void EUSART_print(const char* string);
void EUSART_print_num(uint8_t number);

void main(void) {
    
    TRISA &= !(1<<LED_PIN);		// Set outputs
    
    ANSELA = 0x0;
    
    /*Setup I2C*/
    I2C_setup();
    
    /*Setup serial communication*/
    EUSART_setup();
    
    /*Setup watchdog*/
    WDT_setup();
    
    /*Declare sleep counter*/
    uint8_t sleep_counter = 0;
    
    PORTA |= (1<<LED_PIN);		// Set LED to 1
	
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
        
        //EUSART_print_num(sleep_counter);
        //blink led for debugging
        //I2C_write_query(0x44, 0xFD);
        //I2C_SHT4x_read(&temp, &humidity);
        //I2C_PCF8574_write();
        I2C_MCP23008_read();
        __delay_ms(500);
        //PORTA ^= (1<<LED_PIN);		// Set LED to 1
    }
    return;
}

void WDT_setup(void){
    /*Setup watchdog for slowest interval : 256 sec*/
    //0b10010
    WDTCONbits.PS = 0b10000;//1s
}

void SLEEP_start(void){
    /*Assembly instruction to reset watchdog*/
    asm("CLRWDT");
    /*Assembly instruction to put MCU into sleep*/
    asm("SLEEP");
}

void I2C_setup(void){

    /*Set desired I2C clock frequency (in kHz)*/
    uint8_t i2c_freq = 100;
    /*Set RA1 (SCL) and RA2 (SDA) as inputs*/
    TRISAbits.TRISA1 = 1;
    TRISAbits.TRISA2 = 1;
    ODCONAbits.ODCA1 = 1;
    ODCONAbits.ODCA2 = 1;
    RA1PPS = 0x15; // SCL1 output PPS address (on PIC16F18015)
    RA2PPS = 0x16; // SDA1 output PPS address (on PIC16F18015)
    SSP1CLKPPS = 0x1; // Set SCL1 input PPS to RA1 (on PIC16F18015)
    SSP1DATPPS = 0x2; // Set SDA1 input PPS to RA2 (on PIC16F18015)
    /**/
    SSP1STATbits.SMP = 1; // Default settings + Standard speed
    /*Host Synchronous Serial Port Mode Select bits*/
    SSP1CON1bits.SSPM = 0b1000; // Master mode
    /**/
    SSP1CON2 = 0; // Default settings
    /*Speed*/
    SSP1ADD = 3;//24;//(_XTAL_FREQ/(4*i2c_freq*100))-1; //Setting Clock Speed;   //100kbit/s
    /*Host Synchronous Serial Port Enable bit*/
    SSP1CON1bits.SSPEN = 1; // Enable

}

void I2C_wait(void){
   while ((SSP1STAT & 0b00000100) || (SSP1CON2 & 0x00011111)); //Check if busy
}

void I2C_start(void){
  I2C_wait();    
  SSP1CON2bits.SEN = 1;             //Initiate start condition
}

void I2C_RepeatedStart(void){
  I2C_wait();
  SSP1CON2bits.RSEN = 1;           //Initiate repeated start condition
}

void I2C_stop(void){
  I2C_wait();
  SSP1CON2bits.PEN = 1;           //Initiate stop condition
}

void I2C_write(uint8_t data){
  I2C_wait();
  SSP1BUF = data;         //Write data to SSP1BUF
}

uint8_t I2C_read(){
  //PORTA ^= (1<<LED_PIN);		// Set LED to 1
  uint8_t tmp;
  I2C_wait();
  SSP1CON2bits.RCEN = 1;
  PORTA &= !(1<<LED_PIN);		// Set LED to 0
  I2C_wait();
  //
  //while(SSP1STAT & 0b00000001);
  PORTA |= (1<<LED_PIN);		// Set LED to 1
  tmp = SSP1BUF;      //Read data from SSP1BUF
  I2C_wait();
  SSP1CON2bits.ACKDT = 0;    //Acknowledge bit
  SSP1CON2bits.ACKEN = 1;          //Acknowledge sequence
  return tmp;
}

uint8_t I2C_write_query(uint8_t address, uint8_t data){
    I2C_start();         //Start condition
    address = (address << 1)&0b11111110; // shift left
    I2C_write(address & 0b11111110);     //7 bit address + Write
    I2C_write(data);     //Write data
    I2C_stop();          //Stop condition
    return 0;
}

uint8_t I2C_read_query(uint8_t address, uint8_t *data, uint8_t number_of_bytes){
    I2C_start();         //Start condition
    address = (address << 1)&0b11111110; // shift left
    I2C_write(address | 0b00000001);     //7 bit address + Write
    for(int i=0; i<number_of_bytes; i++){
        data[i] = I2C_read(); // read 
    }
    I2C_stop();          //Stop condition
    return 0;
}

uint8_t I2C_SHT4x_read(float *t_degC, float *rh_pRH){
    I2C_write_query(0x44,0xFD);//0x44
    __delay_ms(10);
    uint8_t rx_data[6];
    I2C_read_query(0x44, rx_data, 6);
    uint16_t raw_temp = (rx_data[0]<<8) + rx_data[1];
    //uint8_t checksum_t = rx_data[2];
    uint16_t raw_rh = (rx_data[3]<<8) + rx_data[4];
    //uint8_t checksum_rh = rx_data[5];
    *t_degC = -45.0 + (175.0 * (raw_temp/65535.0));
    *rh_pRH = -6.0 + (125.0 * (raw_rh/65535.0));
    if (*rh_pRH > 100){
        *rh_pRH = 100;
    }
    if (*rh_pRH < 0){
        *rh_pRH = 0;
    }
    return 0;
}

void I2C_MCP23008_write(void){
    I2C_write_query(0x27,0x01);
}

void I2C_MCP23008_read(void){
    uint8_t rx_data[11];
    //I2C_write_query(0x27,0x01);
    //__delay_ms(1);
    I2C_read_query(0x27, rx_data, 11);
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
    uint8_t d = (number/10)%10;
    uint8_t u = (number)%10;
    EUSART_write(c+48);
    EUSART_write(d+48);
    EUSART_write(u+48);
}