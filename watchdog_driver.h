void WDT_setup(void);
void SLEEP_start(void);

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