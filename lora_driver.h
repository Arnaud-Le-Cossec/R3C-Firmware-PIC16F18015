#define JOIN_RETRY_DELAY_MS 20000 // 20seconds

void LoRa_setup(void);
uint8_t AT_command_check(const char * at_command, const char * expected_response, uint8_t response_size);
void AT_command(const char * at_command);


void LoRa_setup(void){
    /*Wake E5 mini*/
    AT_command("Wake up !!");
    /*Check E5 mini communication*/
    if(!AT_command_check("AT", "+AT: OK", 7)){
        EUSART_print("RX/TX Fail");
    }
    // Check connection
    while(!AT_command_check("AT+JOIN", "+JOIN: Joined already", 21)){
        __delay_ms(JOIN_RETRY_DELAY_MS);
    };
    EUSART_print("Connected !");
}

void LoRa_send_data(uint16_t temperature, uint16_t humidity, uint8_t battery){
    EUSART_print("AT+MSGHEX=");
    /*temperature (big endian)*/
    EUSART_print_hex(temperature&0xFF);
    EUSART_print_hex((temperature >> 8)&0xFF);
    /*humidity (big endian)*/
    EUSART_print_hex(humidity&0xFF);
    EUSART_print_hex((humidity >> 8)&0xFF);
    /*battery*/
    EUSART_print_hex(battery);
    /*End command*/
    EUSART_write(0x0A); // Line feed
    __delay_ms(20);
}

uint8_t AT_command_check(const char * at_command, const char * expected_response, uint8_t response_size){
    /*Clear buffer*/
    EUSART_clear_buffer(RX_buffer, RX_BUFFER_SIZE);
    RX_index = 0;
    /*Flush RX FIFO*/
    uint8_t tmp;
    while(PIR3bits.RC2IF){
        tmp = RC2REG;
    }
    RC2STAbits.CREN = 0;
    RC2STAbits.CREN = 1;
    /*Enable RX interrupts*/
    PIE3bits.RC2IE = 1;
    /*Send AT command*/
    EUSART_print(at_command);
    EUSART_write(0x0A); // Line feed
    /*wait for*/
    //__delay_ms(20);
    while(RX_index < response_size+2); // wait response length + CRLF
    __delay_ms(5);                     // wait extra to keep safe margin
    /*Disable RX interrupts*/
    PIE3bits.RC2IE = 0;
    for(uint8_t i=0; i<response_size; i++){
        //EUSART_write(RX_buffer[i]);
        if(RX_buffer[i] != expected_response[i]){
            return 0; // fail
        }
    }
    return 1; // success
}

void AT_command(const char * at_command){
    EUSART_print(at_command);
    EUSART_write(0x0A); // Line feed
    __delay_ms(20);
}
