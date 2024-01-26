#define BATT_PIN 0
#define BATT_MAX_VOLTAGE_MV 330.0

void Analog_setup(void);
uint16_t Analog_read(void);

void Analog_setup(void){
    /*Settings valid for PIC16F18015*/
    TRISA |= (1<<BATT_PIN);         // Set as input
    ANSELA |= (1<<BATT_PIN);		// Set as analog
    ADPCH = BATT_PIN & 0b00111111;  // Select channel
    ADREF = 0x0;                    // Set positive voltage reference to VDD (default)
    ADCON0bits.CS = 1;              // Use ADC internal clock
    ADCON0bits.FM = 1;              // result right justified
}

uint16_t Analog_read_raw(void){
    ADCON0bits.ON = 1;              // Activate ACD Module
    ADCON0bits.GO = 1;              // Start conversion
    while(ADCON0bits.GO);           // Wait for conversion end
    
    return ADRES;                   // Return result
}

uint16_t Analog_read_voltage(void){
    uint16_t a = Analog_read_raw();
    uint16_t r = a*(BATT_MAX_VOLTAGE_MV/1023.0);  
    return r;                       // Return result (mV)
}

uint8_t Analog_read_percent(void){
    uint16_t a = Analog_read_raw();
    uint8_t r = a*(100.0/1023.0);  
    return r;                       // Return result (%)
}