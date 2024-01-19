# 1 "main.c"
# 1 "<built-in>" 1
# 1 "<built-in>" 3
# 288 "<built-in>" 3
# 1 "<command line>" 1
# 1 "<built-in>" 2
# 1 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\language_support.h" 1 3
# 2 "<built-in>" 2
# 1 "main.c" 2
# 14 "main.c"
#pragma config FEXTOSC = OFF
#pragma config RSTOSC = HFINTOSC_1MHz
#pragma config CLKOUTEN = OFF
#pragma config VDDAR = HI


#pragma config MCLRE = EXTMCLR
#pragma config PWRTS = PWRT_OFF
#pragma config WDTE = ON
#pragma config BOREN = ON
#pragma config DACAUTOEN = OFF
#pragma config BORV = LO
#pragma config ZCD = OFF
#pragma config PPS1WAY = ON
#pragma config STVREN = ON




#pragma config BBSIZE = BB512
#pragma config BBEN = OFF
#pragma config SAFEN = OFF
#pragma config WRTAPP = OFF
#pragma config WRTB = OFF
#pragma config WRTC = OFF
#pragma config WRTD = OFF
#pragma config WRTSAF = OFF
#pragma config LVP = ON


#pragma config CP = OFF
#pragma config CPD = OFF





# 1 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\xc.h" 1 3
# 18 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\xc.h" 3
extern const char __xc8_OPTIM_SPEED;

extern double __fpnormalize(double);



# 1 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\xc8debug.h" 1 3



# 1 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\stdlib.h" 1 3



# 1 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\musl_xc8.h" 1 3
# 4 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\stdlib.h" 2 3






# 1 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\features.h" 1 3
# 10 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\stdlib.h" 2 3
# 21 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\stdlib.h" 3
# 1 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\bits/alltypes.h" 1 3
# 18 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\bits/alltypes.h" 3
typedef long int wchar_t;
# 122 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\bits/alltypes.h" 3
typedef unsigned size_t;
# 168 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\bits/alltypes.h" 3
typedef __int24 int24_t;
# 204 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\bits/alltypes.h" 3
typedef __uint24 uint24_t;
# 21 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\stdlib.h" 2 3


int atoi (const char *);
long atol (const char *);
long long atoll (const char *);
double atof (const char *);

float strtof (const char *restrict, char **restrict);
double strtod (const char *restrict, char **restrict);
long double strtold (const char *restrict, char **restrict);



long strtol (const char *restrict, char **restrict, int);
unsigned long strtoul (const char *restrict, char **restrict, int);
long long strtoll (const char *restrict, char **restrict, int);
unsigned long long strtoull (const char *restrict, char **restrict, int);

int rand (void);
void srand (unsigned);

void *malloc (size_t);
void *calloc (size_t, size_t);
void *realloc (void *, size_t);
void free (void *);

          void abort (void);
int atexit (void (*) (void));
          void exit (int);
          void _Exit (int);

void *bsearch (const void *, const void *, size_t, size_t, int (*)(const void *, const void *));







__attribute__((nonreentrant)) void qsort (void *, size_t, size_t, int (*)(const void *, const void *));

int abs (int);
long labs (long);
long long llabs (long long);

typedef struct { int quot, rem; } div_t;
typedef struct { long quot, rem; } ldiv_t;
typedef struct { long long quot, rem; } lldiv_t;

div_t div (int, int);
ldiv_t ldiv (long, long);
lldiv_t lldiv (long long, long long);

typedef struct { unsigned int quot, rem; } udiv_t;
typedef struct { unsigned long quot, rem; } uldiv_t;
udiv_t udiv (unsigned int, unsigned int);
uldiv_t uldiv (unsigned long, unsigned long);
# 4 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\xc8debug.h" 2 3








#pragma intrinsic(__builtin_software_breakpoint)
extern void __builtin_software_breakpoint(void);
# 23 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\xc.h" 2 3

# 1 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\builtins.h" 1 3



# 1 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\stdint.h" 1 3
# 22 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\stdint.h" 3
# 1 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\bits/alltypes.h" 1 3
# 127 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\bits/alltypes.h" 3
typedef unsigned long uintptr_t;
# 142 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\bits/alltypes.h" 3
typedef long intptr_t;
# 158 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\bits/alltypes.h" 3
typedef signed char int8_t;




typedef short int16_t;
# 173 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\bits/alltypes.h" 3
typedef long int32_t;





typedef long long int64_t;
# 188 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\bits/alltypes.h" 3
typedef long long intmax_t;





typedef unsigned char uint8_t;




typedef unsigned short uint16_t;
# 209 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\bits/alltypes.h" 3
typedef unsigned long uint32_t;





typedef unsigned long long uint64_t;
# 229 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\bits/alltypes.h" 3
typedef unsigned long long uintmax_t;
# 22 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\stdint.h" 2 3


typedef int8_t int_fast8_t;

typedef int64_t int_fast64_t;


typedef int8_t int_least8_t;
typedef int16_t int_least16_t;

typedef int24_t int_least24_t;
typedef int24_t int_fast24_t;

typedef int32_t int_least32_t;

typedef int64_t int_least64_t;


typedef uint8_t uint_fast8_t;

typedef uint64_t uint_fast64_t;


typedef uint8_t uint_least8_t;
typedef uint16_t uint_least16_t;

typedef uint24_t uint_least24_t;
typedef uint24_t uint_fast24_t;

typedef uint32_t uint_least32_t;

typedef uint64_t uint_least64_t;
# 144 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\stdint.h" 3
# 1 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\bits/stdint.h" 1 3
typedef int16_t int_fast16_t;
typedef int32_t int_fast32_t;
typedef uint16_t uint_fast16_t;
typedef uint32_t uint_fast32_t;
# 144 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\stdint.h" 2 3
# 4 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\builtins.h" 2 3



#pragma intrinsic(__nop)
extern void __nop(void);


#pragma intrinsic(_delay)
extern __attribute__((nonreentrant)) void _delay(uint32_t);
#pragma intrinsic(_delaywdt)
extern __attribute__((nonreentrant)) void _delaywdt(uint32_t);

#pragma intrinsic(_delay3)
extern __attribute__((nonreentrant)) void _delay3(uint8_t);
# 24 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\xc.h" 2 3




# 1 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\pic.h" 1 3




# 1 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\htc.h" 1 3



# 1 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\xc.h" 1 3
# 4 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\htc.h" 2 3
# 6 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\pic.h" 2 3







# 1 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\pic_chip_select.h" 1 3
# 259 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\pic_chip_select.h" 3
# 1 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 1 3
# 44 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
# 1 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\__at.h" 1 3
# 45 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 2 3
# 108 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char INDF0 __attribute__((address(0x000)));

__asm("INDF0 equ 00h");


typedef union {
    struct {
        unsigned INDF0 :8;
    };
} INDF0bits_t;
extern volatile INDF0bits_t INDF0bits __attribute__((address(0x000)));
# 128 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char INDF1 __attribute__((address(0x001)));

__asm("INDF1 equ 01h");


typedef union {
    struct {
        unsigned INDF1 :8;
    };
} INDF1bits_t;
extern volatile INDF1bits_t INDF1bits __attribute__((address(0x001)));
# 148 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char PCL __attribute__((address(0x002)));

__asm("PCL equ 02h");


typedef union {
    struct {
        unsigned PCL :8;
    };
} PCLbits_t;
extern volatile PCLbits_t PCLbits __attribute__((address(0x002)));
# 168 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char STATUS __attribute__((address(0x003)));

__asm("STATUS equ 03h");


typedef union {
    struct {
        unsigned C :1;
        unsigned DC :1;
        unsigned Z :1;
        unsigned nPD :1;
        unsigned nTO :1;
    };
    struct {
        unsigned CARRY :1;
        unsigned :1;
        unsigned ZERO :1;
    };
} STATUSbits_t;
extern volatile STATUSbits_t STATUSbits __attribute__((address(0x003)));
# 227 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned short FSR0 __attribute__((address(0x004)));



extern volatile unsigned char FSR0L __attribute__((address(0x004)));

__asm("FSR0L equ 04h");


typedef union {
    struct {
        unsigned FSR0L :8;
    };
} FSR0Lbits_t;
extern volatile FSR0Lbits_t FSR0Lbits __attribute__((address(0x004)));
# 251 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char FSR0H __attribute__((address(0x005)));

__asm("FSR0H equ 05h");


typedef union {
    struct {
        unsigned FSR0H :8;
    };
} FSR0Hbits_t;
extern volatile FSR0Hbits_t FSR0Hbits __attribute__((address(0x005)));
# 271 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned short FSR1 __attribute__((address(0x006)));



extern volatile unsigned char FSR1L __attribute__((address(0x006)));

__asm("FSR1L equ 06h");


typedef union {
    struct {
        unsigned FSR1L :8;
    };
} FSR1Lbits_t;
extern volatile FSR1Lbits_t FSR1Lbits __attribute__((address(0x006)));
# 295 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char FSR1H __attribute__((address(0x007)));

__asm("FSR1H equ 07h");


typedef union {
    struct {
        unsigned FSR1H :8;
    };
} FSR1Hbits_t;
extern volatile FSR1Hbits_t FSR1Hbits __attribute__((address(0x007)));
# 315 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char BSR __attribute__((address(0x008)));

__asm("BSR equ 08h");


typedef union {
    struct {
        unsigned BSR :6;
    };
    struct {
        unsigned BSR0 :1;
        unsigned BSR1 :1;
        unsigned BSR2 :1;
        unsigned BSR3 :1;
        unsigned BSR4 :1;
        unsigned BSR5 :1;
    };
} BSRbits_t;
extern volatile BSRbits_t BSRbits __attribute__((address(0x008)));
# 373 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char WREG __attribute__((address(0x009)));

__asm("WREG equ 09h");


typedef union {
    struct {
        unsigned WREG0 :8;
    };
} WREGbits_t;
extern volatile WREGbits_t WREGbits __attribute__((address(0x009)));
# 393 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char PCLATH __attribute__((address(0x00A)));

__asm("PCLATH equ 0Ah");


typedef union {
    struct {
        unsigned PCLATH :7;
    };
} PCLATHbits_t;
extern volatile PCLATHbits_t PCLATHbits __attribute__((address(0x00A)));
# 413 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char INTCON __attribute__((address(0x00B)));

__asm("INTCON equ 0Bh");


typedef union {
    struct {
        unsigned INTEDG :1;
        unsigned :5;
        unsigned PEIE :1;
        unsigned GIE :1;
    };
} INTCONbits_t;
extern volatile INTCONbits_t INTCONbits __attribute__((address(0x00B)));
# 446 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char PORTA __attribute__((address(0x00C)));

__asm("PORTA equ 0Ch");


typedef union {
    struct {
        unsigned RA0 :1;
        unsigned RA1 :1;
        unsigned RA2 :1;
        unsigned RA3 :1;
        unsigned RA4 :1;
        unsigned RA5 :1;
    };
} PORTAbits_t;
extern volatile PORTAbits_t PORTAbits __attribute__((address(0x00C)));
# 496 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char TRISA __attribute__((address(0x012)));

__asm("TRISA equ 012h");


typedef union {
    struct {
        unsigned TRISA0 :1;
        unsigned TRISA1 :1;
        unsigned TRISA2 :1;
        unsigned :1;
        unsigned TRISA4 :1;
        unsigned TRISA5 :1;
    };
} TRISAbits_t;
extern volatile TRISAbits_t TRISAbits __attribute__((address(0x012)));
# 541 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char LATA __attribute__((address(0x018)));

__asm("LATA equ 018h");


typedef union {
    struct {
        unsigned LATA0 :1;
        unsigned LATA1 :1;
        unsigned LATA2 :1;
        unsigned :1;
        unsigned LATA4 :1;
        unsigned LATA5 :1;
    };
} LATAbits_t;
extern volatile LATAbits_t LATAbits __attribute__((address(0x018)));
# 586 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char PIR0 __attribute__((address(0x08C)));

__asm("PIR0 equ 08Ch");


typedef union {
    struct {
        unsigned INTF :1;
        unsigned :3;
        unsigned IOCIF :1;
        unsigned TMR0IF :1;
    };
} PIR0bits_t;
extern volatile PIR0bits_t PIR0bits __attribute__((address(0x08C)));
# 619 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char PIR1 __attribute__((address(0x08D)));

__asm("PIR1 equ 08Dh");


typedef union {
    struct {
        unsigned NVMIF :1;
        unsigned :1;
        unsigned ACTIF :1;
        unsigned TMR1IF :1;
        unsigned TMR1GIF :1;
        unsigned TMR3IF :1;
        unsigned TMR3GIF :1;
        unsigned TMR2IF :1;
    };
} PIR1bits_t;
extern volatile PIR1bits_t PIR1bits __attribute__((address(0x08D)));
# 676 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char PIR2 __attribute__((address(0x08E)));

__asm("PIR2 equ 08Eh");


typedef union {
    struct {
        unsigned TMR4IF :1;
        unsigned TMR6IF :1;
        unsigned CCP1IF :1;
        unsigned CCP2IF :1;
        unsigned NCO1IF :1;
        unsigned CWG1IF :1;
        unsigned CLC1IF :1;
        unsigned CLC2IF :1;
    };
} PIR2bits_t;
extern volatile PIR2bits_t PIR2bits __attribute__((address(0x08E)));
# 738 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char PIR3 __attribute__((address(0x08F)));

__asm("PIR3 equ 08Fh");


typedef union {
    struct {
        unsigned CLC3IF :1;
        unsigned CLC4IF :1;
        unsigned TX1IF :1;
        unsigned RC1IF :1;
        unsigned TX2IF :1;
        unsigned RC2IF :1;
        unsigned SSP1IF :1;
        unsigned BCL1IF :1;
    };
} PIR3bits_t;
extern volatile PIR3bits_t PIR3bits __attribute__((address(0x08F)));
# 800 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char PIR4 __attribute__((address(0x090)));

__asm("PIR4 equ 090h");


typedef union {
    struct {
        unsigned SSP2IF :1;
        unsigned BCL2IF :1;
        unsigned CM1IF :1;
        unsigned ADIF :1;
        unsigned ADTIF :1;
        unsigned ZCDIF :1;
    };
} PIR4bits_t;
extern volatile PIR4bits_t PIR4bits __attribute__((address(0x090)));
# 850 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char PIE0 __attribute__((address(0x096)));

__asm("PIE0 equ 096h");


typedef union {
    struct {
        unsigned INTE :1;
        unsigned :3;
        unsigned IOCIE :1;
        unsigned TMR0IE :1;
    };
} PIE0bits_t;
extern volatile PIE0bits_t PIE0bits __attribute__((address(0x096)));
# 883 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char PIE1 __attribute__((address(0x097)));

__asm("PIE1 equ 097h");


typedef union {
    struct {
        unsigned NVMIE :1;
        unsigned :1;
        unsigned ACTIE :1;
        unsigned TMR1IE :1;
        unsigned TMR1GIE :1;
        unsigned TMR3IE :1;
        unsigned TMR3GIE :1;
        unsigned TMR2IE :1;
    };
} PIE1bits_t;
extern volatile PIE1bits_t PIE1bits __attribute__((address(0x097)));
# 940 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char PIE2 __attribute__((address(0x098)));

__asm("PIE2 equ 098h");


typedef union {
    struct {
        unsigned TMR4IE :1;
        unsigned TMR6IE :1;
        unsigned CCP1IE :1;
        unsigned CCP2IE :1;
        unsigned NCO1IE :1;
        unsigned CWG1IE :1;
        unsigned CLC1IE :1;
        unsigned CLC2IE :1;
    };
} PIE2bits_t;
extern volatile PIE2bits_t PIE2bits __attribute__((address(0x098)));
# 1002 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char PIE3 __attribute__((address(0x099)));

__asm("PIE3 equ 099h");


typedef union {
    struct {
        unsigned CLC3IE :1;
        unsigned CLC4IE :1;
        unsigned TX1IE :1;
        unsigned RC1IE :1;
        unsigned TX2IE :1;
        unsigned RC2IE :1;
        unsigned SSP1IE :1;
        unsigned BCL1IE :1;
    };
} PIE3bits_t;
extern volatile PIE3bits_t PIE3bits __attribute__((address(0x099)));
# 1064 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char PIE4 __attribute__((address(0x09A)));

__asm("PIE4 equ 09Ah");


typedef union {
    struct {
        unsigned SSP2IE :1;
        unsigned BCL2IE :1;
        unsigned CM1IE :1;
        unsigned ADIE :1;
        unsigned ADTIE :1;
        unsigned ZCDIE :1;
    };
} PIE4bits_t;
extern volatile PIE4bits_t PIE4bits __attribute__((address(0x09A)));
# 1114 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char WDTCON __attribute__((address(0x18C)));

__asm("WDTCON equ 018Ch");


typedef union {
    struct {
        unsigned SEN :1;
        unsigned PS :5;
        unsigned :1;
        unsigned CS :1;
    };
    struct {
        unsigned SWDTEN :1;
        unsigned WDTPS0 :1;
        unsigned WDTPS1 :1;
        unsigned WDTPS2 :1;
        unsigned WDTPS3 :1;
        unsigned WDTPS4 :1;
        unsigned :1;
        unsigned WDTCS :1;
    };
    struct {
        unsigned :1;
        unsigned PS0 :1;
        unsigned PS1 :1;
        unsigned PS2 :1;
        unsigned PS3 :1;
        unsigned PS4 :1;
    };
} WDTCONbits_t;
extern volatile WDTCONbits_t WDTCONbits __attribute__((address(0x18C)));
# 1225 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char BORCON __attribute__((address(0x18D)));

__asm("BORCON equ 018Dh");


typedef union {
    struct {
        unsigned BORRDY :1;
        unsigned :6;
        unsigned SBOREN :1;
    };
} BORCONbits_t;
extern volatile BORCONbits_t BORCONbits __attribute__((address(0x18D)));
# 1252 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char PCON0 __attribute__((address(0x18E)));

__asm("PCON0 equ 018Eh");


typedef union {
    struct {
        unsigned nBOR :1;
        unsigned nPOR :1;
        unsigned nRI :1;
        unsigned nRMCLR :1;
        unsigned nRWDT :1;
        unsigned :1;
        unsigned STKUNF :1;
        unsigned STKOVF :1;
    };
} PCON0bits_t;
extern volatile PCON0bits_t PCON0bits __attribute__((address(0x18E)));
# 1309 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char PCON1 __attribute__((address(0x18F)));

__asm("PCON1 equ 018Fh");


typedef union {
    struct {
        unsigned :1;
        unsigned nMEMV :1;
    };
} PCON1bits_t;
extern volatile PCON1bits_t PCON1bits __attribute__((address(0x18F)));
# 1330 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char TMR0L __attribute__((address(0x19C)));

__asm("TMR0L equ 019Ch");


extern volatile unsigned char TMR0 __attribute__((address(0x19C)));

__asm("TMR0 equ 019Ch");


typedef union {
    struct {
        unsigned TMR0L :8;
    };
    struct {
        unsigned TMR0L0 :1;
        unsigned TMR0L1 :1;
        unsigned TMR0L2 :1;
        unsigned TMR0L3 :1;
        unsigned TMR0L4 :1;
        unsigned TMR0L5 :1;
        unsigned TMR0L6 :1;
        unsigned TMR0L7 :1;
    };
} TMR0Lbits_t;
extern volatile TMR0Lbits_t TMR0Lbits __attribute__((address(0x19C)));
# 1403 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
typedef union {
    struct {
        unsigned TMR0L :8;
    };
    struct {
        unsigned TMR0L0 :1;
        unsigned TMR0L1 :1;
        unsigned TMR0L2 :1;
        unsigned TMR0L3 :1;
        unsigned TMR0L4 :1;
        unsigned TMR0L5 :1;
        unsigned TMR0L6 :1;
        unsigned TMR0L7 :1;
    };
} TMR0bits_t;
extern volatile TMR0bits_t TMR0bits __attribute__((address(0x19C)));
# 1468 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char TMR0H __attribute__((address(0x19D)));

__asm("TMR0H equ 019Dh");


extern volatile unsigned char PR0 __attribute__((address(0x19D)));

__asm("PR0 equ 019Dh");


typedef union {
    struct {
        unsigned TMR0H :8;
    };
    struct {
        unsigned TMR0H0 :1;
        unsigned TMR0H1 :1;
        unsigned TMR0H2 :1;
        unsigned TMR0H3 :1;
        unsigned TMR0H4 :1;
        unsigned TMR0H5 :1;
        unsigned TMR0H6 :1;
        unsigned TMR0H7 :1;
    };
    struct {
        unsigned T0PR0 :1;
        unsigned T0PR1 :1;
        unsigned T0PR2 :1;
        unsigned T0PR3 :1;
        unsigned T0PR4 :1;
        unsigned T0PR5 :1;
        unsigned T0PR6 :1;
        unsigned T0PR7 :1;
    };
    struct {
        unsigned T0PR :8;
    };
} TMR0Hbits_t;
extern volatile TMR0Hbits_t TMR0Hbits __attribute__((address(0x19D)));
# 1599 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
typedef union {
    struct {
        unsigned TMR0H :8;
    };
    struct {
        unsigned TMR0H0 :1;
        unsigned TMR0H1 :1;
        unsigned TMR0H2 :1;
        unsigned TMR0H3 :1;
        unsigned TMR0H4 :1;
        unsigned TMR0H5 :1;
        unsigned TMR0H6 :1;
        unsigned TMR0H7 :1;
    };
    struct {
        unsigned T0PR0 :1;
        unsigned T0PR1 :1;
        unsigned T0PR2 :1;
        unsigned T0PR3 :1;
        unsigned T0PR4 :1;
        unsigned T0PR5 :1;
        unsigned T0PR6 :1;
        unsigned T0PR7 :1;
    };
    struct {
        unsigned T0PR :8;
    };
} PR0bits_t;
extern volatile PR0bits_t PR0bits __attribute__((address(0x19D)));
# 1722 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char T0CON0 __attribute__((address(0x19E)));

__asm("T0CON0 equ 019Eh");


typedef union {
    struct {
        unsigned OUTPS :4;
        unsigned MD16 :1;
        unsigned OUT :1;
        unsigned :1;
        unsigned EN :1;
    };
    struct {
        unsigned T0OUTPS :4;
        unsigned T0MD16 :1;
        unsigned T0OUT :1;
        unsigned :1;
        unsigned T0EN :1;
    };
    struct {
        unsigned OUTPS0 :1;
        unsigned OUTPS1 :1;
        unsigned OUTPS2 :1;
        unsigned OUTPS3 :1;
        unsigned T016BIT :1;
    };
    struct {
        unsigned T0OUTPS0 :1;
        unsigned T0OUTPS1 :1;
        unsigned T0OUTPS2 :1;
        unsigned T0OUTPS3 :1;
    };
} T0CON0bits_t;
extern volatile T0CON0bits_t T0CON0bits __attribute__((address(0x19E)));
# 1846 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char T0CON1 __attribute__((address(0x19F)));

__asm("T0CON1 equ 019Fh");


typedef union {
    struct {
        unsigned CKPS :4;
        unsigned ASYNC :1;
        unsigned CS :3;
    };
    struct {
        unsigned CKPS0 :1;
        unsigned CKPS1 :1;
        unsigned CKPS2 :1;
        unsigned CKPS3 :1;
        unsigned T0ASYNC :1;
        unsigned CS0 :1;
        unsigned CS1 :1;
        unsigned CS2 :1;
    };
    struct {
        unsigned T0CKPS :4;
        unsigned :1;
        unsigned T0CS :3;
    };
    struct {
        unsigned T0CKPS0 :1;
        unsigned T0CKPS1 :1;
        unsigned T0CKPS2 :1;
        unsigned T0CKPS3 :1;
        unsigned :1;
        unsigned T0CS0 :1;
        unsigned T0CS1 :1;
        unsigned T0CS2 :1;
    };
} T0CON1bits_t;
extern volatile T0CON1bits_t T0CON1bits __attribute__((address(0x19F)));
# 1988 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char OSCCON2 __attribute__((address(0x20E)));

__asm("OSCCON2 equ 020Eh");


typedef union {
    struct {
        unsigned :4;
        unsigned COSC :3;
    };
    struct {
        unsigned :4;
        unsigned COSC0 :1;
        unsigned COSC1 :1;
        unsigned COSC2 :1;
    };
} OSCCON2bits_t;
extern volatile OSCCON2bits_t OSCCON2bits __attribute__((address(0x20E)));
# 2030 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char OSCCON3 __attribute__((address(0x20F)));

__asm("OSCCON3 equ 020Fh");


typedef union {
    struct {
        unsigned :6;
        unsigned SOSCPWR :1;
    };
} OSCCON3bits_t;
extern volatile OSCCON3bits_t OSCCON3bits __attribute__((address(0x20F)));
# 2051 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char OSCSTAT __attribute__((address(0x210)));

__asm("OSCSTAT equ 0210h");


typedef union {
    struct {
        unsigned :1;
        unsigned SFOR :1;
        unsigned ADOR :1;
        unsigned SOR :1;
        unsigned LFOR :1;
        unsigned MFOR :1;
        unsigned HFOR :1;
    };
} OSCSTATbits_t;
extern volatile OSCSTATbits_t OSCSTATbits __attribute__((address(0x210)));
# 2102 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char OSCEN __attribute__((address(0x211)));

__asm("OSCEN equ 0211h");


typedef union {
    struct {
        unsigned :2;
        unsigned ADOEN :1;
        unsigned SOSCEN :1;
        unsigned LFOEN :1;
        unsigned MFOEN :1;
        unsigned HFOEN :1;
    };
} OSCENbits_t;
extern volatile OSCENbits_t OSCENbits __attribute__((address(0x211)));
# 2147 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char OSCTUNE __attribute__((address(0x212)));

__asm("OSCTUNE equ 0212h");


typedef union {
    struct {
        unsigned TUN :6;
    };
    struct {
        unsigned TUN0 :1;
        unsigned TUN1 :1;
        unsigned TUN2 :1;
        unsigned TUN3 :1;
        unsigned TUN4 :1;
        unsigned TUN5 :1;
    };
} OSCTUNEbits_t;
extern volatile OSCTUNEbits_t OSCTUNEbits __attribute__((address(0x212)));
# 2205 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char OSCFRQ __attribute__((address(0x213)));

__asm("OSCFRQ equ 0213h");


typedef union {
    struct {
        unsigned FRQ :3;
    };
    struct {
        unsigned FRQ0 :1;
        unsigned FRQ1 :1;
        unsigned FRQ2 :1;
    };
} OSCFRQbits_t;
extern volatile OSCFRQbits_t OSCFRQbits __attribute__((address(0x213)));
# 2245 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char ACTCON __attribute__((address(0x214)));

__asm("ACTCON equ 0214h");


typedef union {
    struct {
        unsigned :1;
        unsigned ACTORS :1;
        unsigned :1;
        unsigned ACTLOCK :1;
        unsigned :2;
        unsigned ACTUD :1;
        unsigned ACTEN :1;
    };
} ACTCONbits_t;
extern volatile ACTCONbits_t ACTCONbits __attribute__((address(0x214)));
# 2286 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned short TMR1 __attribute__((address(0x28C)));

__asm("TMR1 equ 028Ch");




extern volatile unsigned char TMR1L __attribute__((address(0x28C)));

__asm("TMR1L equ 028Ch");


typedef union {
    struct {
        unsigned TMR1L :8;
    };
    struct {
        unsigned TMR1L0 :1;
        unsigned TMR1L1 :1;
        unsigned TMR1L2 :1;
        unsigned TMR1L3 :1;
        unsigned TMR1L4 :1;
        unsigned TMR1L5 :1;
        unsigned TMR1L6 :1;
        unsigned TMR1L7 :1;
    };
    struct {
        unsigned TMR10 :1;
        unsigned TMR11 :1;
        unsigned TMR12 :1;
        unsigned TMR13 :1;
        unsigned TMR14 :1;
        unsigned TMR15 :1;
        unsigned TMR16 :1;
        unsigned TMR17 :1;
    };
    struct {
        unsigned CAL01 :1;
        unsigned CAL11 :1;
        unsigned CAL21 :1;
        unsigned CAL31 :1;
        unsigned CAL41 :1;
        unsigned CAL51 :1;
        unsigned CAL61 :1;
        unsigned CAL71 :1;
    };
} TMR1Lbits_t;
extern volatile TMR1Lbits_t TMR1Lbits __attribute__((address(0x28C)));
# 2463 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char TMR1H __attribute__((address(0x28D)));

__asm("TMR1H equ 028Dh");


typedef union {
    struct {
        unsigned TMR1H :8;
    };
    struct {
        unsigned TMR1H0 :1;
        unsigned TMR1H1 :1;
        unsigned TMR1H2 :1;
        unsigned TMR1H3 :1;
        unsigned TMR1H4 :1;
        unsigned TMR1H5 :1;
        unsigned TMR1H6 :1;
        unsigned TMR1H7 :1;
    };
    struct {
        unsigned TMR18 :1;
        unsigned TMR19 :1;
        unsigned TMR110 :1;
        unsigned TMR111 :1;
        unsigned TMR112 :1;
        unsigned TMR113 :1;
        unsigned TMR114 :1;
        unsigned TMR115 :1;
    };
} TMR1Hbits_t;
extern volatile TMR1Hbits_t TMR1Hbits __attribute__((address(0x28D)));
# 2583 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char T1CON __attribute__((address(0x28E)));

__asm("T1CON equ 028Eh");


typedef union {
    struct {
        unsigned ON :1;
        unsigned RD16 :1;
        unsigned nSYNC :1;
        unsigned :1;
        unsigned CKPS :2;
    };
    struct {
        unsigned TMR1ON :1;
        unsigned T1RD16 :1;
        unsigned nT1SYNC :1;
        unsigned :1;
        unsigned T1CKPS0 :1;
        unsigned T1CKPS1 :1;
    };
    struct {
        unsigned :4;
        unsigned CKPS0 :1;
        unsigned CKPS1 :1;
    };
    struct {
        unsigned :1;
        unsigned RD161 :1;
    };
} T1CONbits_t;
extern volatile T1CONbits_t T1CONbits __attribute__((address(0x28E)));
# 2679 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char T1GCON __attribute__((address(0x28F)));

__asm("T1GCON equ 028Fh");


extern volatile unsigned char PR1 __attribute__((address(0x28F)));

__asm("PR1 equ 028Fh");


typedef union {
    struct {
        unsigned :2;
        unsigned GVAL :1;
        unsigned GGO_nDONE :1;
        unsigned GSPM :1;
        unsigned GTM :1;
        unsigned GPOL :1;
        unsigned GE :1;
    };
    struct {
        unsigned :2;
        unsigned T1GVAL :1;
        unsigned T1GGO_nDONE :1;
        unsigned T1GSPM :1;
        unsigned T1GTM :1;
        unsigned T1GPOL :1;
        unsigned T1GE :1;
    };
    struct {
        unsigned :3;
        unsigned T1GGO :1;
    };
} T1GCONbits_t;
extern volatile T1GCONbits_t T1GCONbits __attribute__((address(0x28F)));
# 2781 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
typedef union {
    struct {
        unsigned :2;
        unsigned GVAL :1;
        unsigned GGO_nDONE :1;
        unsigned GSPM :1;
        unsigned GTM :1;
        unsigned GPOL :1;
        unsigned GE :1;
    };
    struct {
        unsigned :2;
        unsigned T1GVAL :1;
        unsigned T1GGO_nDONE :1;
        unsigned T1GSPM :1;
        unsigned T1GTM :1;
        unsigned T1GPOL :1;
        unsigned T1GE :1;
    };
    struct {
        unsigned :3;
        unsigned T1GGO :1;
    };
} PR1bits_t;
extern volatile PR1bits_t PR1bits __attribute__((address(0x28F)));
# 2875 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char T1GATE __attribute__((address(0x290)));

__asm("T1GATE equ 0290h");


extern volatile unsigned char TMR1GATE __attribute__((address(0x290)));

__asm("TMR1GATE equ 0290h");


typedef union {
    struct {
        unsigned GSS :5;
    };
    struct {
        unsigned GSS0 :1;
        unsigned GSS1 :1;
        unsigned GSS2 :1;
        unsigned GSS3 :1;
        unsigned GSS4 :1;
    };
    struct {
        unsigned T1GSS0 :1;
        unsigned T1GSS1 :1;
        unsigned T1GSS2 :1;
        unsigned T1GSS3 :1;
        unsigned T1GSS4 :1;
    };
} T1GATEbits_t;
extern volatile T1GATEbits_t T1GATEbits __attribute__((address(0x290)));
# 2962 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
typedef union {
    struct {
        unsigned GSS :5;
    };
    struct {
        unsigned GSS0 :1;
        unsigned GSS1 :1;
        unsigned GSS2 :1;
        unsigned GSS3 :1;
        unsigned GSS4 :1;
    };
    struct {
        unsigned T1GSS0 :1;
        unsigned T1GSS1 :1;
        unsigned T1GSS2 :1;
        unsigned T1GSS3 :1;
        unsigned T1GSS4 :1;
    };
} TMR1GATEbits_t;
extern volatile TMR1GATEbits_t TMR1GATEbits __attribute__((address(0x290)));
# 3041 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char T1CLK __attribute__((address(0x291)));

__asm("T1CLK equ 0291h");


extern volatile unsigned char TMR1CLK __attribute__((address(0x291)));

__asm("TMR1CLK equ 0291h");


typedef union {
    struct {
        unsigned CS :8;
    };
    struct {
        unsigned T1CS0 :1;
        unsigned T1CS1 :1;
        unsigned T1CS2 :1;
        unsigned T1CS3 :1;
        unsigned T1CS4 :1;
    };
    struct {
        unsigned CS0 :1;
        unsigned CS1 :1;
        unsigned CS2 :1;
        unsigned CS3 :1;
        unsigned CS4 :1;
    };
} T1CLKbits_t;
extern volatile T1CLKbits_t T1CLKbits __attribute__((address(0x291)));
# 3128 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
typedef union {
    struct {
        unsigned CS :8;
    };
    struct {
        unsigned T1CS0 :1;
        unsigned T1CS1 :1;
        unsigned T1CS2 :1;
        unsigned T1CS3 :1;
        unsigned T1CS4 :1;
    };
    struct {
        unsigned CS0 :1;
        unsigned CS1 :1;
        unsigned CS2 :1;
        unsigned CS3 :1;
        unsigned CS4 :1;
    };
} TMR1CLKbits_t;
extern volatile TMR1CLKbits_t TMR1CLKbits __attribute__((address(0x291)));
# 3207 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned short TMR3 __attribute__((address(0x292)));

__asm("TMR3 equ 0292h");




extern volatile unsigned char TMR3L __attribute__((address(0x292)));

__asm("TMR3L equ 0292h");


typedef union {
    struct {
        unsigned TMR3L :8;
    };
    struct {
        unsigned TMR3L0 :1;
        unsigned TMR3L1 :1;
        unsigned TMR3L2 :1;
        unsigned TMR3L3 :1;
        unsigned TMR3L4 :1;
        unsigned TMR3L5 :1;
        unsigned TMR3L6 :1;
        unsigned TMR3L7 :1;
    };
    struct {
        unsigned TMR30 :1;
        unsigned TMR31 :1;
        unsigned TMR32 :1;
        unsigned TMR33 :1;
        unsigned TMR34 :1;
        unsigned TMR35 :1;
        unsigned TMR36 :1;
        unsigned TMR37 :1;
    };
    struct {
        unsigned CAL03 :1;
        unsigned CAL13 :1;
        unsigned CAL23 :1;
        unsigned CAL33 :1;
        unsigned CAL43 :1;
        unsigned CAL53 :1;
        unsigned CAL63 :1;
        unsigned CAL73 :1;
    };
} TMR3Lbits_t;
extern volatile TMR3Lbits_t TMR3Lbits __attribute__((address(0x292)));
# 3384 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char TMR3H __attribute__((address(0x293)));

__asm("TMR3H equ 0293h");


typedef union {
    struct {
        unsigned TMR3H :8;
    };
    struct {
        unsigned TMR3H0 :1;
        unsigned TMR3H1 :1;
        unsigned TMR3H2 :1;
        unsigned TMR3H3 :1;
        unsigned TMR3H4 :1;
        unsigned TMR3H5 :1;
        unsigned TMR3H6 :1;
        unsigned TMR3H7 :1;
    };
    struct {
        unsigned TMR38 :1;
        unsigned TMR39 :1;
        unsigned TMR310 :1;
        unsigned TMR311 :1;
        unsigned TMR312 :1;
        unsigned TMR313 :1;
        unsigned TMR314 :1;
        unsigned TMR315 :1;
    };
} TMR3Hbits_t;
extern volatile TMR3Hbits_t TMR3Hbits __attribute__((address(0x293)));
# 3504 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char T3CON __attribute__((address(0x294)));

__asm("T3CON equ 0294h");


typedef union {
    struct {
        unsigned ON :1;
        unsigned RD16 :1;
        unsigned nSYNC :1;
        unsigned :1;
        unsigned CKPS :2;
    };
    struct {
        unsigned TMR3ON :1;
        unsigned T3RD16 :1;
        unsigned nT3SYNC :1;
        unsigned :1;
        unsigned T3CKPS0 :1;
        unsigned T3CKPS1 :1;
    };
    struct {
        unsigned :4;
        unsigned CKPS0 :1;
        unsigned CKPS1 :1;
    };
    struct {
        unsigned :1;
        unsigned RD163 :1;
    };
} T3CONbits_t;
extern volatile T3CONbits_t T3CONbits __attribute__((address(0x294)));
# 3600 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char T3GCON __attribute__((address(0x295)));

__asm("T3GCON equ 0295h");


extern volatile unsigned char PR3 __attribute__((address(0x295)));

__asm("PR3 equ 0295h");


typedef union {
    struct {
        unsigned :2;
        unsigned GVAL :1;
        unsigned GGO_nDONE :1;
        unsigned GSPM :1;
        unsigned GTM :1;
        unsigned GPOL :1;
        unsigned GE :1;
    };
    struct {
        unsigned :2;
        unsigned T3GVAL :1;
        unsigned T3GGO_nDONE :1;
        unsigned T3GSPM :1;
        unsigned T3GTM :1;
        unsigned T3GPOL :1;
        unsigned T3GE :1;
    };
    struct {
        unsigned :3;
        unsigned T3GGO :1;
    };
} T3GCONbits_t;
extern volatile T3GCONbits_t T3GCONbits __attribute__((address(0x295)));
# 3702 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
typedef union {
    struct {
        unsigned :2;
        unsigned GVAL :1;
        unsigned GGO_nDONE :1;
        unsigned GSPM :1;
        unsigned GTM :1;
        unsigned GPOL :1;
        unsigned GE :1;
    };
    struct {
        unsigned :2;
        unsigned T3GVAL :1;
        unsigned T3GGO_nDONE :1;
        unsigned T3GSPM :1;
        unsigned T3GTM :1;
        unsigned T3GPOL :1;
        unsigned T3GE :1;
    };
    struct {
        unsigned :3;
        unsigned T3GGO :1;
    };
} PR3bits_t;
extern volatile PR3bits_t PR3bits __attribute__((address(0x295)));
# 3796 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char T3GATE __attribute__((address(0x296)));

__asm("T3GATE equ 0296h");


extern volatile unsigned char TMR3GATE __attribute__((address(0x296)));

__asm("TMR3GATE equ 0296h");


typedef union {
    struct {
        unsigned GSS :5;
    };
    struct {
        unsigned GSS0 :1;
        unsigned GSS1 :1;
        unsigned GSS2 :1;
        unsigned GSS3 :1;
        unsigned GSS4 :1;
    };
    struct {
        unsigned T3GSS0 :1;
        unsigned T3GSS1 :1;
        unsigned T3GSS2 :1;
        unsigned T3GSS3 :1;
        unsigned T3GSS4 :1;
    };
} T3GATEbits_t;
extern volatile T3GATEbits_t T3GATEbits __attribute__((address(0x296)));
# 3883 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
typedef union {
    struct {
        unsigned GSS :5;
    };
    struct {
        unsigned GSS0 :1;
        unsigned GSS1 :1;
        unsigned GSS2 :1;
        unsigned GSS3 :1;
        unsigned GSS4 :1;
    };
    struct {
        unsigned T3GSS0 :1;
        unsigned T3GSS1 :1;
        unsigned T3GSS2 :1;
        unsigned T3GSS3 :1;
        unsigned T3GSS4 :1;
    };
} TMR3GATEbits_t;
extern volatile TMR3GATEbits_t TMR3GATEbits __attribute__((address(0x296)));
# 3962 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char T3CLK __attribute__((address(0x297)));

__asm("T3CLK equ 0297h");


extern volatile unsigned char TMR3CLK __attribute__((address(0x297)));

__asm("TMR3CLK equ 0297h");


typedef union {
    struct {
        unsigned CS :8;
    };
    struct {
        unsigned T3CS0 :1;
        unsigned T3CS1 :1;
        unsigned T3CS2 :1;
        unsigned T3CS3 :1;
        unsigned T3CS4 :1;
    };
    struct {
        unsigned CS0 :1;
        unsigned CS1 :1;
        unsigned CS2 :1;
        unsigned CS3 :1;
        unsigned CS4 :1;
    };
} T3CLKbits_t;
extern volatile T3CLKbits_t T3CLKbits __attribute__((address(0x297)));
# 4049 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
typedef union {
    struct {
        unsigned CS :8;
    };
    struct {
        unsigned T3CS0 :1;
        unsigned T3CS1 :1;
        unsigned T3CS2 :1;
        unsigned T3CS3 :1;
        unsigned T3CS4 :1;
    };
    struct {
        unsigned CS0 :1;
        unsigned CS1 :1;
        unsigned CS2 :1;
        unsigned CS3 :1;
        unsigned CS4 :1;
    };
} TMR3CLKbits_t;
extern volatile TMR3CLKbits_t TMR3CLKbits __attribute__((address(0x297)));
# 4128 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char T2TMR __attribute__((address(0x30C)));

__asm("T2TMR equ 030Ch");


extern volatile unsigned char TMR2 __attribute__((address(0x30C)));

__asm("TMR2 equ 030Ch");


typedef union {
    struct {
        unsigned T2TMR :8;
    };
    struct {
        unsigned TMR2 :8;
    };
} T2TMRbits_t;
extern volatile T2TMRbits_t T2TMRbits __attribute__((address(0x30C)));
# 4159 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
typedef union {
    struct {
        unsigned T2TMR :8;
    };
    struct {
        unsigned TMR2 :8;
    };
} TMR2bits_t;
extern volatile TMR2bits_t TMR2bits __attribute__((address(0x30C)));
# 4182 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char T2PR __attribute__((address(0x30D)));

__asm("T2PR equ 030Dh");


extern volatile unsigned char PR2 __attribute__((address(0x30D)));

__asm("PR2 equ 030Dh");


typedef union {
    struct {
        unsigned T2PR :8;
    };
    struct {
        unsigned PR2 :8;
    };
} T2PRbits_t;
extern volatile T2PRbits_t T2PRbits __attribute__((address(0x30D)));
# 4213 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
typedef union {
    struct {
        unsigned T2PR :8;
    };
    struct {
        unsigned PR2 :8;
    };
} PR2bits_t;
extern volatile PR2bits_t PR2bits __attribute__((address(0x30D)));
# 4236 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char T2CON __attribute__((address(0x30E)));

__asm("T2CON equ 030Eh");


typedef union {
    struct {
        unsigned OUTPS :4;
        unsigned CKPS :3;
        unsigned ON :1;
    };
    struct {
        unsigned T2OUTPS :4;
        unsigned T2CKPS :3;
        unsigned T2ON :1;
    };
    struct {
        unsigned T2OUTPS0 :1;
        unsigned T2OUTPS1 :1;
        unsigned T2OUTPS2 :1;
        unsigned T2OUTPS3 :1;
        unsigned T2CKPS0 :1;
        unsigned T2CKPS1 :1;
        unsigned T2CKPS2 :1;
    };
    struct {
        unsigned OUTPS0 :1;
        unsigned OUTPS1 :1;
        unsigned OUTPS2 :1;
        unsigned OUTPS3 :1;
        unsigned CKPS0 :1;
        unsigned CKPS1 :1;
        unsigned CKPS2 :1;
        unsigned TMR2ON :1;
    };
} T2CONbits_t;
extern volatile T2CONbits_t T2CONbits __attribute__((address(0x30E)));
# 4382 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char T2HLT __attribute__((address(0x30F)));

__asm("T2HLT equ 030Fh");


typedef union {
    struct {
        unsigned MODE :5;
        unsigned CSYNC :1;
        unsigned CPOL :1;
        unsigned PSYNC :1;
    };
    struct {
        unsigned MODE0 :1;
        unsigned MODE1 :1;
        unsigned MODE2 :1;
        unsigned MODE3 :1;
        unsigned MODE4 :1;
        unsigned CKSYNC :1;
        unsigned CKPOL :1;
    };
    struct {
        unsigned T2MODE :5;
        unsigned T2CSYNC :1;
        unsigned T2CPOL :1;
        unsigned T2PSYNC :1;
    };
    struct {
        unsigned T2MODE0 :1;
        unsigned T2MODE1 :1;
        unsigned T2MODE2 :1;
        unsigned T2MODE3 :1;
        unsigned T2MODE4 :1;
    };
} T2HLTbits_t;
extern volatile T2HLTbits_t T2HLTbits __attribute__((address(0x30F)));
# 4522 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char T2CLKCON __attribute__((address(0x310)));

__asm("T2CLKCON equ 0310h");


extern volatile unsigned char T2CLK __attribute__((address(0x310)));

__asm("T2CLK equ 0310h");


typedef union {
    struct {
        unsigned CS :8;
    };
    struct {
        unsigned CS0 :1;
        unsigned CS1 :1;
        unsigned CS2 :1;
        unsigned CS3 :1;
    };
    struct {
        unsigned T2CS :8;
    };
    struct {
        unsigned T2CS0 :1;
        unsigned T2CS1 :1;
        unsigned T2CS2 :1;
        unsigned T2CS3 :1;
    };
} T2CLKCONbits_t;
extern volatile T2CLKCONbits_t T2CLKCONbits __attribute__((address(0x310)));
# 4605 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
typedef union {
    struct {
        unsigned CS :8;
    };
    struct {
        unsigned CS0 :1;
        unsigned CS1 :1;
        unsigned CS2 :1;
        unsigned CS3 :1;
    };
    struct {
        unsigned T2CS :8;
    };
    struct {
        unsigned T2CS0 :1;
        unsigned T2CS1 :1;
        unsigned T2CS2 :1;
        unsigned T2CS3 :1;
    };
} T2CLKbits_t;
extern volatile T2CLKbits_t T2CLKbits __attribute__((address(0x310)));
# 4680 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char T2RST __attribute__((address(0x311)));

__asm("T2RST equ 0311h");


typedef union {
    struct {
        unsigned RSEL :8;
    };
    struct {
        unsigned RSEL0 :1;
        unsigned RSEL1 :1;
        unsigned RSEL2 :1;
        unsigned RSEL3 :1;
    };
    struct {
        unsigned T2RSEL :8;
    };
    struct {
        unsigned T2RSEL0 :1;
        unsigned T2RSEL1 :1;
        unsigned T2RSEL2 :1;
        unsigned T2RSEL3 :1;
    };
} T2RSTbits_t;
extern volatile T2RSTbits_t T2RSTbits __attribute__((address(0x311)));
# 4760 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char T4TMR __attribute__((address(0x312)));

__asm("T4TMR equ 0312h");


extern volatile unsigned char TMR4 __attribute__((address(0x312)));

__asm("TMR4 equ 0312h");


typedef union {
    struct {
        unsigned T4TMR :8;
    };
    struct {
        unsigned TMR4 :8;
    };
} T4TMRbits_t;
extern volatile T4TMRbits_t T4TMRbits __attribute__((address(0x312)));
# 4791 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
typedef union {
    struct {
        unsigned T4TMR :8;
    };
    struct {
        unsigned TMR4 :8;
    };
} TMR4bits_t;
extern volatile TMR4bits_t TMR4bits __attribute__((address(0x312)));
# 4814 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char T4PR __attribute__((address(0x313)));

__asm("T4PR equ 0313h");


extern volatile unsigned char PR4 __attribute__((address(0x313)));

__asm("PR4 equ 0313h");


typedef union {
    struct {
        unsigned T4PR :8;
    };
    struct {
        unsigned PR4 :8;
    };
} T4PRbits_t;
extern volatile T4PRbits_t T4PRbits __attribute__((address(0x313)));
# 4845 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
typedef union {
    struct {
        unsigned T4PR :8;
    };
    struct {
        unsigned PR4 :8;
    };
} PR4bits_t;
extern volatile PR4bits_t PR4bits __attribute__((address(0x313)));
# 4868 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char T4CON __attribute__((address(0x314)));

__asm("T4CON equ 0314h");


typedef union {
    struct {
        unsigned OUTPS :4;
        unsigned CKPS :3;
        unsigned ON :1;
    };
    struct {
        unsigned T4OUTPS :4;
        unsigned T4CKPS :3;
        unsigned T4ON :1;
    };
    struct {
        unsigned T4OUTPS0 :1;
        unsigned T4OUTPS1 :1;
        unsigned T4OUTPS2 :1;
        unsigned T4OUTPS3 :1;
        unsigned T4CKPS0 :1;
        unsigned T4CKPS1 :1;
        unsigned T4CKPS2 :1;
    };
    struct {
        unsigned OUTPS0 :1;
        unsigned OUTPS1 :1;
        unsigned OUTPS2 :1;
        unsigned OUTPS3 :1;
        unsigned CKPS0 :1;
        unsigned CKPS1 :1;
        unsigned CKPS2 :1;
        unsigned TMR4ON :1;
    };
} T4CONbits_t;
extern volatile T4CONbits_t T4CONbits __attribute__((address(0x314)));
# 5014 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char T4HLT __attribute__((address(0x315)));

__asm("T4HLT equ 0315h");


typedef union {
    struct {
        unsigned MODE :5;
        unsigned CSYNC :1;
        unsigned CPOL :1;
        unsigned PSYNC :1;
    };
    struct {
        unsigned MODE0 :1;
        unsigned MODE1 :1;
        unsigned MODE2 :1;
        unsigned MODE3 :1;
        unsigned MODE4 :1;
        unsigned CKSYNC :1;
        unsigned CKPOL :1;
    };
    struct {
        unsigned T4MODE :5;
        unsigned T4CSYNC :1;
        unsigned T4CPOL :1;
        unsigned T4PSYNC :1;
    };
    struct {
        unsigned T4MODE0 :1;
        unsigned T4MODE1 :1;
        unsigned T4MODE2 :1;
        unsigned T4MODE3 :1;
        unsigned T4MODE4 :1;
    };
} T4HLTbits_t;
extern volatile T4HLTbits_t T4HLTbits __attribute__((address(0x315)));
# 5154 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char T4CLKCON __attribute__((address(0x316)));

__asm("T4CLKCON equ 0316h");


extern volatile unsigned char T4CLK __attribute__((address(0x316)));

__asm("T4CLK equ 0316h");


typedef union {
    struct {
        unsigned CS :8;
    };
    struct {
        unsigned CS0 :1;
        unsigned CS1 :1;
        unsigned CS2 :1;
        unsigned CS3 :1;
    };
    struct {
        unsigned T4CS :8;
    };
    struct {
        unsigned T4CS0 :1;
        unsigned T4CS1 :1;
        unsigned T4CS2 :1;
        unsigned T4CS3 :1;
    };
} T4CLKCONbits_t;
extern volatile T4CLKCONbits_t T4CLKCONbits __attribute__((address(0x316)));
# 5237 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
typedef union {
    struct {
        unsigned CS :8;
    };
    struct {
        unsigned CS0 :1;
        unsigned CS1 :1;
        unsigned CS2 :1;
        unsigned CS3 :1;
    };
    struct {
        unsigned T4CS :8;
    };
    struct {
        unsigned T4CS0 :1;
        unsigned T4CS1 :1;
        unsigned T4CS2 :1;
        unsigned T4CS3 :1;
    };
} T4CLKbits_t;
extern volatile T4CLKbits_t T4CLKbits __attribute__((address(0x316)));
# 5312 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char T4RST __attribute__((address(0x317)));

__asm("T4RST equ 0317h");


typedef union {
    struct {
        unsigned RSEL :8;
    };
    struct {
        unsigned RSEL0 :1;
        unsigned RSEL1 :1;
        unsigned RSEL2 :1;
        unsigned RSEL3 :1;
    };
    struct {
        unsigned T4RSEL :8;
    };
    struct {
        unsigned T4RSEL0 :1;
        unsigned T4RSEL1 :1;
        unsigned T4RSEL2 :1;
        unsigned T4RSEL3 :1;
    };
} T4RSTbits_t;
extern volatile T4RSTbits_t T4RSTbits __attribute__((address(0x317)));
# 5392 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char T6TMR __attribute__((address(0x318)));

__asm("T6TMR equ 0318h");


extern volatile unsigned char TMR6 __attribute__((address(0x318)));

__asm("TMR6 equ 0318h");


typedef union {
    struct {
        unsigned T6TMR :8;
    };
    struct {
        unsigned TMR6 :8;
    };
} T6TMRbits_t;
extern volatile T6TMRbits_t T6TMRbits __attribute__((address(0x318)));
# 5423 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
typedef union {
    struct {
        unsigned T6TMR :8;
    };
    struct {
        unsigned TMR6 :8;
    };
} TMR6bits_t;
extern volatile TMR6bits_t TMR6bits __attribute__((address(0x318)));
# 5446 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char T6PR __attribute__((address(0x319)));

__asm("T6PR equ 0319h");


extern volatile unsigned char PR6 __attribute__((address(0x319)));

__asm("PR6 equ 0319h");


typedef union {
    struct {
        unsigned T6PR :8;
    };
    struct {
        unsigned PR6 :8;
    };
} T6PRbits_t;
extern volatile T6PRbits_t T6PRbits __attribute__((address(0x319)));
# 5477 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
typedef union {
    struct {
        unsigned T6PR :8;
    };
    struct {
        unsigned PR6 :8;
    };
} PR6bits_t;
extern volatile PR6bits_t PR6bits __attribute__((address(0x319)));
# 5500 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char T6CON __attribute__((address(0x31A)));

__asm("T6CON equ 031Ah");


typedef union {
    struct {
        unsigned OUTPS :4;
        unsigned CKPS :3;
        unsigned ON :1;
    };
    struct {
        unsigned T6OUTPS :4;
        unsigned T6CKPS :3;
        unsigned T6ON :1;
    };
    struct {
        unsigned T6OUTPS0 :1;
        unsigned T6OUTPS1 :1;
        unsigned T6OUTPS2 :1;
        unsigned T6OUTPS3 :1;
        unsigned T6CKPS0 :1;
        unsigned T6CKPS1 :1;
        unsigned T6CKPS2 :1;
    };
    struct {
        unsigned OUTPS0 :1;
        unsigned OUTPS1 :1;
        unsigned OUTPS2 :1;
        unsigned OUTPS3 :1;
        unsigned CKPS0 :1;
        unsigned CKPS1 :1;
        unsigned CKPS2 :1;
        unsigned TMR6ON :1;
    };
} T6CONbits_t;
extern volatile T6CONbits_t T6CONbits __attribute__((address(0x31A)));
# 5646 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char T6HLT __attribute__((address(0x31B)));

__asm("T6HLT equ 031Bh");


typedef union {
    struct {
        unsigned MODE :5;
        unsigned CSYNC :1;
        unsigned CPOL :1;
        unsigned PSYNC :1;
    };
    struct {
        unsigned MODE0 :1;
        unsigned MODE1 :1;
        unsigned MODE2 :1;
        unsigned MODE3 :1;
        unsigned MODE4 :1;
        unsigned CKSYNC :1;
        unsigned CKPOL :1;
    };
    struct {
        unsigned T6MODE :5;
        unsigned T6CSYNC :1;
        unsigned T6CPOL :1;
        unsigned T6PSYNC :1;
    };
    struct {
        unsigned T6MODE0 :1;
        unsigned T6MODE1 :1;
        unsigned T6MODE2 :1;
        unsigned T6MODE3 :1;
        unsigned T6MODE4 :1;
    };
} T6HLTbits_t;
extern volatile T6HLTbits_t T6HLTbits __attribute__((address(0x31B)));
# 5786 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char T6CLKCON __attribute__((address(0x31C)));

__asm("T6CLKCON equ 031Ch");


extern volatile unsigned char T6CLK __attribute__((address(0x31C)));

__asm("T6CLK equ 031Ch");


typedef union {
    struct {
        unsigned CS :8;
    };
    struct {
        unsigned CS0 :1;
        unsigned CS1 :1;
        unsigned CS2 :1;
        unsigned CS3 :1;
    };
    struct {
        unsigned T6CS :8;
    };
    struct {
        unsigned T6CS0 :1;
        unsigned T6CS1 :1;
        unsigned T6CS2 :1;
        unsigned T6CS3 :1;
    };
} T6CLKCONbits_t;
extern volatile T6CLKCONbits_t T6CLKCONbits __attribute__((address(0x31C)));
# 5869 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
typedef union {
    struct {
        unsigned CS :8;
    };
    struct {
        unsigned CS0 :1;
        unsigned CS1 :1;
        unsigned CS2 :1;
        unsigned CS3 :1;
    };
    struct {
        unsigned T6CS :8;
    };
    struct {
        unsigned T6CS0 :1;
        unsigned T6CS1 :1;
        unsigned T6CS2 :1;
        unsigned T6CS3 :1;
    };
} T6CLKbits_t;
extern volatile T6CLKbits_t T6CLKbits __attribute__((address(0x31C)));
# 5944 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char T6RST __attribute__((address(0x31D)));

__asm("T6RST equ 031Dh");


typedef union {
    struct {
        unsigned RSEL :8;
    };
    struct {
        unsigned RSEL0 :1;
        unsigned RSEL1 :1;
        unsigned RSEL2 :1;
        unsigned RSEL3 :1;
    };
    struct {
        unsigned T6RSEL :8;
    };
    struct {
        unsigned T6RSEL0 :1;
        unsigned T6RSEL1 :1;
        unsigned T6RSEL2 :1;
        unsigned T6RSEL3 :1;
    };
} T6RSTbits_t;
extern volatile T6RSTbits_t T6RSTbits __attribute__((address(0x31D)));
# 6024 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned short CCPR1 __attribute__((address(0x38C)));

__asm("CCPR1 equ 038Ch");




extern volatile unsigned char CCPR1L __attribute__((address(0x38C)));

__asm("CCPR1L equ 038Ch");


typedef union {
    struct {
        unsigned CCPRL :8;
    };
} CCPR1Lbits_t;
extern volatile CCPR1Lbits_t CCPR1Lbits __attribute__((address(0x38C)));
# 6051 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char CCPR1H __attribute__((address(0x38D)));

__asm("CCPR1H equ 038Dh");


typedef union {
    struct {
        unsigned CCPRH :8;
    };
} CCPR1Hbits_t;
extern volatile CCPR1Hbits_t CCPR1Hbits __attribute__((address(0x38D)));
# 6071 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char CCP1CON __attribute__((address(0x38E)));

__asm("CCP1CON equ 038Eh");


typedef union {
    struct {
        unsigned MODE :4;
        unsigned FMT :1;
        unsigned OUT :1;
        unsigned :1;
        unsigned EN :1;
    };
    struct {
        unsigned MODE0 :1;
        unsigned MODE1 :1;
        unsigned MODE2 :1;
        unsigned MODE3 :1;
    };
    struct {
        unsigned CCP1MODE :4;
        unsigned CCP1FMT :1;
        unsigned CCP1OUT :1;
        unsigned :1;
        unsigned CCP1EN :1;
    };
    struct {
        unsigned CCP1MODE0 :1;
        unsigned CCP1MODE1 :1;
        unsigned CCP1MODE2 :1;
        unsigned CCP1MODE3 :1;
    };
    struct {
        unsigned :7;
        unsigned P1M1 :1;
    };
} CCP1CONbits_t;
extern volatile CCP1CONbits_t CCP1CONbits __attribute__((address(0x38E)));
# 6198 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char CCP1CAP __attribute__((address(0x38F)));

__asm("CCP1CAP equ 038Fh");


typedef union {
    struct {
        unsigned CTS :6;
    };
    struct {
        unsigned CTS0 :1;
        unsigned CTS1 :1;
        unsigned CTS2 :1;
        unsigned CTS3 :1;
    };
    struct {
        unsigned CCP1CTS :6;
    };
    struct {
        unsigned CCP1CTS0 :1;
        unsigned CCP1CTS1 :1;
        unsigned CCP1CTS2 :1;
        unsigned CCP1CTS3 :1;
    };
} CCP1CAPbits_t;
extern volatile CCP1CAPbits_t CCP1CAPbits __attribute__((address(0x38F)));
# 6278 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned short CCPR2 __attribute__((address(0x390)));

__asm("CCPR2 equ 0390h");




extern volatile unsigned char CCPR2L __attribute__((address(0x390)));

__asm("CCPR2L equ 0390h");


typedef union {
    struct {
        unsigned CCPRL :8;
    };
} CCPR2Lbits_t;
extern volatile CCPR2Lbits_t CCPR2Lbits __attribute__((address(0x390)));
# 6305 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char CCPR2H __attribute__((address(0x391)));

__asm("CCPR2H equ 0391h");


typedef union {
    struct {
        unsigned CCPRH :8;
    };
} CCPR2Hbits_t;
extern volatile CCPR2Hbits_t CCPR2Hbits __attribute__((address(0x391)));
# 6325 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char CCP2CON __attribute__((address(0x392)));

__asm("CCP2CON equ 0392h");


typedef union {
    struct {
        unsigned MODE :4;
        unsigned FMT :1;
        unsigned OUT :1;
        unsigned :1;
        unsigned EN :1;
    };
    struct {
        unsigned MODE0 :1;
        unsigned MODE1 :1;
        unsigned MODE2 :1;
        unsigned MODE3 :1;
    };
    struct {
        unsigned CCP2MODE :4;
        unsigned CCP2FMT :1;
        unsigned CCP2OUT :1;
        unsigned :1;
        unsigned CCP2EN :1;
    };
    struct {
        unsigned CCP2MODE0 :1;
        unsigned CCP2MODE1 :1;
        unsigned CCP2MODE2 :1;
        unsigned CCP2MODE3 :1;
    };
    struct {
        unsigned :7;
        unsigned P2M1 :1;
    };
} CCP2CONbits_t;
extern volatile CCP2CONbits_t CCP2CONbits __attribute__((address(0x392)));
# 6452 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char CCP2CAP __attribute__((address(0x393)));

__asm("CCP2CAP equ 0393h");


typedef union {
    struct {
        unsigned CTS :6;
    };
    struct {
        unsigned CTS0 :1;
        unsigned CTS1 :1;
        unsigned CTS2 :1;
        unsigned CTS3 :1;
    };
    struct {
        unsigned CCP2CTS :6;
    };
    struct {
        unsigned CCP2CTS0 :1;
        unsigned CCP2CTS1 :1;
        unsigned CCP2CTS2 :1;
        unsigned CCP2CTS3 :1;
    };
} CCP2CAPbits_t;
extern volatile CCP2CAPbits_t CCP2CAPbits __attribute__((address(0x393)));
# 6532 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char CCPTMRS0 __attribute__((address(0x39F)));

__asm("CCPTMRS0 equ 039Fh");


typedef union {
    struct {
        unsigned C1TSEL :2;
        unsigned C2TSEL :2;
    };
} CCPTMRS0bits_t;
extern volatile CCPTMRS0bits_t CCPTMRS0bits __attribute__((address(0x39F)));
# 6558 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned short PWM3DC __attribute__((address(0x40C)));

__asm("PWM3DC equ 040Ch");




extern volatile unsigned char PWM3DCL __attribute__((address(0x40C)));

__asm("PWM3DCL equ 040Ch");


typedef union {
    struct {
        unsigned :6;
        unsigned DCL :2;
    };
} PWM3DCLbits_t;
extern volatile PWM3DCLbits_t PWM3DCLbits __attribute__((address(0x40C)));
# 6586 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char PWM3DCH __attribute__((address(0x40D)));

__asm("PWM3DCH equ 040Dh");


typedef union {
    struct {
        unsigned DCH :8;
    };
} PWM3DCHbits_t;
extern volatile PWM3DCHbits_t PWM3DCHbits __attribute__((address(0x40D)));
# 6606 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char PWM3CON __attribute__((address(0x40E)));

__asm("PWM3CON equ 040Eh");


typedef union {
    struct {
        unsigned :4;
        unsigned POL :1;
        unsigned OUT :1;
        unsigned :1;
        unsigned EN :1;
    };
    struct {
        unsigned :4;
        unsigned PWM3POL :1;
        unsigned PWM3OUT :1;
        unsigned :1;
        unsigned PWM3EN :1;
    };
} PWM3CONbits_t;
extern volatile PWM3CONbits_t PWM3CONbits __attribute__((address(0x40E)));
# 6662 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned short PWM4DC __attribute__((address(0x40F)));

__asm("PWM4DC equ 040Fh");




extern volatile unsigned char PWM4DCL __attribute__((address(0x40F)));

__asm("PWM4DCL equ 040Fh");


typedef union {
    struct {
        unsigned :6;
        unsigned DCL :2;
    };
} PWM4DCLbits_t;
extern volatile PWM4DCLbits_t PWM4DCLbits __attribute__((address(0x40F)));
# 6690 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char PWM4DCH __attribute__((address(0x410)));

__asm("PWM4DCH equ 0410h");


typedef union {
    struct {
        unsigned DCH :8;
    };
} PWM4DCHbits_t;
extern volatile PWM4DCHbits_t PWM4DCHbits __attribute__((address(0x410)));
# 6710 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char PWM4CON __attribute__((address(0x411)));

__asm("PWM4CON equ 0411h");


typedef union {
    struct {
        unsigned :4;
        unsigned POL :1;
        unsigned OUT :1;
        unsigned :1;
        unsigned EN :1;
    };
    struct {
        unsigned :4;
        unsigned PWM4POL :1;
        unsigned PWM4OUT :1;
        unsigned :1;
        unsigned PWM4EN :1;
    };
} PWM4CONbits_t;
extern volatile PWM4CONbits_t PWM4CONbits __attribute__((address(0x411)));
# 6766 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned short PWM5DC __attribute__((address(0x412)));

__asm("PWM5DC equ 0412h");




extern volatile unsigned char PWM5DCL __attribute__((address(0x412)));

__asm("PWM5DCL equ 0412h");


typedef union {
    struct {
        unsigned :6;
        unsigned DCL :2;
    };
} PWM5DCLbits_t;
extern volatile PWM5DCLbits_t PWM5DCLbits __attribute__((address(0x412)));
# 6794 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char PWM5DCH __attribute__((address(0x413)));

__asm("PWM5DCH equ 0413h");


typedef union {
    struct {
        unsigned DCH :8;
    };
} PWM5DCHbits_t;
extern volatile PWM5DCHbits_t PWM5DCHbits __attribute__((address(0x413)));
# 6814 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char PWM5CON __attribute__((address(0x414)));

__asm("PWM5CON equ 0414h");


typedef union {
    struct {
        unsigned :4;
        unsigned POL :1;
        unsigned OUT :1;
        unsigned :1;
        unsigned EN :1;
    };
    struct {
        unsigned :4;
        unsigned PWM5POL :1;
        unsigned PWM5OUT :1;
        unsigned :1;
        unsigned PWM5EN :1;
    };
} PWM5CONbits_t;
extern volatile PWM5CONbits_t PWM5CONbits __attribute__((address(0x414)));
# 6870 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char PWMTMRS0 __attribute__((address(0x41F)));

__asm("PWMTMRS0 equ 041Fh");


typedef union {
    struct {
        unsigned P3TSEL :2;
        unsigned P4TSEL :2;
        unsigned P5TSEL :2;
    };
} PWMTMRS0bits_t;
extern volatile PWMTMRS0bits_t PWMTMRS0bits __attribute__((address(0x41F)));
# 6903 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile __uint24 NCO1ACC __attribute__((address(0x48C)));


__asm("NCO1ACC equ 048Ch");




extern volatile unsigned char NCO1ACCL __attribute__((address(0x48C)));

__asm("NCO1ACCL equ 048Ch");


typedef union {
    struct {
        unsigned ACC :8;
    };
    struct {
        unsigned NCO1ACC0 :1;
        unsigned NCO1ACC1 :1;
        unsigned NCO1ACC2 :1;
        unsigned NCO1ACC3 :1;
        unsigned NCO1ACC4 :1;
        unsigned NCO1ACC5 :1;
        unsigned NCO1ACC6 :1;
        unsigned NCO1ACC7 :1;
    };
    struct {
        unsigned ACC0 :1;
        unsigned ACC1 :1;
        unsigned ACC2 :1;
        unsigned ACC3 :1;
        unsigned ACC4 :1;
        unsigned ACC5 :1;
        unsigned ACC6 :1;
        unsigned ACC7 :1;
    };
    struct {
        unsigned NCO1ACC :8;
    };
} NCO1ACCLbits_t;
extern volatile NCO1ACCLbits_t NCO1ACCLbits __attribute__((address(0x48C)));
# 7039 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char NCO1ACCH __attribute__((address(0x48D)));

__asm("NCO1ACCH equ 048Dh");


typedef union {
    struct {
        unsigned ACC :8;
    };
    struct {
        unsigned NCO1ACC8 :1;
        unsigned NCO1ACC9 :1;
        unsigned NCO1ACC10 :1;
        unsigned NCO1ACC11 :1;
        unsigned NCO1ACC12 :1;
        unsigned NCO1ACC13 :1;
        unsigned NCO1ACC14 :1;
        unsigned NCO1ACC15 :1;
    };
    struct {
        unsigned ACC8 :1;
        unsigned ACC9 :1;
        unsigned ACC10 :1;
        unsigned ACC11 :1;
        unsigned ACC12 :1;
        unsigned ACC13 :1;
        unsigned ACC14 :1;
        unsigned ACC15 :1;
    };
    struct {
        unsigned NCO1ACC :8;
    };
} NCO1ACCHbits_t;
extern volatile NCO1ACCHbits_t NCO1ACCHbits __attribute__((address(0x48D)));
# 7167 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char NCO1ACCU __attribute__((address(0x48E)));

__asm("NCO1ACCU equ 048Eh");


typedef union {
    struct {
        unsigned ACC :8;
    };
    struct {
        unsigned NCO1ACC16 :1;
        unsigned NCO1ACC17 :1;
        unsigned NCO1ACC18 :1;
        unsigned NCO1ACC19 :1;
    };
    struct {
        unsigned ACC16 :1;
        unsigned ACC17 :1;
        unsigned ACC18 :1;
        unsigned ACC19 :1;
    };
    struct {
        unsigned NCO1ACC :8;
    };
} NCO1ACCUbits_t;
extern volatile NCO1ACCUbits_t NCO1ACCUbits __attribute__((address(0x48E)));
# 7248 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile __uint24 NCO1INC __attribute__((address(0x48F)));


__asm("NCO1INC equ 048Fh");




extern volatile unsigned char NCO1INCL __attribute__((address(0x48F)));

__asm("NCO1INCL equ 048Fh");


typedef union {
    struct {
        unsigned INC :8;
    };
    struct {
        unsigned NCO1INC0 :1;
        unsigned NCO1INC1 :1;
        unsigned NCO1INC2 :1;
        unsigned NCO1INC3 :1;
        unsigned NCO1INC4 :1;
        unsigned NCO1INC5 :1;
        unsigned NCO1INC6 :1;
        unsigned NCO1INC7 :1;
    };
    struct {
        unsigned INC0 :1;
        unsigned INC1 :1;
        unsigned INC2 :1;
        unsigned INC3 :1;
        unsigned INC4 :1;
        unsigned INC5 :1;
        unsigned INC6 :1;
        unsigned INC7 :1;
    };
    struct {
        unsigned NCO1INC :8;
    };
} NCO1INCLbits_t;
extern volatile NCO1INCLbits_t NCO1INCLbits __attribute__((address(0x48F)));
# 7384 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char NCO1INCH __attribute__((address(0x490)));

__asm("NCO1INCH equ 0490h");


typedef union {
    struct {
        unsigned INC :8;
    };
    struct {
        unsigned NCO1INC8 :1;
        unsigned NCO1INC9 :1;
        unsigned NCO1INC10 :1;
        unsigned NCO1INC11 :1;
        unsigned NCO1INC12 :1;
        unsigned NCO1INC13 :1;
        unsigned NCO1INC14 :1;
        unsigned NCO1INC15 :1;
    };
    struct {
        unsigned INC8 :1;
        unsigned INC9 :1;
        unsigned INC10 :1;
        unsigned INC11 :1;
        unsigned INC12 :1;
        unsigned INC13 :1;
        unsigned INC14 :1;
        unsigned INC15 :1;
    };
    struct {
        unsigned NCO1INC :8;
    };
} NCO1INCHbits_t;
extern volatile NCO1INCHbits_t NCO1INCHbits __attribute__((address(0x490)));
# 7512 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char NCO1INCU __attribute__((address(0x491)));

__asm("NCO1INCU equ 0491h");


typedef union {
    struct {
        unsigned INC :8;
    };
    struct {
        unsigned NCO1INC16 :1;
        unsigned NCO1INC17 :1;
        unsigned NCO1INC18 :1;
        unsigned NCO1INC19 :1;
    };
    struct {
        unsigned INC16 :1;
        unsigned INC17 :1;
        unsigned INC18 :1;
        unsigned INC19 :1;
    };
    struct {
        unsigned NCO1INC :8;
    };
} NCO1INCUbits_t;
extern volatile NCO1INCUbits_t NCO1INCUbits __attribute__((address(0x491)));
# 7592 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char NCO1CON __attribute__((address(0x492)));

__asm("NCO1CON equ 0492h");


typedef union {
    struct {
        unsigned PFM :1;
        unsigned :3;
        unsigned POL :1;
        unsigned OUT :1;
        unsigned :1;
        unsigned EN :1;
    };
    struct {
        unsigned NCO1PFM :1;
        unsigned :3;
        unsigned NCO1POL :1;
        unsigned NCO1OUT :1;
        unsigned :1;
        unsigned NCO1EN :1;
    };
} NCO1CONbits_t;
extern volatile NCO1CONbits_t NCO1CONbits __attribute__((address(0x492)));
# 7660 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char NCO1CLK __attribute__((address(0x493)));

__asm("NCO1CLK equ 0493h");


typedef union {
    struct {
        unsigned CKS :5;
        unsigned PWS :3;
    };
    struct {
        unsigned CKS0 :1;
        unsigned CKS1 :1;
        unsigned CKS2 :1;
        unsigned CKS3 :1;
        unsigned :1;
        unsigned PWS0 :1;
        unsigned PWS1 :1;
        unsigned PWS2 :1;
    };
    struct {
        unsigned NCO1CKS :5;
        unsigned NCO1PWS :3;
    };
    struct {
        unsigned NCO1CKS0 :1;
        unsigned NCO1CKS1 :1;
        unsigned NCO1CKS2 :1;
        unsigned NCO1CKS3 :1;
        unsigned :1;
        unsigned NCO1PWS0 :1;
        unsigned NCO1PWS1 :1;
        unsigned NCO1PWS2 :1;
    };
} NCO1CLKbits_t;
extern volatile NCO1CLKbits_t NCO1CLKbits __attribute__((address(0x493)));
# 7790 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char CWG1CLK __attribute__((address(0x50C)));

__asm("CWG1CLK equ 050Ch");


extern volatile unsigned char CWG1CLKCON __attribute__((address(0x50C)));

__asm("CWG1CLKCON equ 050Ch");


typedef union {
    struct {
        unsigned CS :1;
    };
    struct {
        unsigned CWG1CS :1;
    };
} CWG1CLKbits_t;
extern volatile CWG1CLKbits_t CWG1CLKbits __attribute__((address(0x50C)));
# 7821 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
typedef union {
    struct {
        unsigned CS :1;
    };
    struct {
        unsigned CWG1CS :1;
    };
} CWG1CLKCONbits_t;
extern volatile CWG1CLKCONbits_t CWG1CLKCONbits __attribute__((address(0x50C)));
# 7844 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char CWG1ISM __attribute__((address(0x50D)));

__asm("CWG1ISM equ 050Dh");


extern volatile unsigned char CWG1DAT __attribute__((address(0x50D)));

__asm("CWG1DAT equ 050Dh");


typedef union {
    struct {
        unsigned ISM :8;
    };
    struct {
        unsigned CWG1ISM :8;
    };
    struct {
        unsigned ISM0 :1;
        unsigned ISM1 :1;
        unsigned ISM2 :1;
        unsigned ISM3 :1;
    };
    struct {
        unsigned DAT :8;
    };
    struct {
        unsigned DAT0 :1;
        unsigned DAT1 :1;
        unsigned DAT2 :1;
        unsigned DAT3 :1;
    };
} CWG1ISMbits_t;
extern volatile CWG1ISMbits_t CWG1ISMbits __attribute__((address(0x50D)));
# 7935 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
typedef union {
    struct {
        unsigned ISM :8;
    };
    struct {
        unsigned CWG1ISM :8;
    };
    struct {
        unsigned ISM0 :1;
        unsigned ISM1 :1;
        unsigned ISM2 :1;
        unsigned ISM3 :1;
    };
    struct {
        unsigned DAT :8;
    };
    struct {
        unsigned DAT0 :1;
        unsigned DAT1 :1;
        unsigned DAT2 :1;
        unsigned DAT3 :1;
    };
} CWG1DATbits_t;
extern volatile CWG1DATbits_t CWG1DATbits __attribute__((address(0x50D)));
# 8018 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char CWG1DBR __attribute__((address(0x50E)));

__asm("CWG1DBR equ 050Eh");


typedef union {
    struct {
        unsigned DBR :8;
    };
    struct {
        unsigned DBR0 :1;
        unsigned DBR1 :1;
        unsigned DBR2 :1;
        unsigned DBR3 :1;
        unsigned DBR4 :1;
        unsigned DBR5 :1;
    };
    struct {
        unsigned CWG1DBR :8;
    };
    struct {
        unsigned CWG1DBR0 :1;
        unsigned CWG1DBR1 :1;
        unsigned CWG1DBR2 :1;
        unsigned CWG1DBR3 :1;
        unsigned CWG1DBR4 :1;
        unsigned CWG1DBR5 :1;
    };
} CWG1DBRbits_t;
extern volatile CWG1DBRbits_t CWG1DBRbits __attribute__((address(0x50E)));
# 8122 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char CWG1DBF __attribute__((address(0x50F)));

__asm("CWG1DBF equ 050Fh");


typedef union {
    struct {
        unsigned DBF :8;
    };
    struct {
        unsigned DBF0 :1;
        unsigned DBF1 :1;
        unsigned DBF2 :1;
        unsigned DBF3 :1;
        unsigned DBF4 :1;
        unsigned DBF5 :1;
    };
    struct {
        unsigned CWG1DBF :8;
    };
    struct {
        unsigned CWG1DBF0 :1;
        unsigned CWG1DBF1 :1;
        unsigned CWG1DBF2 :1;
        unsigned CWG1DBF3 :1;
        unsigned CWG1DBF4 :1;
        unsigned CWG1DBF5 :1;
    };
} CWG1DBFbits_t;
extern volatile CWG1DBFbits_t CWG1DBFbits __attribute__((address(0x50F)));
# 8226 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char CWG1CON0 __attribute__((address(0x510)));

__asm("CWG1CON0 equ 0510h");


typedef union {
    struct {
        unsigned MODE :3;
        unsigned :3;
        unsigned LD :1;
        unsigned EN :1;
    };
    struct {
        unsigned MODE0 :1;
        unsigned MODE1 :1;
        unsigned MODE2 :1;
        unsigned :4;
        unsigned G1EN :1;
    };
    struct {
        unsigned CWG1MODE :3;
        unsigned :3;
        unsigned CWG1LD :1;
        unsigned CWG1EN :1;
    };
    struct {
        unsigned CWG1MODE0 :1;
        unsigned CWG1MODE1 :1;
        unsigned CWG1MODE2 :1;
    };
} CWG1CON0bits_t;
extern volatile CWG1CON0bits_t CWG1CON0bits __attribute__((address(0x510)));
# 8327 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char CWG1CON1 __attribute__((address(0x511)));

__asm("CWG1CON1 equ 0511h");


typedef union {
    struct {
        unsigned POLA :1;
        unsigned POLB :1;
        unsigned POLC :1;
        unsigned POLD :1;
        unsigned :1;
        unsigned IN :1;
    };
    struct {
        unsigned CWG1POLA :1;
        unsigned CWG1POLB :1;
        unsigned CWG1POLC :1;
        unsigned CWG1POLD :1;
        unsigned :1;
        unsigned CWG1IN :1;
    };
} CWG1CON1bits_t;
extern volatile CWG1CON1bits_t CWG1CON1bits __attribute__((address(0x511)));
# 8405 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char CWG1AS0 __attribute__((address(0x512)));

__asm("CWG1AS0 equ 0512h");


typedef union {
    struct {
        unsigned :2;
        unsigned LSAC :2;
        unsigned LSBD :2;
        unsigned REN :1;
        unsigned SHUTDOWN :1;
    };
    struct {
        unsigned :2;
        unsigned LSAC0 :1;
        unsigned LSCA1 :1;
        unsigned LSBD0 :1;
        unsigned LSBD1 :1;
    };
    struct {
        unsigned :2;
        unsigned CWG1LSAC0 :1;
        unsigned CWG1LSAC1 :1;
        unsigned CWG1LSBD0 :1;
        unsigned CWG1LSBD1 :1;
    };
    struct {
        unsigned :2;
        unsigned CWG1LSAC :2;
        unsigned CWG1LSBD :2;
    };
} CWG1AS0bits_t;
extern volatile CWG1AS0bits_t CWG1AS0bits __attribute__((address(0x512)));
# 8513 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char CWG1AS1 __attribute__((address(0x513)));

__asm("CWG1AS1 equ 0513h");


typedef union {
    struct {
        unsigned AS0E :1;
        unsigned AS1E :1;
        unsigned AS2E :1;
        unsigned AS3E :1;
        unsigned AS4E :1;
        unsigned AS5E :1;
        unsigned AS6E :1;
        unsigned AS7E :1;
    };
} CWG1AS1bits_t;
extern volatile CWG1AS1bits_t CWG1AS1bits __attribute__((address(0x513)));
# 8575 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char CWG1STR __attribute__((address(0x514)));

__asm("CWG1STR equ 0514h");


typedef union {
    struct {
        unsigned STRA :1;
        unsigned STRB :1;
        unsigned STRC :1;
        unsigned STRD :1;
        unsigned OVRA :1;
        unsigned OVRB :1;
        unsigned OVRC :1;
        unsigned OVRD :1;
    };
    struct {
        unsigned CWG1STRA :1;
        unsigned CWG1STRB :1;
        unsigned CWG1STRC :1;
        unsigned CWG1STRD :1;
        unsigned CWG1OVRA :1;
        unsigned CWG1OVRB :1;
        unsigned CWG1OVRC :1;
        unsigned CWG1OVRD :1;
    };
} CWG1STRbits_t;
extern volatile CWG1STRbits_t CWG1STRbits __attribute__((address(0x514)));
# 8687 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char CLCnCON __attribute__((address(0x58C)));

__asm("CLCnCON equ 058Ch");


typedef union {
    struct {
        unsigned MODE :3;
        unsigned INTN :1;
        unsigned INTP :1;
        unsigned OUT :1;
        unsigned :1;
        unsigned EN :1;
    };
    struct {
        unsigned MODE0 :1;
        unsigned MODE1 :1;
        unsigned MODE2 :1;
    };
} CLCnCONbits_t;
extern volatile CLCnCONbits_t CLCnCONbits __attribute__((address(0x58C)));
# 8752 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char CLCnPOL __attribute__((address(0x58D)));

__asm("CLCnPOL equ 058Dh");


typedef union {
    struct {
        unsigned G1POL :1;
        unsigned G2POL :1;
        unsigned G3POL :1;
        unsigned G4POL :1;
        unsigned :3;
        unsigned POL :1;
    };
} CLCnPOLbits_t;
extern volatile CLCnPOLbits_t CLCnPOLbits __attribute__((address(0x58D)));
# 8797 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char CLCnSEL0 __attribute__((address(0x58E)));

__asm("CLCnSEL0 equ 058Eh");


typedef union {
    struct {
        unsigned D1S :8;
    };
    struct {
        unsigned D1S0 :1;
        unsigned D1S1 :1;
        unsigned D1S2 :1;
        unsigned D1S3 :1;
        unsigned D1S4 :1;
        unsigned D1S5 :1;
    };
} CLCnSEL0bits_t;
extern volatile CLCnSEL0bits_t CLCnSEL0bits __attribute__((address(0x58E)));
# 8855 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char CLCnSEL1 __attribute__((address(0x58F)));

__asm("CLCnSEL1 equ 058Fh");


typedef union {
    struct {
        unsigned D2S :8;
    };
    struct {
        unsigned D2S0 :1;
        unsigned D2S1 :1;
        unsigned D2S2 :1;
        unsigned D2S3 :1;
        unsigned D2S4 :1;
        unsigned D2S5 :1;
    };
} CLCnSEL1bits_t;
extern volatile CLCnSEL1bits_t CLCnSEL1bits __attribute__((address(0x58F)));
# 8913 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char CLCnSEL2 __attribute__((address(0x590)));

__asm("CLCnSEL2 equ 0590h");


typedef union {
    struct {
        unsigned D3S :8;
    };
    struct {
        unsigned D3S0 :1;
        unsigned D3S1 :1;
        unsigned D3S2 :1;
        unsigned D3S3 :1;
        unsigned D3S4 :1;
        unsigned D3S5 :1;
    };
} CLCnSEL2bits_t;
extern volatile CLCnSEL2bits_t CLCnSEL2bits __attribute__((address(0x590)));
# 8971 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char CLCnSEL3 __attribute__((address(0x591)));

__asm("CLCnSEL3 equ 0591h");


typedef union {
    struct {
        unsigned D4S :8;
    };
    struct {
        unsigned D4S0 :1;
        unsigned D4S1 :1;
        unsigned D4S2 :1;
        unsigned D4S3 :1;
        unsigned D4S4 :1;
        unsigned D4S5 :1;
    };
} CLCnSEL3bits_t;
extern volatile CLCnSEL3bits_t CLCnSEL3bits __attribute__((address(0x591)));
# 9029 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char CLCnGLS0 __attribute__((address(0x592)));

__asm("CLCnGLS0 equ 0592h");


typedef union {
    struct {
        unsigned G1D1N :1;
        unsigned G1D1T :1;
        unsigned G1D2N :1;
        unsigned G1D2T :1;
        unsigned G1D3N :1;
        unsigned G1D3T :1;
        unsigned G1D4N :1;
        unsigned G1D4T :1;
    };
} CLCnGLS0bits_t;
extern volatile CLCnGLS0bits_t CLCnGLS0bits __attribute__((address(0x592)));
# 9091 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char CLCnGLS1 __attribute__((address(0x593)));

__asm("CLCnGLS1 equ 0593h");


typedef union {
    struct {
        unsigned G2D1N :1;
        unsigned G2D1T :1;
        unsigned G2D2N :1;
        unsigned G2D2T :1;
        unsigned G2D3N :1;
        unsigned G2D3T :1;
        unsigned G2D4N :1;
        unsigned G2D4T :1;
    };
} CLCnGLS1bits_t;
extern volatile CLCnGLS1bits_t CLCnGLS1bits __attribute__((address(0x593)));
# 9153 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char CLCnGLS2 __attribute__((address(0x594)));

__asm("CLCnGLS2 equ 0594h");


typedef union {
    struct {
        unsigned G3D1N :1;
        unsigned G3D1T :1;
        unsigned G3D2N :1;
        unsigned G3D2T :1;
        unsigned G3D3N :1;
        unsigned G3D3T :1;
        unsigned G3D4N :1;
        unsigned G3D4T :1;
    };
} CLCnGLS2bits_t;
extern volatile CLCnGLS2bits_t CLCnGLS2bits __attribute__((address(0x594)));
# 9215 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char CLCnGLS3 __attribute__((address(0x595)));

__asm("CLCnGLS3 equ 0595h");


typedef union {
    struct {
        unsigned G4D1N :1;
        unsigned G4D1T :1;
        unsigned G4D2N :1;
        unsigned G4D2T :1;
        unsigned G4D3N :1;
        unsigned G4D3T :1;
        unsigned G4D4N :1;
        unsigned G4D4T :1;
    };
} CLCnGLS3bits_t;
extern volatile CLCnGLS3bits_t CLCnGLS3bits __attribute__((address(0x595)));
# 9277 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char CLCSELECT __attribute__((address(0x596)));

__asm("CLCSELECT equ 0596h");


typedef union {
    struct {
        unsigned SLCT :8;
    };
    struct {
        unsigned SLCT0 :1;
        unsigned SLCT1 :1;
        unsigned SLCT2 :1;
        unsigned SLCT3 :1;
    };
} CLCSELECTbits_t;
extern volatile CLCSELECTbits_t CLCSELECTbits __attribute__((address(0x596)));
# 9323 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char CLCDATA __attribute__((address(0x597)));

__asm("CLCDATA equ 0597h");


typedef union {
    struct {
        unsigned CLC1OUT :1;
        unsigned CLC2OUT :1;
        unsigned CLC3OUT :1;
        unsigned CLC4OUT :1;
    };
} CLCDATAbits_t;
extern volatile CLCDATAbits_t CLCDATAbits __attribute__((address(0x597)));
# 9361 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char RC1REG __attribute__((address(0x60C)));

__asm("RC1REG equ 060Ch");


typedef union {
    struct {
        unsigned RCREG :8;
    };
} RC1REGbits_t;
extern volatile RC1REGbits_t RC1REGbits __attribute__((address(0x60C)));
# 9381 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char TX1REG __attribute__((address(0x60D)));

__asm("TX1REG equ 060Dh");


typedef union {
    struct {
        unsigned TXREG :8;
    };
} TX1REGbits_t;
extern volatile TX1REGbits_t TX1REGbits __attribute__((address(0x60D)));
# 9401 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned short SP1BRG __attribute__((address(0x60E)));

__asm("SP1BRG equ 060Eh");




extern volatile unsigned char SP1BRGL __attribute__((address(0x60E)));

__asm("SP1BRGL equ 060Eh");


typedef union {
    struct {
        unsigned SPBRGL :8;
    };
} SP1BRGLbits_t;
extern volatile SP1BRGLbits_t SP1BRGLbits __attribute__((address(0x60E)));
# 9428 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char SP1BRGH __attribute__((address(0x60F)));

__asm("SP1BRGH equ 060Fh");


typedef union {
    struct {
        unsigned SPBRGH :8;
    };
} SP1BRGHbits_t;
extern volatile SP1BRGHbits_t SP1BRGHbits __attribute__((address(0x60F)));
# 9448 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char RC1STA __attribute__((address(0x610)));

__asm("RC1STA equ 0610h");


typedef union {
    struct {
        unsigned RX9D :1;
        unsigned OERR :1;
        unsigned FERR :1;
        unsigned ADDEN :1;
        unsigned CREN :1;
        unsigned SREN :1;
        unsigned RX9 :1;
        unsigned SPEN :1;
    };
} RC1STAbits_t;
extern volatile RC1STAbits_t RC1STAbits __attribute__((address(0x610)));
# 9510 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char TX1STA __attribute__((address(0x611)));

__asm("TX1STA equ 0611h");


typedef union {
    struct {
        unsigned TX9D :1;
        unsigned TRMT :1;
        unsigned BRGH :1;
        unsigned SENDB :1;
        unsigned SYNC :1;
        unsigned TXEN :1;
        unsigned TX9 :1;
        unsigned CSRC :1;
    };
} TX1STAbits_t;
extern volatile TX1STAbits_t TX1STAbits __attribute__((address(0x611)));
# 9572 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char BAUD1CON __attribute__((address(0x612)));

__asm("BAUD1CON equ 0612h");


typedef union {
    struct {
        unsigned ABDEN :1;
        unsigned WUE :1;
        unsigned :1;
        unsigned BRG16 :1;
        unsigned SCKP :1;
        unsigned :1;
        unsigned RCIDL :1;
        unsigned ABDOVF :1;
    };
} BAUD1CONbits_t;
extern volatile BAUD1CONbits_t BAUD1CONbits __attribute__((address(0x612)));
# 9624 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char RC2REG __attribute__((address(0x616)));

__asm("RC2REG equ 0616h");


typedef union {
    struct {
        unsigned RCREG :8;
    };
} RC2REGbits_t;
extern volatile RC2REGbits_t RC2REGbits __attribute__((address(0x616)));
# 9644 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char TX2REG __attribute__((address(0x617)));

__asm("TX2REG equ 0617h");


typedef union {
    struct {
        unsigned TXREG :8;
    };
} TX2REGbits_t;
extern volatile TX2REGbits_t TX2REGbits __attribute__((address(0x617)));
# 9664 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned short SP2BRG __attribute__((address(0x618)));

__asm("SP2BRG equ 0618h");




extern volatile unsigned char SP2BRGL __attribute__((address(0x618)));

__asm("SP2BRGL equ 0618h");


typedef union {
    struct {
        unsigned SPBRGL :8;
    };
} SP2BRGLbits_t;
extern volatile SP2BRGLbits_t SP2BRGLbits __attribute__((address(0x618)));
# 9691 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char SP2BRGH __attribute__((address(0x619)));

__asm("SP2BRGH equ 0619h");


typedef union {
    struct {
        unsigned SPBRGH :8;
    };
} SP2BRGHbits_t;
extern volatile SP2BRGHbits_t SP2BRGHbits __attribute__((address(0x619)));
# 9711 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char RC2STA __attribute__((address(0x61A)));

__asm("RC2STA equ 061Ah");


typedef union {
    struct {
        unsigned RX9D :1;
        unsigned OERR :1;
        unsigned FERR :1;
        unsigned ADDEN :1;
        unsigned CREN :1;
        unsigned SREN :1;
        unsigned RX9 :1;
        unsigned SPEN :1;
    };
} RC2STAbits_t;
extern volatile RC2STAbits_t RC2STAbits __attribute__((address(0x61A)));
# 9773 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char TX2STA __attribute__((address(0x61B)));

__asm("TX2STA equ 061Bh");


typedef union {
    struct {
        unsigned TX9D :1;
        unsigned TRMT :1;
        unsigned BRGH :1;
        unsigned SENDB :1;
        unsigned SYNC :1;
        unsigned TXEN :1;
        unsigned TX9 :1;
        unsigned CSRC :1;
    };
} TX2STAbits_t;
extern volatile TX2STAbits_t TX2STAbits __attribute__((address(0x61B)));
# 9835 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char BAUD2CON __attribute__((address(0x61C)));

__asm("BAUD2CON equ 061Ch");


typedef union {
    struct {
        unsigned ABDEN :1;
        unsigned WUE :1;
        unsigned :1;
        unsigned BRG16 :1;
        unsigned SCKP :1;
        unsigned :1;
        unsigned RCIDL :1;
        unsigned ABDOVF :1;
    };
} BAUD2CONbits_t;
extern volatile BAUD2CONbits_t BAUD2CONbits __attribute__((address(0x61C)));
# 9887 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char SSP1BUF __attribute__((address(0x68C)));

__asm("SSP1BUF equ 068Ch");


typedef union {
    struct {
        unsigned SSPBUF :8;
    };
} SSP1BUFbits_t;
extern volatile SSP1BUFbits_t SSP1BUFbits __attribute__((address(0x68C)));
# 9907 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char SSP1ADD __attribute__((address(0x68D)));

__asm("SSP1ADD equ 068Dh");


typedef union {
    struct {
        unsigned SSPADD :8;
    };
    struct {
        unsigned MSK0 :1;
        unsigned MSK1 :1;
        unsigned MSK2 :1;
        unsigned MSK3 :1;
        unsigned MSK4 :1;
        unsigned MSK5 :1;
        unsigned MSK6 :1;
        unsigned MSK7 :1;
    };
    struct {
        unsigned MSK01 :1;
        unsigned MSK11 :1;
        unsigned MSK21 :1;
        unsigned MSK31 :1;
        unsigned MSK41 :1;
        unsigned MSK51 :1;
        unsigned MSK61 :1;
        unsigned MSK71 :1;
    };
} SSP1ADDbits_t;
extern volatile SSP1ADDbits_t SSP1ADDbits __attribute__((address(0x68D)));
# 10027 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char SSP1MSK __attribute__((address(0x68E)));

__asm("SSP1MSK equ 068Eh");


typedef union {
    struct {
        unsigned SSPMSK :8;
    };
    struct {
        unsigned MSK0 :1;
        unsigned MSK1 :1;
        unsigned MSK2 :1;
        unsigned MSK3 :1;
        unsigned MSK4 :1;
        unsigned MSK5 :1;
        unsigned MSK6 :1;
        unsigned MSK7 :1;
    };
} SSP1MSKbits_t;
extern volatile SSP1MSKbits_t SSP1MSKbits __attribute__((address(0x68E)));
# 10097 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char SSP1STAT __attribute__((address(0x68F)));

__asm("SSP1STAT equ 068Fh");


typedef union {
    struct {
        unsigned BF :1;
        unsigned UA :1;
        unsigned R_nW :1;
        unsigned S :1;
        unsigned P :1;
        unsigned D_nA :1;
        unsigned CKE :1;
        unsigned SMP :1;
    };
    struct {
        unsigned :2;
        unsigned R_W :1;
        unsigned :2;
        unsigned D_A :1;
    };
    struct {
        unsigned :2;
        unsigned nW :1;
        unsigned :2;
        unsigned nA :1;
    };
    struct {
        unsigned :2;
        unsigned nWRITE :1;
        unsigned :2;
        unsigned nADDRESS :1;
    };
    struct {
        unsigned :2;
        unsigned READ_WRITE :1;
        unsigned :2;
        unsigned DATA_ADDRESS :1;
    };
    struct {
        unsigned :2;
        unsigned I2C_READ :1;
        unsigned I2C_START :1;
        unsigned I2C_STOP :1;
        unsigned I2C_DAT :1;
    };
    struct {
        unsigned BF1 :1;
        unsigned UA1 :1;
        unsigned R :1;
        unsigned START :1;
        unsigned STOP :1;
        unsigned D :1;
        unsigned CKE1 :1;
        unsigned SMP1 :1;
    };
    struct {
        unsigned :2;
        unsigned RW :1;
        unsigned START1 :1;
        unsigned STOP1 :1;
        unsigned DA :1;
    };
    struct {
        unsigned :2;
        unsigned RW1 :1;
        unsigned I2C_START1 :1;
        unsigned I2C_STOP2 :1;
        unsigned DA1 :1;
    };
    struct {
        unsigned :2;
        unsigned I2C_READ1 :1;
        unsigned S2 :1;
        unsigned P2 :1;
        unsigned DATA_ADDRESS1 :1;
    };
    struct {
        unsigned :2;
        unsigned READ_WRITE1 :1;
        unsigned :2;
        unsigned D_A1 :1;
    };
    struct {
        unsigned :2;
        unsigned R_W1 :1;
        unsigned :2;
        unsigned D_nA1 :1;
    };
    struct {
        unsigned :2;
        unsigned R_nW1 :1;
        unsigned :2;
        unsigned I2C_DAT1 :1;
    };
    struct {
        unsigned :2;
        unsigned nW2 :1;
        unsigned :2;
        unsigned nA2 :1;
    };
    struct {
        unsigned :2;
        unsigned nWRITE1 :1;
        unsigned :2;
        unsigned nADDRESS1 :1;
    };
} SSP1STATbits_t;
extern volatile SSP1STATbits_t SSP1STATbits __attribute__((address(0x68F)));
# 10461 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char SSP1CON1 __attribute__((address(0x690)));

__asm("SSP1CON1 equ 0690h");


typedef union {
    struct {
        unsigned SSPM :4;
        unsigned CKP :1;
        unsigned SSPEN :1;
        unsigned SSPOV :1;
        unsigned WCOL :1;
    };
    struct {
        unsigned SSPM0 :1;
        unsigned SSPM1 :1;
        unsigned SSPM2 :1;
        unsigned SSPM3 :1;
    };
    struct {
        unsigned SSPM01 :1;
        unsigned SSPM11 :1;
        unsigned SSPM21 :1;
        unsigned SSPM31 :1;
        unsigned CKP1 :1;
        unsigned SSPEN1 :1;
        unsigned SSPOV1 :1;
        unsigned WCOL1 :1;
    };
} SSP1CON1bits_t;
extern volatile SSP1CON1bits_t SSP1CON1bits __attribute__((address(0x690)));
# 10581 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char SSP1CON2 __attribute__((address(0x691)));

__asm("SSP1CON2 equ 0691h");


typedef union {
    struct {
        unsigned SEN :1;
        unsigned RSEN :1;
        unsigned PEN :1;
        unsigned RCEN :1;
        unsigned ACKEN :1;
        unsigned ACKDT :1;
        unsigned ACKSTAT :1;
        unsigned GCEN :1;
    };
    struct {
        unsigned :1;
        unsigned ADMSK :5;
    };
    struct {
        unsigned :1;
        unsigned ADMSK1 :1;
        unsigned ADMSK2 :1;
        unsigned ADMSK3 :1;
        unsigned ADMSK4 :1;
        unsigned ADMSK5 :1;
    };
    struct {
        unsigned SEN1 :1;
        unsigned ADMSK11 :1;
        unsigned ADMSK21 :1;
        unsigned ADMSK31 :1;
        unsigned ACKEN1 :1;
        unsigned ACKDT1 :1;
        unsigned ACKSTAT1 :1;
        unsigned GCEN1 :1;
    };
    struct {
        unsigned :1;
        unsigned RSEN1 :1;
        unsigned PEN1 :1;
        unsigned RCEN1 :1;
        unsigned ADMSK41 :1;
        unsigned ADMSK51 :1;
    };
} SSP1CON2bits_t;
extern volatile SSP1CON2bits_t SSP1CON2bits __attribute__((address(0x691)));
# 10768 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char SSP1CON3 __attribute__((address(0x692)));

__asm("SSP1CON3 equ 0692h");


typedef union {
    struct {
        unsigned DHEN :1;
        unsigned AHEN :1;
        unsigned SBCDE :1;
        unsigned SDAHT :1;
        unsigned BOEN :1;
        unsigned SCIE :1;
        unsigned PCIE :1;
        unsigned ACKTIM :1;
    };
} SSP1CON3bits_t;
extern volatile SSP1CON3bits_t SSP1CON3bits __attribute__((address(0x692)));
# 10830 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char SSP2BUF __attribute__((address(0x696)));

__asm("SSP2BUF equ 0696h");


typedef union {
    struct {
        unsigned SSPBUF :8;
    };
} SSP2BUFbits_t;
extern volatile SSP2BUFbits_t SSP2BUFbits __attribute__((address(0x696)));
# 10850 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char SSP2ADD __attribute__((address(0x697)));

__asm("SSP2ADD equ 0697h");


typedef union {
    struct {
        unsigned SSPADD :8;
    };
    struct {
        unsigned MSK0 :1;
        unsigned MSK1 :1;
        unsigned MSK2 :1;
        unsigned MSK3 :1;
        unsigned MSK4 :1;
        unsigned MSK5 :1;
        unsigned MSK6 :1;
        unsigned MSK7 :1;
    };
    struct {
        unsigned MSK02 :1;
        unsigned MSK12 :1;
        unsigned MSK22 :1;
        unsigned MSK32 :1;
        unsigned MSK42 :1;
        unsigned MSK52 :1;
        unsigned MSK62 :1;
        unsigned MSK72 :1;
    };
} SSP2ADDbits_t;
extern volatile SSP2ADDbits_t SSP2ADDbits __attribute__((address(0x697)));
# 10970 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char SSP2MSK __attribute__((address(0x698)));

__asm("SSP2MSK equ 0698h");


typedef union {
    struct {
        unsigned SSPMSK :8;
    };
    struct {
        unsigned MSK0 :1;
        unsigned MSK1 :1;
        unsigned MSK2 :1;
        unsigned MSK3 :1;
        unsigned MSK4 :1;
        unsigned MSK5 :1;
        unsigned MSK6 :1;
        unsigned MSK7 :1;
    };
} SSP2MSKbits_t;
extern volatile SSP2MSKbits_t SSP2MSKbits __attribute__((address(0x698)));
# 11040 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char SSP2STAT __attribute__((address(0x699)));

__asm("SSP2STAT equ 0699h");


typedef union {
    struct {
        unsigned BF :1;
        unsigned UA :1;
        unsigned R_nW :1;
        unsigned S :1;
        unsigned P :1;
        unsigned D_nA :1;
        unsigned CKE :1;
        unsigned SMP :1;
    };
    struct {
        unsigned :2;
        unsigned R_W :1;
        unsigned :2;
        unsigned D_A :1;
    };
    struct {
        unsigned :2;
        unsigned nW :1;
        unsigned :2;
        unsigned nA :1;
    };
    struct {
        unsigned :2;
        unsigned nWRITE :1;
        unsigned :2;
        unsigned nADDRESS :1;
    };
    struct {
        unsigned :2;
        unsigned READ_WRITE :1;
        unsigned :2;
        unsigned DATA_ADDRESS :1;
    };
    struct {
        unsigned :2;
        unsigned I2C_READ :1;
        unsigned I2C_START :1;
        unsigned I2C_STOP :1;
        unsigned I2C_DAT :1;
    };
    struct {
        unsigned BF2 :1;
        unsigned UA2 :1;
        unsigned R :1;
        unsigned START :1;
        unsigned STOP :1;
        unsigned D :1;
        unsigned CKE2 :1;
        unsigned SMP2 :1;
    };
    struct {
        unsigned :2;
        unsigned RW :1;
        unsigned START2 :1;
        unsigned STOP2 :1;
        unsigned DA :1;
    };
    struct {
        unsigned :2;
        unsigned RW2 :1;
        unsigned I2C_START2 :1;
        unsigned I2C_STOP2 :1;
        unsigned DA2 :1;
    };
    struct {
        unsigned :2;
        unsigned I2C_READ2 :1;
        unsigned S2 :1;
        unsigned P2 :1;
        unsigned DATA_ADDRESS2 :1;
    };
    struct {
        unsigned :2;
        unsigned READ_WRITE2 :1;
        unsigned :2;
        unsigned D_A2 :1;
    };
    struct {
        unsigned :2;
        unsigned R_W2 :1;
        unsigned :2;
        unsigned D_nA2 :1;
    };
    struct {
        unsigned :2;
        unsigned R_nW2 :1;
        unsigned :2;
        unsigned I2C_DAT2 :1;
    };
    struct {
        unsigned :2;
        unsigned nW2 :1;
        unsigned :2;
        unsigned nA2 :1;
    };
    struct {
        unsigned :2;
        unsigned nWRITE2 :1;
        unsigned :2;
        unsigned nADDRESS2 :1;
    };
} SSP2STATbits_t;
extern volatile SSP2STATbits_t SSP2STATbits __attribute__((address(0x699)));
# 11404 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char SSP2CON1 __attribute__((address(0x69A)));

__asm("SSP2CON1 equ 069Ah");


typedef union {
    struct {
        unsigned SSPM :4;
        unsigned CKP :1;
        unsigned SSPEN :1;
        unsigned SSPOV :1;
        unsigned WCOL :1;
    };
    struct {
        unsigned SSPM0 :1;
        unsigned SSPM1 :1;
        unsigned SSPM2 :1;
        unsigned SSPM3 :1;
    };
    struct {
        unsigned SSPM02 :1;
        unsigned SSPM12 :1;
        unsigned SSPM22 :1;
        unsigned SSPM32 :1;
        unsigned CKP2 :1;
        unsigned SSPEN2 :1;
        unsigned SSPOV2 :1;
        unsigned WCOL2 :1;
    };
} SSP2CON1bits_t;
extern volatile SSP2CON1bits_t SSP2CON1bits __attribute__((address(0x69A)));
# 11524 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char SSP2CON2 __attribute__((address(0x69B)));

__asm("SSP2CON2 equ 069Bh");


typedef union {
    struct {
        unsigned SEN :1;
        unsigned RSEN :1;
        unsigned PEN :1;
        unsigned RCEN :1;
        unsigned ACKEN :1;
        unsigned ACKDT :1;
        unsigned ACKSTAT :1;
        unsigned GCEN :1;
    };
    struct {
        unsigned :1;
        unsigned ADMSK :5;
    };
    struct {
        unsigned :1;
        unsigned ADMSK1 :1;
        unsigned ADMSK2 :1;
        unsigned ADMSK3 :1;
        unsigned ADMSK4 :1;
        unsigned ADMSK5 :1;
    };
    struct {
        unsigned SEN2 :1;
        unsigned ADMSK12 :1;
        unsigned ADMSK22 :1;
        unsigned ADMSK32 :1;
        unsigned ACKEN2 :1;
        unsigned ACKDT2 :1;
        unsigned ACKSTAT2 :1;
        unsigned GCEN2 :1;
    };
    struct {
        unsigned :1;
        unsigned RSEN2 :1;
        unsigned PEN2 :1;
        unsigned RCEN2 :1;
        unsigned ADMSK42 :1;
        unsigned ADMSK52 :1;
    };
} SSP2CON2bits_t;
extern volatile SSP2CON2bits_t SSP2CON2bits __attribute__((address(0x69B)));
# 11711 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char SSP2CON3 __attribute__((address(0x69C)));

__asm("SSP2CON3 equ 069Ch");


typedef union {
    struct {
        unsigned DHEN :1;
        unsigned AHEN :1;
        unsigned SBCDE :1;
        unsigned SDAHT :1;
        unsigned BOEN :1;
        unsigned SCIE :1;
        unsigned PCIE :1;
        unsigned ACKTIM :1;
    };
} SSP2CON3bits_t;
extern volatile SSP2CON3bits_t SSP2CON3bits __attribute__((address(0x69C)));
# 11773 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char FVRCON __attribute__((address(0x100C)));

__asm("FVRCON equ 0100Ch");


typedef union {
    struct {
        unsigned ADFVR :2;
        unsigned CDAFVR :2;
        unsigned TSRNG :1;
        unsigned TSEN :1;
        unsigned RDY :1;
        unsigned EN :1;
    };
    struct {
        unsigned ADFVR0 :1;
        unsigned ADFVR1 :1;
        unsigned CDAFVR0 :1;
        unsigned CDAFVR1 :1;
        unsigned :2;
        unsigned FVRRDY :1;
        unsigned FVREN :1;
    };
} FVRCONbits_t;
extern volatile FVRCONbits_t FVRCONbits __attribute__((address(0x100C)));
# 11862 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char CPCON __attribute__((address(0x100D)));

__asm("CPCON equ 0100Dh");


typedef union {
    struct {
        unsigned CPRDY :1;
        unsigned CPT :1;
        unsigned CPREQ :1;
        unsigned :2;
        unsigned CPOS :1;
        unsigned CPON :2;
    };
} CPCONbits_t;
extern volatile CPCONbits_t CPCONbits __attribute__((address(0x100D)));
# 11907 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char ZCDCON __attribute__((address(0x101F)));

__asm("ZCDCON equ 0101Fh");


typedef union {
    struct {
        unsigned INTN :1;
        unsigned INTP :1;
        unsigned :2;
        unsigned POL :1;
        unsigned OUT :1;
        unsigned :1;
        unsigned SEN :1;
    };
    struct {
        unsigned ZCDINTN :1;
        unsigned ZCDINTP :1;
        unsigned :2;
        unsigned ZCDPOL :1;
        unsigned ZCDOUT :1;
        unsigned :1;
        unsigned ZCDSEN :1;
    };
} ZCDCONbits_t;
extern volatile ZCDCONbits_t ZCDCONbits __attribute__((address(0x101F)));
# 11987 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char CM1CON0 __attribute__((address(0x108C)));

__asm("CM1CON0 equ 0108Ch");


typedef union {
    struct {
        unsigned SYNC :1;
        unsigned HYS :1;
        unsigned :2;
        unsigned POL :1;
        unsigned :1;
        unsigned OUT :1;
        unsigned EN :1;
    };
    struct {
        unsigned C1SYNC :1;
        unsigned C1HYS :1;
        unsigned :2;
        unsigned C1POL :1;
        unsigned :1;
        unsigned C1OUT :1;
        unsigned C1EN :1;
    };
} CM1CON0bits_t;
extern volatile CM1CON0bits_t CM1CON0bits __attribute__((address(0x108C)));
# 12067 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char CM1CON1 __attribute__((address(0x108D)));

__asm("CM1CON1 equ 0108Dh");


typedef union {
    struct {
        unsigned INTN :1;
        unsigned INTP :1;
    };
    struct {
        unsigned C1INTN :1;
        unsigned C1INTP :1;
    };
} CM1CON1bits_t;
extern volatile CM1CON1bits_t CM1CON1bits __attribute__((address(0x108D)));
# 12107 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char CM1NCH __attribute__((address(0x108E)));

__asm("CM1NCH equ 0108Eh");


typedef union {
    struct {
        unsigned NCH :3;
    };
    struct {
        unsigned NCH0 :1;
        unsigned NCH1 :1;
        unsigned NCH2 :1;
    };
    struct {
        unsigned C1NCH0 :1;
        unsigned C1NCH1 :1;
        unsigned C1NCH2 :1;
    };
} CM1NCHbits_t;
extern volatile CM1NCHbits_t CM1NCHbits __attribute__((address(0x108E)));
# 12167 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char CM1PCH __attribute__((address(0x108F)));

__asm("CM1PCH equ 0108Fh");


typedef union {
    struct {
        unsigned PCH :3;
    };
    struct {
        unsigned PCH0 :1;
        unsigned PCH1 :1;
        unsigned PCH2 :1;
    };
    struct {
        unsigned C1PCH0 :1;
        unsigned C1PCH1 :1;
        unsigned C1PCH2 :1;
    };
} CM1PCHbits_t;
extern volatile CM1PCHbits_t CM1PCHbits __attribute__((address(0x108F)));
# 12227 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char CMOUT __attribute__((address(0x109F)));

__asm("CMOUT equ 0109Fh");


extern volatile unsigned char CMSTAT __attribute__((address(0x109F)));

__asm("CMSTAT equ 0109Fh");


typedef union {
    struct {
        unsigned C1OUT :1;
    };
} CMOUTbits_t;
extern volatile CMOUTbits_t CMOUTbits __attribute__((address(0x109F)));







typedef union {
    struct {
        unsigned C1OUT :1;
    };
} CMSTATbits_t;
extern volatile CMSTATbits_t CMSTATbits __attribute__((address(0x109F)));
# 12265 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char DAC1CON __attribute__((address(0x110C)));

__asm("DAC1CON equ 0110Ch");


typedef union {
    struct {
        unsigned NSS :1;
        unsigned :1;
        unsigned PSS :2;
        unsigned OE :2;
        unsigned REFRNG :1;
        unsigned EN :1;
    };
    struct {
        unsigned :2;
        unsigned PSS0 :1;
        unsigned PSS1 :1;
        unsigned OE0 :1;
        unsigned OE1 :1;
    };
} DAC1CONbits_t;
extern volatile DAC1CONbits_t DAC1CONbits __attribute__((address(0x110C)));
# 12337 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char DAC1DATL __attribute__((address(0x110D)));

__asm("DAC1DATL equ 0110Dh");


typedef union {
    struct {
        unsigned DAC1R :8;
    };
    struct {
        unsigned DAT :8;
    };
} DAC1DATLbits_t;
extern volatile DAC1DATLbits_t DAC1DATLbits __attribute__((address(0x110D)));
# 12365 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned short NVMADR __attribute__((address(0x1C8C)));

__asm("NVMADR equ 01C8Ch");




extern volatile unsigned char NVMADRL __attribute__((address(0x1C8C)));

__asm("NVMADRL equ 01C8Ch");


typedef union {
    struct {
        unsigned NVMADR :8;
    };
    struct {
        unsigned NVMADR0 :1;
        unsigned NVMADR1 :1;
        unsigned NVMADR2 :1;
        unsigned NVMADR3 :1;
        unsigned NVMADR4 :1;
        unsigned NVMADR5 :1;
        unsigned NVMADR6 :1;
        unsigned NVMADR7 :1;
    };
} NVMADRLbits_t;
extern volatile NVMADRLbits_t NVMADRLbits __attribute__((address(0x1C8C)));
# 12442 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char NVMADRH __attribute__((address(0x1C8D)));

__asm("NVMADRH equ 01C8Dh");


typedef union {
    struct {
        unsigned NVMADR :7;
    };
    struct {
        unsigned NVMADR8 :1;
        unsigned NVMADR9 :1;
        unsigned NVMADR10 :1;
        unsigned NVMADR11 :1;
        unsigned NVMADR12 :1;
        unsigned NVMADR13 :1;
        unsigned NVMADR14 :1;
    };
} NVMADRHbits_t;
extern volatile NVMADRHbits_t NVMADRHbits __attribute__((address(0x1C8D)));
# 12506 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned short NVMDAT __attribute__((address(0x1C8E)));

__asm("NVMDAT equ 01C8Eh");




extern volatile unsigned char NVMDATL __attribute__((address(0x1C8E)));

__asm("NVMDATL equ 01C8Eh");


typedef union {
    struct {
        unsigned NVMDAT :8;
    };
    struct {
        unsigned NVMDAT0 :1;
        unsigned NVMDAT1 :1;
        unsigned NVMDAT2 :1;
        unsigned NVMDAT3 :1;
        unsigned NVMDAT4 :1;
        unsigned NVMDAT5 :1;
        unsigned NVMDAT6 :1;
        unsigned NVMDAT7 :1;
    };
} NVMDATLbits_t;
extern volatile NVMDATLbits_t NVMDATLbits __attribute__((address(0x1C8E)));
# 12583 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char NVMDATH __attribute__((address(0x1C8F)));

__asm("NVMDATH equ 01C8Fh");


typedef union {
    struct {
        unsigned NVMDAT :6;
    };
    struct {
        unsigned NVMDAT8 :1;
        unsigned NVMDAT9 :1;
        unsigned NVMDAT10 :1;
        unsigned NVMDAT11 :1;
        unsigned NVMDAT12 :1;
        unsigned NVMDAT13 :1;
    };
} NVMDATHbits_t;
extern volatile NVMDATHbits_t NVMDATHbits __attribute__((address(0x1C8F)));
# 12641 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char NVMCON1 __attribute__((address(0x1C90)));

__asm("NVMCON1 equ 01C90h");


typedef union {
    struct {
        unsigned RD :1;
        unsigned WR :1;
        unsigned WREN :1;
        unsigned WRERR :1;
        unsigned FREE :1;
        unsigned LWLO :1;
        unsigned NVMREGS :1;
    };
} NVMCON1bits_t;
extern volatile NVMCON1bits_t NVMCON1bits __attribute__((address(0x1C90)));
# 12697 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char NVMCON2 __attribute__((address(0x1C91)));

__asm("NVMCON2 equ 01C91h");


typedef union {
    struct {
        unsigned NVMCON2 :8;
    };
} NVMCON2bits_t;
extern volatile NVMCON2bits_t NVMCON2bits __attribute__((address(0x1C91)));
# 12717 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned short ADLTH __attribute__((address(0x1D0C)));

__asm("ADLTH equ 01D0Ch");




extern volatile unsigned char ADLTHL __attribute__((address(0x1D0C)));

__asm("ADLTHL equ 01D0Ch");


typedef union {
    struct {
        unsigned LTH :8;
    };
    struct {
        unsigned ADLTH0 :1;
        unsigned ADLTH1 :1;
        unsigned ADLTH2 :1;
        unsigned ADLTH3 :1;
        unsigned ADLTH4 :1;
        unsigned ADLTH5 :1;
        unsigned ADLTH6 :1;
        unsigned ADLTH7 :1;
    };
    struct {
        unsigned ADLTH :8;
    };
    struct {
        unsigned LTH0 :1;
        unsigned LTH1 :1;
        unsigned LTH2 :1;
        unsigned LTH3 :1;
        unsigned LTH4 :1;
        unsigned LTH5 :1;
        unsigned LTH6 :1;
        unsigned LTH7 :1;
    };
} ADLTHLbits_t;
extern volatile ADLTHLbits_t ADLTHLbits __attribute__((address(0x1D0C)));
# 12852 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char ADLTHH __attribute__((address(0x1D0D)));

__asm("ADLTHH equ 01D0Dh");


typedef union {
    struct {
        unsigned LTH :8;
    };
    struct {
        unsigned ADLTH8 :1;
        unsigned ADLTH9 :1;
        unsigned ADLTH10 :1;
        unsigned ADLTH11 :1;
        unsigned ADLTH12 :1;
        unsigned ADLTH13 :1;
        unsigned ADLTH14 :1;
        unsigned ADLTH15 :1;
    };
    struct {
        unsigned ADLTH :8;
    };
    struct {
        unsigned LTH8 :1;
        unsigned LTH9 :1;
        unsigned LTH10 :1;
        unsigned LTH11 :1;
        unsigned LTH12 :1;
        unsigned LTH13 :1;
        unsigned LTH14 :1;
        unsigned LTH15 :1;
    };
} ADLTHHbits_t;
extern volatile ADLTHHbits_t ADLTHHbits __attribute__((address(0x1D0D)));
# 12980 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned short ADUTH __attribute__((address(0x1D0E)));

__asm("ADUTH equ 01D0Eh");




extern volatile unsigned char ADUTHL __attribute__((address(0x1D0E)));

__asm("ADUTHL equ 01D0Eh");


typedef union {
    struct {
        unsigned UTH :8;
    };
    struct {
        unsigned ADUTH0 :1;
        unsigned ADUTH1 :1;
        unsigned ADUTH2 :1;
        unsigned ADUTH3 :1;
        unsigned ADUTH4 :1;
        unsigned ADUTH5 :1;
        unsigned ADUTH6 :1;
        unsigned ADUTH7 :1;
    };
    struct {
        unsigned ADUTH :8;
    };
    struct {
        unsigned UTH0 :1;
        unsigned UTH1 :1;
        unsigned UTH2 :1;
        unsigned UTH3 :1;
        unsigned UTH4 :1;
        unsigned UTH5 :1;
        unsigned UTH6 :1;
        unsigned UTH7 :1;
    };
} ADUTHLbits_t;
extern volatile ADUTHLbits_t ADUTHLbits __attribute__((address(0x1D0E)));
# 13115 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char ADUTHH __attribute__((address(0x1D0F)));

__asm("ADUTHH equ 01D0Fh");


typedef union {
    struct {
        unsigned UTH :8;
    };
    struct {
        unsigned ADUTH8 :1;
        unsigned ADUTH9 :1;
        unsigned ADUTH10 :1;
        unsigned ADUTH11 :1;
        unsigned ADUTH12 :1;
        unsigned ADUTH13 :1;
        unsigned ADUTH14 :1;
        unsigned ADUTH15 :1;
    };
    struct {
        unsigned ADUTH :8;
    };
    struct {
        unsigned UTH8 :1;
        unsigned UTH9 :1;
        unsigned UTH10 :1;
        unsigned UTH11 :1;
        unsigned UTH12 :1;
        unsigned UTH13 :1;
        unsigned UTH14 :1;
        unsigned UTH15 :1;
    };
} ADUTHHbits_t;
extern volatile ADUTHHbits_t ADUTHHbits __attribute__((address(0x1D0F)));
# 13243 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned short ADERR __attribute__((address(0x1D10)));

__asm("ADERR equ 01D10h");




extern volatile unsigned char ADERRL __attribute__((address(0x1D10)));

__asm("ADERRL equ 01D10h");


typedef union {
    struct {
        unsigned ERR :8;
    };
    struct {
        unsigned ADERR0 :1;
        unsigned ADERR1 :1;
        unsigned ADERR2 :1;
        unsigned ADERR3 :1;
        unsigned ADERR4 :1;
        unsigned ADERR5 :1;
        unsigned ADERR6 :1;
        unsigned ADERR7 :1;
    };
    struct {
        unsigned ADERR :8;
    };
    struct {
        unsigned ERR0 :1;
        unsigned ERR1 :1;
        unsigned ERR2 :1;
        unsigned ERR3 :1;
        unsigned ERR4 :1;
        unsigned ERR5 :1;
        unsigned ERR6 :1;
        unsigned ERR7 :1;
    };
} ADERRLbits_t;
extern volatile ADERRLbits_t ADERRLbits __attribute__((address(0x1D10)));
# 13378 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char ADERRH __attribute__((address(0x1D11)));

__asm("ADERRH equ 01D11h");


typedef union {
    struct {
        unsigned ERR :8;
    };
    struct {
        unsigned ADERR8 :1;
        unsigned ADERR9 :1;
        unsigned ADERR10 :1;
        unsigned ADERR11 :1;
        unsigned ADERR12 :1;
        unsigned ADERR13 :1;
        unsigned ADERR14 :1;
        unsigned ADERR15 :1;
    };
    struct {
        unsigned ADERR :8;
    };
    struct {
        unsigned ERR8 :1;
        unsigned ERR9 :1;
        unsigned ERR10 :1;
        unsigned ERR11 :1;
        unsigned ERR12 :1;
        unsigned ERR13 :1;
        unsigned ERR14 :1;
        unsigned ERR15 :1;
    };
} ADERRHbits_t;
extern volatile ADERRHbits_t ADERRHbits __attribute__((address(0x1D11)));
# 13506 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned short ADSTPT __attribute__((address(0x1D12)));

__asm("ADSTPT equ 01D12h");




extern volatile unsigned char ADSTPTL __attribute__((address(0x1D12)));

__asm("ADSTPTL equ 01D12h");


typedef union {
    struct {
        unsigned STPT :8;
    };
    struct {
        unsigned ADSTPT0 :1;
        unsigned ADSTPT1 :1;
        unsigned ADSTPT2 :1;
        unsigned ADSTPT3 :1;
        unsigned ADSTPT4 :1;
        unsigned ADSTPT5 :1;
        unsigned ADSTPT6 :1;
        unsigned ADSTPT7 :1;
    };
    struct {
        unsigned ADSTPT :8;
    };
    struct {
        unsigned STPT0 :1;
        unsigned STPT1 :1;
        unsigned STPT2 :1;
        unsigned STPT3 :1;
        unsigned STPT4 :1;
        unsigned STPT5 :1;
        unsigned STPT6 :1;
        unsigned STPT7 :1;
    };
} ADSTPTLbits_t;
extern volatile ADSTPTLbits_t ADSTPTLbits __attribute__((address(0x1D12)));
# 13641 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char ADSTPTH __attribute__((address(0x1D13)));

__asm("ADSTPTH equ 01D13h");


typedef union {
    struct {
        unsigned STPT :8;
    };
    struct {
        unsigned ADSTPT8 :1;
        unsigned ADSTPT9 :1;
        unsigned ADSTPT10 :1;
        unsigned ADSTPT11 :1;
        unsigned ADSTPT12 :1;
        unsigned ADSTPT13 :1;
        unsigned ADSTPT14 :1;
        unsigned ADSTPT15 :1;
    };
    struct {
        unsigned ADSTPT :8;
    };
    struct {
        unsigned STPT8 :1;
        unsigned STPT9 :1;
        unsigned STPT10 :1;
        unsigned STPT11 :1;
        unsigned STPT12 :1;
        unsigned STPT13 :1;
        unsigned STPT15 :1;
        unsigned STPT16 :1;
    };
} ADSTPTHbits_t;
extern volatile ADSTPTHbits_t ADSTPTHbits __attribute__((address(0x1D13)));
# 13769 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned short ADFLTR __attribute__((address(0x1D14)));

__asm("ADFLTR equ 01D14h");




extern volatile unsigned char ADFLTRL __attribute__((address(0x1D14)));

__asm("ADFLTRL equ 01D14h");


typedef union {
    struct {
        unsigned FLTR :8;
    };
    struct {
        unsigned ADFLTR0 :1;
        unsigned ADFLTR1 :1;
        unsigned ADFLTR2 :1;
        unsigned ADFLTR3 :1;
        unsigned ADFLTR4 :1;
        unsigned ADFLTR5 :1;
        unsigned ADFLTR6 :1;
        unsigned ADFLTR7 :1;
    };
    struct {
        unsigned ADFLTR :8;
    };
    struct {
        unsigned FLTR0 :1;
        unsigned FLTR1 :1;
        unsigned FLTR2 :1;
        unsigned FLTR3 :1;
        unsigned FLTR4 :1;
        unsigned FLTR5 :1;
        unsigned FLTR6 :1;
        unsigned FLTR7 :1;
    };
} ADFLTRLbits_t;
extern volatile ADFLTRLbits_t ADFLTRLbits __attribute__((address(0x1D14)));
# 13904 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char ADFLTRH __attribute__((address(0x1D15)));

__asm("ADFLTRH equ 01D15h");


typedef union {
    struct {
        unsigned FLTR :8;
    };
    struct {
        unsigned ADFLTR8 :1;
        unsigned ADFLTR9 :1;
        unsigned ADFLTR10 :1;
        unsigned ADFLTR11 :1;
        unsigned ADFLTR12 :1;
        unsigned ADFLTR13 :1;
        unsigned ADFLTR14 :1;
        unsigned ADFLTR15 :1;
    };
    struct {
        unsigned ADFLTR :8;
    };
    struct {
        unsigned FLTR8 :1;
        unsigned FLTR9 :1;
        unsigned FLTR10 :1;
        unsigned FLTR11 :1;
        unsigned FLTR12 :1;
        unsigned FLTR13 :1;
        unsigned FLTR14 :1;
        unsigned FLTR15 :1;
    };
} ADFLTRHbits_t;
extern volatile ADFLTRHbits_t ADFLTRHbits __attribute__((address(0x1D15)));
# 14033 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile __uint24 ADACC __attribute__((address(0x1D16)));


__asm("ADACC equ 01D16h");




extern volatile unsigned char ADACCL __attribute__((address(0x1D16)));

__asm("ADACCL equ 01D16h");


typedef union {
    struct {
        unsigned ACC :8;
    };
    struct {
        unsigned ADACC0 :1;
        unsigned ADACC1 :1;
        unsigned ADACC2 :1;
        unsigned ADACC3 :1;
        unsigned ADACC4 :1;
        unsigned ADACC5 :1;
        unsigned ADACC6 :1;
        unsigned ADACC7 :1;
    };
    struct {
        unsigned ADACC :8;
    };
    struct {
        unsigned ACC0 :1;
        unsigned ACC1 :1;
        unsigned ACC2 :1;
        unsigned ACC3 :1;
        unsigned ACC4 :1;
        unsigned ACC5 :1;
        unsigned ACC6 :1;
        unsigned ACC7 :1;
    };
} ADACCLbits_t;
extern volatile ADACCLbits_t ADACCLbits __attribute__((address(0x1D16)));
# 14169 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char ADACCH __attribute__((address(0x1D17)));

__asm("ADACCH equ 01D17h");


typedef union {
    struct {
        unsigned ACC :8;
    };
    struct {
        unsigned ADACC8 :1;
        unsigned ADACC9 :1;
        unsigned ADACC10 :1;
        unsigned ADACC11 :1;
        unsigned ADACC12 :1;
        unsigned ADACC13 :1;
        unsigned ADACC14 :1;
        unsigned ADACC15 :1;
    };
    struct {
        unsigned ADACC :8;
    };
    struct {
        unsigned ACC8 :1;
        unsigned ACC9 :1;
        unsigned ACC10 :1;
        unsigned ACC11 :1;
        unsigned ACC12 :1;
        unsigned ACC13 :1;
        unsigned ACC14 :1;
        unsigned ACC15 :1;
    };
} ADACCHbits_t;
extern volatile ADACCHbits_t ADACCHbits __attribute__((address(0x1D17)));
# 14297 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char ADACCU __attribute__((address(0x1D18)));

__asm("ADACCU equ 01D18h");


typedef union {
    struct {
        unsigned ACC :8;
    };
    struct {
        unsigned ADACC16 :1;
        unsigned ADACC17 :1;
    };
    struct {
        unsigned ADACC :8;
    };
    struct {
        unsigned ACC16 :1;
        unsigned ACC17 :1;
    };
} ADACCUbits_t;
extern volatile ADACCUbits_t ADACCUbits __attribute__((address(0x1D18)));
# 14353 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char ADCNT __attribute__((address(0x1D19)));

__asm("ADCNT equ 01D19h");


typedef union {
    struct {
        unsigned CNT :8;
    };
    struct {
        unsigned ADCNT0 :1;
        unsigned ADCNT1 :1;
        unsigned ADCNT2 :1;
        unsigned ADCNT3 :1;
        unsigned ADCNT4 :1;
        unsigned ADCNT5 :1;
        unsigned ADCNT6 :1;
        unsigned ADCNT7 :1;
    };
    struct {
        unsigned ADCNT :8;
    };
    struct {
        unsigned CNT0 :1;
        unsigned CNT1 :1;
        unsigned CNT2 :1;
        unsigned CNT3 :1;
        unsigned CNT4 :1;
        unsigned CNT5 :1;
        unsigned CNT6 :1;
        unsigned CNT7 :1;
    };
} ADCNTbits_t;
extern volatile ADCNTbits_t ADCNTbits __attribute__((address(0x1D19)));
# 14481 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char ADRPT __attribute__((address(0x1D1A)));

__asm("ADRPT equ 01D1Ah");


typedef union {
    struct {
        unsigned RPT :8;
    };
    struct {
        unsigned ADRPT0 :1;
        unsigned ADRPT1 :1;
        unsigned ADRPT2 :1;
        unsigned ADRPT3 :1;
        unsigned ADRPT4 :1;
        unsigned ADRPT5 :1;
        unsigned ADRPT6 :1;
        unsigned ADRPT7 :1;
    };
    struct {
        unsigned ADRPT :8;
    };
    struct {
        unsigned RPT0 :1;
        unsigned RPT1 :1;
        unsigned RPT2 :1;
        unsigned RPT3 :1;
        unsigned RPT4 :1;
        unsigned RPT5 :1;
        unsigned RPT6 :1;
        unsigned RPT7 :1;
    };
} ADRPTbits_t;
extern volatile ADRPTbits_t ADRPTbits __attribute__((address(0x1D1A)));
# 14609 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned short ADPREV __attribute__((address(0x1D1B)));

__asm("ADPREV equ 01D1Bh");




extern volatile unsigned char ADPREVL __attribute__((address(0x1D1B)));

__asm("ADPREVL equ 01D1Bh");


typedef union {
    struct {
        unsigned PREV :8;
    };
    struct {
        unsigned ADPREV0 :1;
        unsigned ADPREV1 :1;
        unsigned ADPREV2 :1;
        unsigned ADPREV3 :1;
        unsigned ADPREV4 :1;
        unsigned ADPREV5 :1;
        unsigned ADPREV6 :1;
        unsigned ADPREV7 :1;
    };
    struct {
        unsigned ADPREV :8;
    };
    struct {
        unsigned PREV0 :1;
        unsigned PREV1 :1;
        unsigned PREV2 :1;
        unsigned PREV3 :1;
        unsigned PREV4 :1;
        unsigned PREV5 :1;
        unsigned PREV6 :1;
        unsigned PREV7 :1;
    };
} ADPREVLbits_t;
extern volatile ADPREVLbits_t ADPREVLbits __attribute__((address(0x1D1B)));
# 14744 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char ADPREVH __attribute__((address(0x1D1C)));

__asm("ADPREVH equ 01D1Ch");


typedef union {
    struct {
        unsigned PREV :8;
    };
    struct {
        unsigned ADPREV8 :1;
        unsigned ADPREV9 :1;
        unsigned ADPREV10 :1;
        unsigned ADPREV11 :1;
        unsigned ADPREV12 :1;
        unsigned ADPREV13 :1;
        unsigned ADPREV14 :1;
        unsigned ADPREV15 :1;
    };
    struct {
        unsigned ADPREV :8;
    };
    struct {
        unsigned PREV8 :1;
        unsigned PREV9 :1;
        unsigned PREV10 :1;
        unsigned PREV11 :1;
        unsigned PREV12 :1;
        unsigned PREV13 :1;
        unsigned PREV14 :1;
        unsigned PREV15 :1;
    };
} ADPREVHbits_t;
extern volatile ADPREVHbits_t ADPREVHbits __attribute__((address(0x1D1C)));
# 14872 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned short ADRES __attribute__((address(0x1D1D)));

__asm("ADRES equ 01D1Dh");




extern volatile unsigned char ADRESL __attribute__((address(0x1D1D)));

__asm("ADRESL equ 01D1Dh");


typedef union {
    struct {
        unsigned RES :8;
    };
    struct {
        unsigned ADRES0 :1;
        unsigned ADRES1 :1;
        unsigned ADRES2 :1;
        unsigned ADRES3 :1;
        unsigned ADRES4 :1;
        unsigned ADRES5 :1;
        unsigned ADRES6 :1;
        unsigned ADRES7 :1;
    };
    struct {
        unsigned ADRES :8;
    };
    struct {
        unsigned RES0 :1;
        unsigned RES1 :1;
        unsigned RES2 :1;
        unsigned RES3 :1;
        unsigned RES4 :1;
        unsigned RES5 :1;
        unsigned RES6 :1;
        unsigned RES7 :1;
    };
} ADRESLbits_t;
extern volatile ADRESLbits_t ADRESLbits __attribute__((address(0x1D1D)));
# 15007 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char ADRESH __attribute__((address(0x1D1E)));

__asm("ADRESH equ 01D1Eh");


typedef union {
    struct {
        unsigned ADRES8 :1;
        unsigned ADRES9 :1;
        unsigned ADRES10 :1;
        unsigned ADRES11 :1;
        unsigned ADRES12 :1;
        unsigned ADRES13 :1;
        unsigned ADRES14 :1;
        unsigned ADRES15 :1;
    };
    struct {
        unsigned ADRES :8;
    };
    struct {
        unsigned RES8 :1;
        unsigned RES9 :1;
        unsigned RES10 :1;
        unsigned RES11 :1;
        unsigned RES12 :1;
        unsigned RES13 :1;
        unsigned RES14 :1;
        unsigned RES15 :1;
    };
} ADRESHbits_t;
extern volatile ADRESHbits_t ADRESHbits __attribute__((address(0x1D1E)));
# 15127 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char ADPCH __attribute__((address(0x1D1F)));

__asm("ADPCH equ 01D1Fh");


typedef union {
    struct {
        unsigned PCH :8;
    };
    struct {
        unsigned ADPCH0 :1;
        unsigned ADPCH1 :1;
        unsigned ADPCH2 :1;
        unsigned ADPCH3 :1;
        unsigned ADPCH4 :1;
        unsigned ADPCH5 :1;
    };
} ADPCHbits_t;
extern volatile ADPCHbits_t ADPCHbits __attribute__((address(0x1D1F)));
# 15185 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned short ADACQ __attribute__((address(0x1D21)));

__asm("ADACQ equ 01D21h");




extern volatile unsigned char ADACQL __attribute__((address(0x1D21)));

__asm("ADACQL equ 01D21h");


typedef union {
    struct {
        unsigned ACQ :8;
    };
    struct {
        unsigned ADACQ0 :1;
        unsigned ADACQ1 :1;
        unsigned ADACQ2 :1;
        unsigned ADACQ3 :1;
        unsigned ADACQ4 :1;
        unsigned ADACQ5 :1;
        unsigned ADACQ6 :1;
        unsigned ADACQ7 :1;
    };
    struct {
        unsigned ADACQ :8;
    };
    struct {
        unsigned ACQ0 :1;
        unsigned ACQ1 :1;
        unsigned ACQ2 :1;
        unsigned ACQ3 :1;
        unsigned ACQ4 :1;
        unsigned ACQ5 :1;
        unsigned ACQ6 :1;
        unsigned ACQ7 :1;
    };
} ADACQLbits_t;
extern volatile ADACQLbits_t ADACQLbits __attribute__((address(0x1D21)));
# 15320 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char ADACQH __attribute__((address(0x1D22)));

__asm("ADACQH equ 01D22h");


typedef union {
    struct {
        unsigned ACQ :5;
    };
    struct {
        unsigned ADACQ8 :1;
        unsigned ADACQ9 :1;
        unsigned ADACQ10 :1;
        unsigned ADACQ11 :1;
        unsigned ADACQ12 :1;
    };
    struct {
        unsigned ADACQ :5;
    };
    struct {
        unsigned ACQ8 :1;
        unsigned ACQ9 :1;
        unsigned ACQ10 :1;
        unsigned ACQ11 :1;
        unsigned ACQ12 :1;
    };
} ADACQHbits_t;
extern volatile ADACQHbits_t ADACQHbits __attribute__((address(0x1D22)));
# 15412 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char ADCAP __attribute__((address(0x1D23)));

__asm("ADCAP equ 01D23h");


typedef union {
    struct {
        unsigned CAP :8;
    };
    struct {
        unsigned ADCAP0 :1;
        unsigned ADCAP1 :1;
        unsigned ADCAP2 :1;
        unsigned ADCAP3 :1;
        unsigned ADCAP4 :1;
    };
} ADCAPbits_t;
extern volatile ADCAPbits_t ADCAPbits __attribute__((address(0x1D23)));
# 15464 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned short ADPRE __attribute__((address(0x1D24)));

__asm("ADPRE equ 01D24h");




extern volatile unsigned char ADPREL __attribute__((address(0x1D24)));

__asm("ADPREL equ 01D24h");


typedef union {
    struct {
        unsigned PRE :8;
    };
    struct {
        unsigned PRE0 :1;
        unsigned PRE1 :1;
        unsigned PRE2 :1;
        unsigned PRE3 :1;
        unsigned PRE4 :1;
        unsigned PRE5 :1;
        unsigned PRE6 :1;
        unsigned PRE7 :1;
    };
    struct {
        unsigned ADPRE :8;
    };
    struct {
        unsigned ADPRE0 :1;
        unsigned ADPRE1 :1;
        unsigned ADPRE2 :1;
        unsigned ADPRE3 :1;
        unsigned ADPRE4 :1;
        unsigned ADPRE5 :1;
        unsigned ADPRE6 :1;
        unsigned ADPRE7 :1;
    };
} ADPRELbits_t;
extern volatile ADPRELbits_t ADPRELbits __attribute__((address(0x1D24)));
# 15599 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char ADPREH __attribute__((address(0x1D25)));

__asm("ADPREH equ 01D25h");


typedef union {
    struct {
        unsigned PRE :5;
    };
    struct {
        unsigned PRE8 :1;
        unsigned PRE9 :1;
        unsigned PRE10 :1;
        unsigned PRE11 :1;
        unsigned PRE12 :1;
    };
    struct {
        unsigned ADPRE :5;
    };
    struct {
        unsigned ADPRE8 :1;
        unsigned ADPRE9 :1;
        unsigned ADPRE10 :1;
        unsigned ADPRE11 :1;
        unsigned ADPRE12 :1;
    };
} ADPREHbits_t;
extern volatile ADPREHbits_t ADPREHbits __attribute__((address(0x1D25)));
# 15691 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char ADCON0 __attribute__((address(0x1D26)));

__asm("ADCON0 equ 01D26h");


typedef union {
    struct {
        unsigned GO :1;
        unsigned :1;
        unsigned FM :2;
        unsigned CS :1;
        unsigned :1;
        unsigned CONT :1;
        unsigned ON :1;
    };
    struct {
        unsigned ADGO :1;
        unsigned :1;
        unsigned ADFM :2;
        unsigned ADCS :1;
        unsigned :1;
        unsigned ADCONT :1;
        unsigned ADON :1;
    };
    struct {
        unsigned DONE :1;
        unsigned :1;
        unsigned FM0 :1;
    };
    struct {
        unsigned GO_nDONE :1;
        unsigned :1;
        unsigned ADFM0 :1;
    };
} ADCON0bits_t;
extern volatile ADCON0bits_t ADCON0bits __attribute__((address(0x1D26)));
# 15801 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char ADCON1 __attribute__((address(0x1D27)));

__asm("ADCON1 equ 01D27h");


typedef union {
    struct {
        unsigned DSEN :1;
        unsigned PCSC :1;
        unsigned :3;
        unsigned GPOL :1;
        unsigned IPEN :1;
        unsigned PPOL :1;
    };
    struct {
        unsigned ADDSEN :1;
        unsigned :4;
        unsigned ADGPOL :1;
        unsigned ADIPEN :1;
        unsigned ADPPOL :1;
    };
} ADCON1bits_t;
extern volatile ADCON1bits_t ADCON1bits __attribute__((address(0x1D27)));
# 15873 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char ADCON2 __attribute__((address(0x1D28)));

__asm("ADCON2 equ 01D28h");


typedef union {
    struct {
        unsigned MD :3;
        unsigned ACLR :1;
        unsigned CRS :3;
        unsigned PSIS :1;
    };
    struct {
        unsigned ADMD0 :1;
        unsigned ADMD1 :1;
        unsigned ADMD2 :1;
        unsigned ADACLR :1;
        unsigned ADCRS0 :1;
        unsigned ADCRS1 :1;
        unsigned ADCRS2 :1;
        unsigned ADPSIS :1;
    };
    struct {
        unsigned ADMD :3;
        unsigned :1;
        unsigned ADCRS :3;
    };
    struct {
        unsigned MD0 :1;
        unsigned MD1 :1;
        unsigned MD2 :1;
        unsigned :1;
        unsigned CRS0 :1;
        unsigned CRS1 :1;
        unsigned CRS2 :1;
    };
} ADCON2bits_t;
extern volatile ADCON2bits_t ADCON2bits __attribute__((address(0x1D28)));
# 16015 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char ADCON3 __attribute__((address(0x1D29)));

__asm("ADCON3 equ 01D29h");


typedef union {
    struct {
        unsigned TMD :3;
        unsigned SOI :1;
        unsigned CALC :3;
    };
    struct {
        unsigned ADTMD0 :1;
        unsigned ADTMD1 :1;
        unsigned ADTMD2 :1;
        unsigned ADSOI :1;
        unsigned ADCALC0 :1;
        unsigned ADCALC1 :1;
        unsigned ADCALC2 :1;
    };
    struct {
        unsigned ADTMD :3;
        unsigned :1;
        unsigned ADCALC :3;
    };
    struct {
        unsigned TMD0 :1;
        unsigned TMD1 :1;
        unsigned TMD2 :1;
        unsigned :1;
        unsigned CALC0 :1;
        unsigned CALC1 :1;
        unsigned CALC2 :1;
    };
} ADCON3bits_t;
extern volatile ADCON3bits_t ADCON3bits __attribute__((address(0x1D29)));
# 16145 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char ADSTAT __attribute__((address(0x1D2A)));

__asm("ADSTAT equ 01D2Ah");


typedef union {
    struct {
        unsigned STAT :3;
        unsigned :1;
        unsigned MATH :1;
        unsigned LTHR :1;
        unsigned UTHR :1;
        unsigned AOV :1;
    };
    struct {
        unsigned ADSTAT0 :1;
        unsigned ADSTAT1 :1;
        unsigned ADSTAT2 :1;
        unsigned :1;
        unsigned ADMATH :1;
        unsigned ADLTHR :1;
        unsigned ADUTHR :1;
        unsigned ADAOV :1;
    };
    struct {
        unsigned ADSTAT :3;
        unsigned :4;
        unsigned ADOV :1;
    };
    struct {
        unsigned STAT0 :1;
        unsigned STAT1 :1;
        unsigned STAT2 :1;
        unsigned :4;
        unsigned OV :1;
    };
} ADSTATbits_t;
extern volatile ADSTATbits_t ADSTATbits __attribute__((address(0x1D2A)));
# 16277 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char ADREF __attribute__((address(0x1D2B)));

__asm("ADREF equ 01D2Bh");


typedef union {
    struct {
        unsigned PREF :4;
    };
    struct {
        unsigned ADPREF :4;
    };
    struct {
        unsigned PREF0 :1;
        unsigned PREF1 :1;
    };
    struct {
        unsigned ADPREF0 :1;
        unsigned ADPREF1 :1;
    };
} ADREFbits_t;
extern volatile ADREFbits_t ADREFbits __attribute__((address(0x1D2B)));
# 16333 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char ADACT __attribute__((address(0x1D2C)));

__asm("ADACT equ 01D2Ch");


typedef union {
    struct {
        unsigned ACT :8;
    };
    struct {
        unsigned ADACT0 :1;
        unsigned ADACT1 :1;
        unsigned ADACT2 :1;
        unsigned ADACT3 :1;
        unsigned ADACT4 :1;
        unsigned ADACT5 :1;
    };
    struct {
        unsigned ADACT :8;
    };
    struct {
        unsigned ACT0 :1;
        unsigned ACT1 :1;
        unsigned ACT2 :1;
        unsigned ACT3 :1;
        unsigned ACT4 :1;
        unsigned ACT5 :1;
    };
} ADACTbits_t;
extern volatile ADACTbits_t ADACTbits __attribute__((address(0x1D2C)));
# 16437 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char ADCLK __attribute__((address(0x1D2D)));

__asm("ADCLK equ 01D2Dh");


typedef union {
    struct {
        unsigned CS :8;
    };
    struct {
        unsigned ADCS0 :1;
        unsigned ADCS1 :1;
        unsigned ADCS2 :1;
        unsigned ADCS3 :1;
        unsigned ADCS4 :1;
        unsigned ADCS5 :1;
    };
    struct {
        unsigned ADCS :8;
    };
    struct {
        unsigned CS0 :1;
        unsigned CS1 :1;
        unsigned CS2 :1;
        unsigned CS3 :1;
        unsigned CS4 :1;
        unsigned CS5 :1;
    };
} ADCLKbits_t;
extern volatile ADCLKbits_t ADCLKbits __attribute__((address(0x1D2D)));
# 16541 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char ADCG1A __attribute__((address(0x1D2E)));

__asm("ADCG1A equ 01D2Eh");


typedef union {
    struct {
        unsigned CGA0 :1;
        unsigned CGA1 :1;
        unsigned CGA2 :1;
        unsigned :1;
        unsigned CGA4 :1;
        unsigned CGA5 :1;
    };
} ADCG1Abits_t;
extern volatile ADCG1Abits_t ADCG1Abits __attribute__((address(0x1D2E)));
# 16586 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char RA0PPS __attribute__((address(0x1D8C)));

__asm("RA0PPS equ 01D8Ch");


typedef union {
    struct {
        unsigned RA0PPS :5;
    };
} RA0PPSbits_t;
extern volatile RA0PPSbits_t RA0PPSbits __attribute__((address(0x1D8C)));
# 16606 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char RA1PPS __attribute__((address(0x1D8D)));

__asm("RA1PPS equ 01D8Dh");


typedef union {
    struct {
        unsigned RA1PPS :5;
    };
} RA1PPSbits_t;
extern volatile RA1PPSbits_t RA1PPSbits __attribute__((address(0x1D8D)));
# 16626 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char RA2PPS __attribute__((address(0x1D8E)));

__asm("RA2PPS equ 01D8Eh");


typedef union {
    struct {
        unsigned RA2PPS :5;
    };
} RA2PPSbits_t;
extern volatile RA2PPSbits_t RA2PPSbits __attribute__((address(0x1D8E)));
# 16646 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char RA4PPS __attribute__((address(0x1D90)));

__asm("RA4PPS equ 01D90h");


typedef union {
    struct {
        unsigned RA4PPS :5;
    };
} RA4PPSbits_t;
extern volatile RA4PPSbits_t RA4PPSbits __attribute__((address(0x1D90)));
# 16666 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char RA5PPS __attribute__((address(0x1D91)));

__asm("RA5PPS equ 01D91h");


typedef union {
    struct {
        unsigned RA5PPS :5;
    };
} RA5PPSbits_t;
extern volatile RA5PPSbits_t RA5PPSbits __attribute__((address(0x1D91)));
# 16686 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char PPSLOCK __attribute__((address(0x1E0C)));

__asm("PPSLOCK equ 01E0Ch");


typedef union {
    struct {
        unsigned PPSLOCKED :1;
    };
} PPSLOCKbits_t;
extern volatile PPSLOCKbits_t PPSLOCKbits __attribute__((address(0x1E0C)));
# 16706 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char INTPPS __attribute__((address(0x1E0D)));

__asm("INTPPS equ 01E0Dh");


typedef union {
    struct {
        unsigned INTPPS :5;
    };
    struct {
        unsigned INTPPS0 :1;
        unsigned INTPPS1 :1;
        unsigned INTPPS2 :1;
        unsigned INTPPS3 :1;
        unsigned INTPPS4 :1;
    };
} INTPPSbits_t;
extern volatile INTPPSbits_t INTPPSbits __attribute__((address(0x1E0D)));
# 16758 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char T0CKIPPS __attribute__((address(0x1E0E)));

__asm("T0CKIPPS equ 01E0Eh");


typedef union {
    struct {
        unsigned T0CKIPPS :5;
    };
    struct {
        unsigned T0CKIPPS0 :1;
        unsigned T0CKIPPS1 :1;
        unsigned T0CKIPPS2 :1;
        unsigned T0CKIPPS3 :1;
        unsigned T0CKIPPS4 :1;
    };
} T0CKIPPSbits_t;
extern volatile T0CKIPPSbits_t T0CKIPPSbits __attribute__((address(0x1E0E)));
# 16810 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char T1CKIPPS __attribute__((address(0x1E0F)));

__asm("T1CKIPPS equ 01E0Fh");


typedef union {
    struct {
        unsigned T1CKIPPS :5;
    };
    struct {
        unsigned T1CKIPPS0 :1;
        unsigned T1CKIPPS1 :1;
        unsigned T1CKIPPS2 :1;
        unsigned T1CKIPPS3 :1;
        unsigned T1CKIPPS4 :1;
    };
} T1CKIPPSbits_t;
extern volatile T1CKIPPSbits_t T1CKIPPSbits __attribute__((address(0x1E0F)));
# 16862 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char T1GPPS __attribute__((address(0x1E10)));

__asm("T1GPPS equ 01E10h");


typedef union {
    struct {
        unsigned T1GPPS :5;
    };
    struct {
        unsigned T1GPPS0 :1;
        unsigned T1GPPS1 :1;
        unsigned T1GPPS2 :1;
        unsigned T1GPPS3 :1;
        unsigned T1GPPS4 :1;
    };
} T1GPPSbits_t;
extern volatile T1GPPSbits_t T1GPPSbits __attribute__((address(0x1E10)));
# 16914 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char T3CKIPPS __attribute__((address(0x1E11)));

__asm("T3CKIPPS equ 01E11h");


typedef union {
    struct {
        unsigned T3CKIPPS :5;
    };
} T3CKIPPSbits_t;
extern volatile T3CKIPPSbits_t T3CKIPPSbits __attribute__((address(0x1E11)));
# 16934 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char T3GPPS __attribute__((address(0x1E12)));

__asm("T3GPPS equ 01E12h");


typedef union {
    struct {
        unsigned T3GPPS :5;
    };
} T3GPPSbits_t;
extern volatile T3GPPSbits_t T3GPPSbits __attribute__((address(0x1E12)));
# 16954 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char T2INPPS __attribute__((address(0x1E19)));

__asm("T2INPPS equ 01E19h");


typedef union {
    struct {
        unsigned T2INPPS :5;
    };
    struct {
        unsigned T2INPPS0 :1;
        unsigned T2INPPS1 :1;
        unsigned T2INPPS2 :1;
        unsigned T2INPPS3 :1;
        unsigned T2INPPS4 :1;
    };
} T2INPPSbits_t;
extern volatile T2INPPSbits_t T2INPPSbits __attribute__((address(0x1E19)));
# 17006 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char T4INPPS __attribute__((address(0x1E1A)));

__asm("T4INPPS equ 01E1Ah");


typedef union {
    struct {
        unsigned T4INPPS :5;
    };
} T4INPPSbits_t;
extern volatile T4INPPSbits_t T4INPPSbits __attribute__((address(0x1E1A)));
# 17026 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char T6INPPS __attribute__((address(0x1E1B)));

__asm("T6INPPS equ 01E1Bh");


typedef union {
    struct {
        unsigned T6INPPS :5;
    };
} T6INPPSbits_t;
extern volatile T6INPPSbits_t T6INPPSbits __attribute__((address(0x1E1B)));
# 17046 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char CCP1PPS __attribute__((address(0x1E1E)));

__asm("CCP1PPS equ 01E1Eh");


typedef union {
    struct {
        unsigned CCP1PPS :5;
    };
    struct {
        unsigned CCP1PPS0 :1;
        unsigned CCP1PPS1 :1;
        unsigned CCP1PPS2 :1;
        unsigned CCP1PPS3 :1;
        unsigned CCP1PPS4 :1;
    };
} CCP1PPSbits_t;
extern volatile CCP1PPSbits_t CCP1PPSbits __attribute__((address(0x1E1E)));
# 17098 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char CCP2PPS __attribute__((address(0x1E1F)));

__asm("CCP2PPS equ 01E1Fh");


typedef union {
    struct {
        unsigned CCP2PPS :5;
    };
    struct {
        unsigned CCP2PPS0 :1;
        unsigned CCP2PPS1 :1;
        unsigned CCP2PPS2 :1;
        unsigned CCP2PPS3 :1;
        unsigned CCP2PPS4 :1;
    };
} CCP2PPSbits_t;
extern volatile CCP2PPSbits_t CCP2PPSbits __attribute__((address(0x1E1F)));
# 17150 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char CWG1PPS __attribute__((address(0x1E39)));

__asm("CWG1PPS equ 01E39h");


typedef union {
    struct {
        unsigned CWG1PPS :5;
    };
    struct {
        unsigned CWG1PPS0 :1;
        unsigned CWG1PPS1 :1;
        unsigned CWG1PPS2 :1;
        unsigned CWG1PPS3 :1;
        unsigned CWG1PPS4 :1;
    };
} CWG1PPSbits_t;
extern volatile CWG1PPSbits_t CWG1PPSbits __attribute__((address(0x1E39)));
# 17202 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char CLCIN0PPS __attribute__((address(0x1E3D)));

__asm("CLCIN0PPS equ 01E3Dh");


typedef union {
    struct {
        unsigned CLCIN0PPS :5;
    };
    struct {
        unsigned CLCIN0PPS0 :1;
        unsigned CLCIN0PPS1 :1;
        unsigned CLCIN0PPS2 :1;
        unsigned CLCIN0PPS3 :1;
        unsigned CLCIN0PPS4 :1;
    };
} CLCIN0PPSbits_t;
extern volatile CLCIN0PPSbits_t CLCIN0PPSbits __attribute__((address(0x1E3D)));
# 17254 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char CLCIN1PPS __attribute__((address(0x1E3E)));

__asm("CLCIN1PPS equ 01E3Eh");


typedef union {
    struct {
        unsigned CLCIN1PPS :5;
    };
    struct {
        unsigned CLCIN1PPS0 :1;
        unsigned CLCIN1PPS1 :1;
        unsigned CLCIN1PPS2 :1;
        unsigned CLCIN1PPS3 :1;
        unsigned CLCIN1PPS4 :1;
    };
} CLCIN1PPSbits_t;
extern volatile CLCIN1PPSbits_t CLCIN1PPSbits __attribute__((address(0x1E3E)));
# 17306 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char CLCIN2PPS __attribute__((address(0x1E3F)));

__asm("CLCIN2PPS equ 01E3Fh");


typedef union {
    struct {
        unsigned CLCIN2PPS :5;
    };
    struct {
        unsigned CLCIN2PPS0 :1;
        unsigned CLCIN2PPS1 :1;
        unsigned CLCIN2PPS2 :1;
        unsigned CLCIN2PPS3 :1;
        unsigned CLCIN2PPS4 :1;
    };
} CLCIN2PPSbits_t;
extern volatile CLCIN2PPSbits_t CLCIN2PPSbits __attribute__((address(0x1E3F)));
# 17358 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char CLCIN3PPS __attribute__((address(0x1E40)));

__asm("CLCIN3PPS equ 01E40h");


typedef union {
    struct {
        unsigned CLCIN3PPS :5;
    };
    struct {
        unsigned CLCIN3PPS0 :1;
        unsigned CLCIN3PPS1 :1;
        unsigned CLCIN3PPS2 :1;
        unsigned CLCIN3PPS3 :1;
        unsigned CLCIN3PPS4 :1;
    };
} CLCIN3PPSbits_t;
extern volatile CLCIN3PPSbits_t CLCIN3PPSbits __attribute__((address(0x1E40)));
# 17410 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char RX1PPS __attribute__((address(0x1E41)));

__asm("RX1PPS equ 01E41h");


extern volatile unsigned char RX1DTPPS __attribute__((address(0x1E41)));

__asm("RX1DTPPS equ 01E41h");


typedef union {
    struct {
        unsigned RX1PPS :5;
    };
} RX1PPSbits_t;
extern volatile RX1PPSbits_t RX1PPSbits __attribute__((address(0x1E41)));







typedef union {
    struct {
        unsigned RX1PPS :5;
    };
} RX1DTPPSbits_t;
extern volatile RX1DTPPSbits_t RX1DTPPSbits __attribute__((address(0x1E41)));
# 17448 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char CK1PPS __attribute__((address(0x1E42)));

__asm("CK1PPS equ 01E42h");


extern volatile unsigned char TX1CKPPS __attribute__((address(0x1E42)));

__asm("TX1CKPPS equ 01E42h");


typedef union {
    struct {
        unsigned CK1PPS :5;
    };
} CK1PPSbits_t;
extern volatile CK1PPSbits_t CK1PPSbits __attribute__((address(0x1E42)));







typedef union {
    struct {
        unsigned CK1PPS :5;
    };
} TX1CKPPSbits_t;
extern volatile TX1CKPPSbits_t TX1CKPPSbits __attribute__((address(0x1E42)));
# 17486 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char RX2PPS __attribute__((address(0x1E43)));

__asm("RX2PPS equ 01E43h");


extern volatile unsigned char RX2DTPPS __attribute__((address(0x1E43)));

__asm("RX2DTPPS equ 01E43h");


typedef union {
    struct {
        unsigned RX2PPS :5;
    };
} RX2PPSbits_t;
extern volatile RX2PPSbits_t RX2PPSbits __attribute__((address(0x1E43)));







typedef union {
    struct {
        unsigned RX2PPS :5;
    };
} RX2DTPPSbits_t;
extern volatile RX2DTPPSbits_t RX2DTPPSbits __attribute__((address(0x1E43)));
# 17524 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char CK2PPS __attribute__((address(0x1E44)));

__asm("CK2PPS equ 01E44h");


extern volatile unsigned char TX2CKPPS __attribute__((address(0x1E44)));

__asm("TX2CKPPS equ 01E44h");


typedef union {
    struct {
        unsigned CK2PPS :5;
    };
} CK2PPSbits_t;
extern volatile CK2PPSbits_t CK2PPSbits __attribute__((address(0x1E44)));







typedef union {
    struct {
        unsigned CK2PPS :5;
    };
} TX2CKPPSbits_t;
extern volatile TX2CKPPSbits_t TX2CKPPSbits __attribute__((address(0x1E44)));
# 17562 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char SSP1CLKPPS __attribute__((address(0x1E47)));

__asm("SSP1CLKPPS equ 01E47h");


typedef union {
    struct {
        unsigned SSP1CLKPPS :5;
    };
    struct {
        unsigned SSP1CLKPPS0 :1;
        unsigned SSP1CLKPPS1 :1;
        unsigned SSP1CLKPPS2 :1;
        unsigned SSP1CLKPPS3 :1;
        unsigned SSP1CLKPPS4 :1;
    };
} SSP1CLKPPSbits_t;
extern volatile SSP1CLKPPSbits_t SSP1CLKPPSbits __attribute__((address(0x1E47)));
# 17614 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char SSP1DATPPS __attribute__((address(0x1E48)));

__asm("SSP1DATPPS equ 01E48h");


typedef union {
    struct {
        unsigned SSP1DATPPS :5;
    };
    struct {
        unsigned SSP1DATPPS0 :1;
        unsigned SSP1DATPPS1 :1;
        unsigned SSP1DATPPS2 :1;
        unsigned SSP1DATPPS3 :1;
        unsigned SSP1DATPPS4 :1;
    };
} SSP1DATPPSbits_t;
extern volatile SSP1DATPPSbits_t SSP1DATPPSbits __attribute__((address(0x1E48)));
# 17666 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char SSP1SSPPS __attribute__((address(0x1E49)));

__asm("SSP1SSPPS equ 01E49h");


typedef union {
    struct {
        unsigned SSP1SSPPS :5;
    };
    struct {
        unsigned SSP1SSPPS0 :1;
        unsigned SSP1SSPPS1 :1;
        unsigned SSP1SSPPS2 :1;
        unsigned SSP1SSPPS3 :1;
        unsigned SSP1SSPPS4 :1;
    };
} SSP1SSPPSbits_t;
extern volatile SSP1SSPPSbits_t SSP1SSPPSbits __attribute__((address(0x1E49)));
# 17718 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char SSP2CLKPPS __attribute__((address(0x1E4A)));

__asm("SSP2CLKPPS equ 01E4Ah");


typedef union {
    struct {
        unsigned SSP2CLKPPS :5;
    };
} SSP2CLKPPSbits_t;
extern volatile SSP2CLKPPSbits_t SSP2CLKPPSbits __attribute__((address(0x1E4A)));
# 17738 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char SSP2DATPPS __attribute__((address(0x1E4B)));

__asm("SSP2DATPPS equ 01E4Bh");


typedef union {
    struct {
        unsigned SSP2DATPPS :5;
    };
} SSP2DATPPSbits_t;
extern volatile SSP2DATPPSbits_t SSP2DATPPSbits __attribute__((address(0x1E4B)));
# 17758 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char SSP2SSPPS __attribute__((address(0x1E4C)));

__asm("SSP2SSPPS equ 01E4Ch");


typedef union {
    struct {
        unsigned SSP2SSPPS :5;
    };
} SSP2SSPPSbits_t;
extern volatile SSP2SSPPSbits_t SSP2SSPPSbits __attribute__((address(0x1E4C)));
# 17778 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char ADACTPPS __attribute__((address(0x1E50)));

__asm("ADACTPPS equ 01E50h");


typedef union {
    struct {
        unsigned ADACTPPS :5;
    };
} ADACTPPSbits_t;
extern volatile ADACTPPSbits_t ADACTPPSbits __attribute__((address(0x1E50)));
# 17798 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char ANSELA __attribute__((address(0x1E8C)));

__asm("ANSELA equ 01E8Ch");


typedef union {
    struct {
        unsigned ANSELA0 :1;
        unsigned ANSELA1 :1;
        unsigned ANSELA2 :1;
        unsigned :1;
        unsigned ANSELA4 :1;
        unsigned ANSELA5 :1;
    };
    struct {
        unsigned ANSA0 :1;
        unsigned ANSA1 :1;
        unsigned ANSA2 :1;
        unsigned :1;
        unsigned ANSA4 :1;
        unsigned ANSA5 :1;
    };
} ANSELAbits_t;
extern volatile ANSELAbits_t ANSELAbits __attribute__((address(0x1E8C)));
# 17876 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char WPUA __attribute__((address(0x1E8D)));

__asm("WPUA equ 01E8Dh");


typedef union {
    struct {
        unsigned WPUA0 :1;
        unsigned WPUA1 :1;
        unsigned WPUA2 :1;
        unsigned WPUA3 :1;
        unsigned WPUA4 :1;
        unsigned WPUA5 :1;
    };
} WPUAbits_t;
extern volatile WPUAbits_t WPUAbits __attribute__((address(0x1E8D)));
# 17926 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char ODCONA __attribute__((address(0x1E8E)));

__asm("ODCONA equ 01E8Eh");


typedef union {
    struct {
        unsigned ODCA0 :1;
        unsigned ODCA1 :1;
        unsigned ODCA2 :1;
        unsigned :1;
        unsigned ODCA4 :1;
        unsigned ODCA5 :1;
    };
} ODCONAbits_t;
extern volatile ODCONAbits_t ODCONAbits __attribute__((address(0x1E8E)));
# 17971 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char SLRCONA __attribute__((address(0x1E8F)));

__asm("SLRCONA equ 01E8Fh");


typedef union {
    struct {
        unsigned SLRA0 :1;
        unsigned SLRA1 :1;
        unsigned SLRA2 :1;
        unsigned :1;
        unsigned SLRA4 :1;
        unsigned SLRA5 :1;
    };
} SLRCONAbits_t;
extern volatile SLRCONAbits_t SLRCONAbits __attribute__((address(0x1E8F)));
# 18016 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char INLVLA __attribute__((address(0x1E90)));

__asm("INLVLA equ 01E90h");


typedef union {
    struct {
        unsigned INLVLA0 :1;
        unsigned INLVLA1 :1;
        unsigned INLVLA2 :1;
        unsigned :1;
        unsigned INLVLA4 :1;
        unsigned INLVLA5 :1;
    };
} INLVLAbits_t;
extern volatile INLVLAbits_t INLVLAbits __attribute__((address(0x1E90)));
# 18061 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char IOCAP __attribute__((address(0x1E91)));

__asm("IOCAP equ 01E91h");


typedef union {
    struct {
        unsigned IOCAP0 :1;
        unsigned IOCAP1 :1;
        unsigned IOCAP2 :1;
        unsigned IOCAP3 :1;
        unsigned IOCAP4 :1;
        unsigned IOCAP5 :1;
    };
} IOCAPbits_t;
extern volatile IOCAPbits_t IOCAPbits __attribute__((address(0x1E91)));
# 18111 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char IOCAN __attribute__((address(0x1E92)));

__asm("IOCAN equ 01E92h");


typedef union {
    struct {
        unsigned IOCAN0 :1;
        unsigned IOCAN1 :1;
        unsigned IOCAN2 :1;
        unsigned IOCAN3 :1;
        unsigned IOCAN4 :1;
        unsigned IOCAN5 :1;
    };
} IOCANbits_t;
extern volatile IOCANbits_t IOCANbits __attribute__((address(0x1E92)));
# 18161 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char IOCAF __attribute__((address(0x1E93)));

__asm("IOCAF equ 01E93h");


typedef union {
    struct {
        unsigned IOCAF0 :1;
        unsigned IOCAF1 :1;
        unsigned IOCAF2 :1;
        unsigned IOCAF3 :1;
        unsigned IOCAF4 :1;
        unsigned IOCAF5 :1;
    };
} IOCAFbits_t;
extern volatile IOCAFbits_t IOCAFbits __attribute__((address(0x1E93)));
# 18211 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char RA1I2C __attribute__((address(0x1EE1)));

__asm("RA1I2C equ 01EE1h");


typedef union {
    struct {
        unsigned TH :2;
        unsigned :2;
        unsigned PU :2;
        unsigned SLEW :2;
    };
} RA1I2Cbits_t;
extern volatile RA1I2Cbits_t RA1I2Cbits __attribute__((address(0x1EE1)));
# 18244 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char RA2I2C __attribute__((address(0x1EE2)));

__asm("RA2I2C equ 01EE2h");


typedef union {
    struct {
        unsigned TH :2;
        unsigned :2;
        unsigned PU :2;
        unsigned SLEW :2;
    };
} RA2I2Cbits_t;
extern volatile RA2I2Cbits_t RA2I2Cbits __attribute__((address(0x1EE2)));
# 18277 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char UMTOAP __attribute__((address(0x1F0C)));

__asm("UMTOAP equ 01F0Ch");


typedef union {
    struct {
        unsigned UMTOAP :8;
    };
} UMTOAPbits_t;
extern volatile UMTOAPbits_t UMTOAPbits __attribute__((address(0x1F0C)));
# 18297 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char UMTOAL __attribute__((address(0x1F0D)));

__asm("UMTOAL equ 01F0Dh");


typedef union {
    struct {
        unsigned UMTOAL :8;
    };
} UMTOALbits_t;
extern volatile UMTOALbits_t UMTOALbits __attribute__((address(0x1F0D)));
# 18317 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char UMTOAH __attribute__((address(0x1F0E)));

__asm("UMTOAH equ 01F0Eh");


typedef union {
    struct {
        unsigned UMTOAH :8;
    };
} UMTOAHbits_t;
extern volatile UMTOAHbits_t UMTOAHbits __attribute__((address(0x1F0E)));
# 18337 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char STATUS_SHAD __attribute__((address(0x1FE4)));

__asm("STATUS_SHAD equ 01FE4h");


typedef union {
    struct {
        unsigned STATUS_SHAD :3;
    };
    struct {
        unsigned STATUS_SHAD0 :1;
        unsigned STATUS_SHAD1 :1;
        unsigned STATUS_SHAD2 :1;
    };
} STATUS_SHADbits_t;
extern volatile STATUS_SHADbits_t STATUS_SHADbits __attribute__((address(0x1FE4)));
# 18377 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char WREG_SHAD __attribute__((address(0x1FE5)));

__asm("WREG_SHAD equ 01FE5h");


typedef union {
    struct {
        unsigned WREG_SHAD :8;
    };
} WREG_SHADbits_t;
extern volatile WREG_SHADbits_t WREG_SHADbits __attribute__((address(0x1FE5)));
# 18397 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char BSR_SHAD __attribute__((address(0x1FE6)));

__asm("BSR_SHAD equ 01FE6h");


typedef union {
    struct {
        unsigned BSR_SHAD :8;
    };
} BSR_SHADbits_t;
extern volatile BSR_SHADbits_t BSR_SHADbits __attribute__((address(0x1FE6)));
# 18417 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char PCLATH_SHAD __attribute__((address(0x1FE7)));

__asm("PCLATH_SHAD equ 01FE7h");


typedef union {
    struct {
        unsigned PCLATH_SHAD :8;
    };
} PCLATH_SHADbits_t;
extern volatile PCLATH_SHADbits_t PCLATH_SHADbits __attribute__((address(0x1FE7)));
# 18437 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned short FSR0_SHAD __attribute__((address(0x1FE8)));

__asm("FSR0_SHAD equ 01FE8h");




extern volatile unsigned char FSR0L_SHAD __attribute__((address(0x1FE8)));

__asm("FSR0L_SHAD equ 01FE8h");


typedef union {
    struct {
        unsigned FSR0L_SHAD :8;
    };
} FSR0L_SHADbits_t;
extern volatile FSR0L_SHADbits_t FSR0L_SHADbits __attribute__((address(0x1FE8)));
# 18464 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char FSR0H_SHAD __attribute__((address(0x1FE9)));

__asm("FSR0H_SHAD equ 01FE9h");


typedef union {
    struct {
        unsigned FSR0H_SHAD :8;
    };
} FSR0H_SHADbits_t;
extern volatile FSR0H_SHADbits_t FSR0H_SHADbits __attribute__((address(0x1FE9)));
# 18484 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned short FSR1_SHAD __attribute__((address(0x1FEA)));

__asm("FSR1_SHAD equ 01FEAh");




extern volatile unsigned char FSR1L_SHAD __attribute__((address(0x1FEA)));

__asm("FSR1L_SHAD equ 01FEAh");


typedef union {
    struct {
        unsigned FSR1L_SHAD :8;
    };
} FSR1L_SHADbits_t;
extern volatile FSR1L_SHADbits_t FSR1L_SHADbits __attribute__((address(0x1FEA)));
# 18511 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char FSR1H_SHAD __attribute__((address(0x1FEB)));

__asm("FSR1H_SHAD equ 01FEBh");


typedef union {
    struct {
        unsigned FSR1H_SHAD :8;
    };
} FSR1H_SHADbits_t;
extern volatile FSR1H_SHADbits_t FSR1H_SHADbits __attribute__((address(0x1FEB)));
# 18531 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char STKPTR __attribute__((address(0x1FED)));

__asm("STKPTR equ 01FEDh");


typedef union {
    struct {
        unsigned STKPTR0 :1;
        unsigned STKPTR1 :1;
        unsigned STKPTR2 :1;
        unsigned STKPTR3 :1;
        unsigned STKPTR4 :1;
    };
} STKPTRbits_t;
extern volatile STKPTRbits_t STKPTRbits __attribute__((address(0x1FED)));
# 18575 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char TOSL __attribute__((address(0x1FEE)));

__asm("TOSL equ 01FEEh");


typedef union {
    struct {
        unsigned TOSL :8;
    };
    struct {
        unsigned TOSL0 :1;
        unsigned TOSL1 :1;
        unsigned TOSL2 :1;
        unsigned TOSL3 :1;
        unsigned TOSL4 :1;
        unsigned TOSL5 :1;
        unsigned TOSL6 :1;
        unsigned TOSL7 :1;
    };
} TOSLbits_t;
extern volatile TOSLbits_t TOSLbits __attribute__((address(0x1FEE)));
# 18645 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile unsigned char TOSH __attribute__((address(0x1FEF)));

__asm("TOSH equ 01FEFh");


typedef union {
    struct {
        unsigned TOSH :8;
    };
    struct {
        unsigned TOSH0 :1;
        unsigned TOSH1 :1;
        unsigned TOSH2 :1;
        unsigned TOSH3 :1;
        unsigned TOSH4 :1;
        unsigned TOSH5 :1;
        unsigned TOSH6 :1;
        unsigned TOSH7 :1;
    };
} TOSHbits_t;
extern volatile TOSHbits_t TOSHbits __attribute__((address(0x1FEF)));
# 18725 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\proc\\pic16f18015.h" 3
extern volatile __bit ACC18 __attribute__((address(0x2472)));


extern volatile __bit ACC19 __attribute__((address(0x2473)));


extern volatile __bit ACKDT1 __attribute__((address(0x348D)));


extern volatile __bit ACKDT2 __attribute__((address(0x34DD)));


extern volatile __bit ACKEN1 __attribute__((address(0x348C)));


extern volatile __bit ACKEN2 __attribute__((address(0x34DC)));


extern volatile __bit ACKSTAT1 __attribute__((address(0x348E)));


extern volatile __bit ACKSTAT2 __attribute__((address(0x34DE)));


extern volatile __bit ACLR __attribute__((address(0xE943)));


extern volatile __bit ACQ0 __attribute__((address(0xE908)));


extern volatile __bit ACQ1 __attribute__((address(0xE909)));


extern volatile __bit ACQ10 __attribute__((address(0xE912)));


extern volatile __bit ACQ11 __attribute__((address(0xE913)));


extern volatile __bit ACQ12 __attribute__((address(0xE914)));


extern volatile __bit ACQ2 __attribute__((address(0xE90A)));


extern volatile __bit ACQ3 __attribute__((address(0xE90B)));


extern volatile __bit ACQ4 __attribute__((address(0xE90C)));


extern volatile __bit ACQ5 __attribute__((address(0xE90D)));


extern volatile __bit ACQ6 __attribute__((address(0xE90E)));


extern volatile __bit ACQ7 __attribute__((address(0xE90F)));


extern volatile __bit ACQ8 __attribute__((address(0xE910)));


extern volatile __bit ACQ9 __attribute__((address(0xE911)));


extern volatile __bit ACT0 __attribute__((address(0xE960)));


extern volatile __bit ACT1 __attribute__((address(0xE961)));


extern volatile __bit ACT2 __attribute__((address(0xE962)));


extern volatile __bit ACT3 __attribute__((address(0xE963)));


extern volatile __bit ACT4 __attribute__((address(0xE964)));


extern volatile __bit ACT5 __attribute__((address(0xE965)));


extern volatile __bit ACTEN __attribute__((address(0x10A7)));


extern volatile __bit ACTIE __attribute__((address(0x4BA)));


extern volatile __bit ACTIF __attribute__((address(0x46A)));


extern volatile __bit ACTLOCK __attribute__((address(0x10A3)));


extern volatile __bit ACTORS __attribute__((address(0x10A1)));


extern volatile __bit ACTUD __attribute__((address(0x10A6)));


extern volatile __bit ADACC0 __attribute__((address(0xE8B0)));


extern volatile __bit ADACC1 __attribute__((address(0xE8B1)));


extern volatile __bit ADACC10 __attribute__((address(0xE8BA)));


extern volatile __bit ADACC11 __attribute__((address(0xE8BB)));


extern volatile __bit ADACC12 __attribute__((address(0xE8BC)));


extern volatile __bit ADACC13 __attribute__((address(0xE8BD)));


extern volatile __bit ADACC14 __attribute__((address(0xE8BE)));


extern volatile __bit ADACC15 __attribute__((address(0xE8BF)));


extern volatile __bit ADACC16 __attribute__((address(0xE8C0)));


extern volatile __bit ADACC17 __attribute__((address(0xE8C1)));


extern volatile __bit ADACC2 __attribute__((address(0xE8B2)));


extern volatile __bit ADACC3 __attribute__((address(0xE8B3)));


extern volatile __bit ADACC4 __attribute__((address(0xE8B4)));


extern volatile __bit ADACC5 __attribute__((address(0xE8B5)));


extern volatile __bit ADACC6 __attribute__((address(0xE8B6)));


extern volatile __bit ADACC7 __attribute__((address(0xE8B7)));


extern volatile __bit ADACC8 __attribute__((address(0xE8B8)));


extern volatile __bit ADACC9 __attribute__((address(0xE8B9)));


extern volatile __bit ADACLR __attribute__((address(0xE943)));


extern volatile __bit ADACQ0 __attribute__((address(0xE908)));


extern volatile __bit ADACQ1 __attribute__((address(0xE909)));


extern volatile __bit ADACQ10 __attribute__((address(0xE912)));


extern volatile __bit ADACQ11 __attribute__((address(0xE913)));


extern volatile __bit ADACQ12 __attribute__((address(0xE914)));


extern volatile __bit ADACQ2 __attribute__((address(0xE90A)));


extern volatile __bit ADACQ3 __attribute__((address(0xE90B)));


extern volatile __bit ADACQ4 __attribute__((address(0xE90C)));


extern volatile __bit ADACQ5 __attribute__((address(0xE90D)));


extern volatile __bit ADACQ6 __attribute__((address(0xE90E)));


extern volatile __bit ADACQ7 __attribute__((address(0xE90F)));


extern volatile __bit ADACQ8 __attribute__((address(0xE910)));


extern volatile __bit ADACQ9 __attribute__((address(0xE911)));


extern volatile __bit ADACT0 __attribute__((address(0xE960)));


extern volatile __bit ADACT1 __attribute__((address(0xE961)));


extern volatile __bit ADACT2 __attribute__((address(0xE962)));


extern volatile __bit ADACT3 __attribute__((address(0xE963)));


extern volatile __bit ADACT4 __attribute__((address(0xE964)));


extern volatile __bit ADACT5 __attribute__((address(0xE965)));


extern volatile __bit ADAOV __attribute__((address(0xE957)));


extern volatile __bit ADCALC0 __attribute__((address(0xE94C)));


extern volatile __bit ADCALC1 __attribute__((address(0xE94D)));


extern volatile __bit ADCALC2 __attribute__((address(0xE94E)));


extern volatile __bit ADCAP0 __attribute__((address(0xE918)));


extern volatile __bit ADCAP1 __attribute__((address(0xE919)));


extern volatile __bit ADCAP2 __attribute__((address(0xE91A)));


extern volatile __bit ADCAP3 __attribute__((address(0xE91B)));


extern volatile __bit ADCAP4 __attribute__((address(0xE91C)));


extern volatile __bit ADCNT0 __attribute__((address(0xE8C8)));


extern volatile __bit ADCNT1 __attribute__((address(0xE8C9)));


extern volatile __bit ADCNT2 __attribute__((address(0xE8CA)));


extern volatile __bit ADCNT3 __attribute__((address(0xE8CB)));


extern volatile __bit ADCNT4 __attribute__((address(0xE8CC)));


extern volatile __bit ADCNT5 __attribute__((address(0xE8CD)));


extern volatile __bit ADCNT6 __attribute__((address(0xE8CE)));


extern volatile __bit ADCNT7 __attribute__((address(0xE8CF)));


extern volatile __bit ADCONT __attribute__((address(0xE936)));


extern volatile __bit ADCRS0 __attribute__((address(0xE944)));


extern volatile __bit ADCRS1 __attribute__((address(0xE945)));


extern volatile __bit ADCRS2 __attribute__((address(0xE946)));


extern volatile __bit ADCS __attribute__((address(0xE934)));


extern volatile __bit ADCS0 __attribute__((address(0xE968)));


extern volatile __bit ADCS1 __attribute__((address(0xE969)));


extern volatile __bit ADCS2 __attribute__((address(0xE96A)));


extern volatile __bit ADCS3 __attribute__((address(0xE96B)));


extern volatile __bit ADCS4 __attribute__((address(0xE96C)));


extern volatile __bit ADCS5 __attribute__((address(0xE96D)));


extern volatile __bit ADDSEN __attribute__((address(0xE938)));


extern volatile __bit ADERR0 __attribute__((address(0xE880)));


extern volatile __bit ADERR1 __attribute__((address(0xE881)));


extern volatile __bit ADERR10 __attribute__((address(0xE88A)));


extern volatile __bit ADERR11 __attribute__((address(0xE88B)));


extern volatile __bit ADERR12 __attribute__((address(0xE88C)));


extern volatile __bit ADERR13 __attribute__((address(0xE88D)));


extern volatile __bit ADERR14 __attribute__((address(0xE88E)));


extern volatile __bit ADERR15 __attribute__((address(0xE88F)));


extern volatile __bit ADERR2 __attribute__((address(0xE882)));


extern volatile __bit ADERR3 __attribute__((address(0xE883)));


extern volatile __bit ADERR4 __attribute__((address(0xE884)));


extern volatile __bit ADERR5 __attribute__((address(0xE885)));


extern volatile __bit ADERR6 __attribute__((address(0xE886)));


extern volatile __bit ADERR7 __attribute__((address(0xE887)));


extern volatile __bit ADERR8 __attribute__((address(0xE888)));


extern volatile __bit ADERR9 __attribute__((address(0xE889)));


extern volatile __bit ADFLTR0 __attribute__((address(0xE8A0)));


extern volatile __bit ADFLTR1 __attribute__((address(0xE8A1)));


extern volatile __bit ADFLTR10 __attribute__((address(0xE8AA)));


extern volatile __bit ADFLTR11 __attribute__((address(0xE8AB)));


extern volatile __bit ADFLTR12 __attribute__((address(0xE8AC)));


extern volatile __bit ADFLTR13 __attribute__((address(0xE8AD)));


extern volatile __bit ADFLTR14 __attribute__((address(0xE8AE)));


extern volatile __bit ADFLTR15 __attribute__((address(0xE8AF)));


extern volatile __bit ADFLTR2 __attribute__((address(0xE8A2)));


extern volatile __bit ADFLTR3 __attribute__((address(0xE8A3)));


extern volatile __bit ADFLTR4 __attribute__((address(0xE8A4)));


extern volatile __bit ADFLTR5 __attribute__((address(0xE8A5)));


extern volatile __bit ADFLTR6 __attribute__((address(0xE8A6)));


extern volatile __bit ADFLTR7 __attribute__((address(0xE8A7)));


extern volatile __bit ADFLTR8 __attribute__((address(0xE8A8)));


extern volatile __bit ADFLTR9 __attribute__((address(0xE8A9)));


extern volatile __bit ADFM0 __attribute__((address(0xE932)));


extern volatile __bit ADFVR0 __attribute__((address(0x8060)));


extern volatile __bit ADFVR1 __attribute__((address(0x8061)));


extern volatile __bit ADGO __attribute__((address(0xE930)));


extern volatile __bit ADGPOL __attribute__((address(0xE93D)));


extern volatile __bit ADIE __attribute__((address(0x4D3)));


extern volatile __bit ADIF __attribute__((address(0x483)));


extern volatile __bit ADIPEN __attribute__((address(0xE93E)));


extern volatile __bit ADLTH0 __attribute__((address(0xE860)));


extern volatile __bit ADLTH1 __attribute__((address(0xE861)));


extern volatile __bit ADLTH10 __attribute__((address(0xE86A)));


extern volatile __bit ADLTH11 __attribute__((address(0xE86B)));


extern volatile __bit ADLTH12 __attribute__((address(0xE86C)));


extern volatile __bit ADLTH13 __attribute__((address(0xE86D)));


extern volatile __bit ADLTH14 __attribute__((address(0xE86E)));


extern volatile __bit ADLTH15 __attribute__((address(0xE86F)));


extern volatile __bit ADLTH2 __attribute__((address(0xE862)));


extern volatile __bit ADLTH3 __attribute__((address(0xE863)));


extern volatile __bit ADLTH4 __attribute__((address(0xE864)));


extern volatile __bit ADLTH5 __attribute__((address(0xE865)));


extern volatile __bit ADLTH6 __attribute__((address(0xE866)));


extern volatile __bit ADLTH7 __attribute__((address(0xE867)));


extern volatile __bit ADLTH8 __attribute__((address(0xE868)));


extern volatile __bit ADLTH9 __attribute__((address(0xE869)));


extern volatile __bit ADLTHR __attribute__((address(0xE955)));


extern volatile __bit ADMATH __attribute__((address(0xE954)));


extern volatile __bit ADMD0 __attribute__((address(0xE940)));


extern volatile __bit ADMD1 __attribute__((address(0xE941)));


extern volatile __bit ADMD2 __attribute__((address(0xE942)));


extern volatile __bit ADMSK11 __attribute__((address(0x3489)));


extern volatile __bit ADMSK12 __attribute__((address(0x34D9)));


extern volatile __bit ADMSK21 __attribute__((address(0x348A)));


extern volatile __bit ADMSK22 __attribute__((address(0x34DA)));


extern volatile __bit ADMSK31 __attribute__((address(0x348B)));


extern volatile __bit ADMSK32 __attribute__((address(0x34DB)));


extern volatile __bit ADMSK41 __attribute__((address(0x348C)));


extern volatile __bit ADMSK42 __attribute__((address(0x34DC)));


extern volatile __bit ADMSK51 __attribute__((address(0x348D)));


extern volatile __bit ADMSK52 __attribute__((address(0x34DD)));


extern volatile __bit ADOEN __attribute__((address(0x108A)));


extern volatile __bit ADON __attribute__((address(0xE937)));


extern volatile __bit ADOR __attribute__((address(0x1082)));


extern volatile __bit ADOV __attribute__((address(0xE957)));


extern volatile __bit ADPCH0 __attribute__((address(0xE8F8)));


extern volatile __bit ADPCH1 __attribute__((address(0xE8F9)));


extern volatile __bit ADPCH2 __attribute__((address(0xE8FA)));


extern volatile __bit ADPCH3 __attribute__((address(0xE8FB)));


extern volatile __bit ADPCH4 __attribute__((address(0xE8FC)));


extern volatile __bit ADPCH5 __attribute__((address(0xE8FD)));


extern volatile __bit ADPPOL __attribute__((address(0xE93F)));


extern volatile __bit ADPRE0 __attribute__((address(0xE920)));


extern volatile __bit ADPRE1 __attribute__((address(0xE921)));


extern volatile __bit ADPRE10 __attribute__((address(0xE92A)));


extern volatile __bit ADPRE11 __attribute__((address(0xE92B)));


extern volatile __bit ADPRE12 __attribute__((address(0xE92C)));


extern volatile __bit ADPRE2 __attribute__((address(0xE922)));


extern volatile __bit ADPRE3 __attribute__((address(0xE923)));


extern volatile __bit ADPRE4 __attribute__((address(0xE924)));


extern volatile __bit ADPRE5 __attribute__((address(0xE925)));


extern volatile __bit ADPRE6 __attribute__((address(0xE926)));


extern volatile __bit ADPRE7 __attribute__((address(0xE927)));


extern volatile __bit ADPRE8 __attribute__((address(0xE928)));


extern volatile __bit ADPRE9 __attribute__((address(0xE929)));


extern volatile __bit ADPREF0 __attribute__((address(0xE958)));


extern volatile __bit ADPREF1 __attribute__((address(0xE959)));


extern volatile __bit ADPREV0 __attribute__((address(0xE8D8)));


extern volatile __bit ADPREV1 __attribute__((address(0xE8D9)));


extern volatile __bit ADPREV10 __attribute__((address(0xE8E2)));


extern volatile __bit ADPREV11 __attribute__((address(0xE8E3)));


extern volatile __bit ADPREV12 __attribute__((address(0xE8E4)));


extern volatile __bit ADPREV13 __attribute__((address(0xE8E5)));


extern volatile __bit ADPREV14 __attribute__((address(0xE8E6)));


extern volatile __bit ADPREV15 __attribute__((address(0xE8E7)));


extern volatile __bit ADPREV2 __attribute__((address(0xE8DA)));


extern volatile __bit ADPREV3 __attribute__((address(0xE8DB)));


extern volatile __bit ADPREV4 __attribute__((address(0xE8DC)));


extern volatile __bit ADPREV5 __attribute__((address(0xE8DD)));


extern volatile __bit ADPREV6 __attribute__((address(0xE8DE)));


extern volatile __bit ADPREV7 __attribute__((address(0xE8DF)));


extern volatile __bit ADPREV8 __attribute__((address(0xE8E0)));


extern volatile __bit ADPREV9 __attribute__((address(0xE8E1)));


extern volatile __bit ADPSIS __attribute__((address(0xE947)));


extern volatile __bit ADRES0 __attribute__((address(0xE8E8)));


extern volatile __bit ADRES1 __attribute__((address(0xE8E9)));


extern volatile __bit ADRES10 __attribute__((address(0xE8F2)));


extern volatile __bit ADRES11 __attribute__((address(0xE8F3)));


extern volatile __bit ADRES12 __attribute__((address(0xE8F4)));


extern volatile __bit ADRES13 __attribute__((address(0xE8F5)));


extern volatile __bit ADRES14 __attribute__((address(0xE8F6)));


extern volatile __bit ADRES15 __attribute__((address(0xE8F7)));


extern volatile __bit ADRES2 __attribute__((address(0xE8EA)));


extern volatile __bit ADRES3 __attribute__((address(0xE8EB)));


extern volatile __bit ADRES4 __attribute__((address(0xE8EC)));


extern volatile __bit ADRES5 __attribute__((address(0xE8ED)));


extern volatile __bit ADRES6 __attribute__((address(0xE8EE)));


extern volatile __bit ADRES7 __attribute__((address(0xE8EF)));


extern volatile __bit ADRES8 __attribute__((address(0xE8F0)));


extern volatile __bit ADRES9 __attribute__((address(0xE8F1)));


extern volatile __bit ADRPT0 __attribute__((address(0xE8D0)));


extern volatile __bit ADRPT1 __attribute__((address(0xE8D1)));


extern volatile __bit ADRPT2 __attribute__((address(0xE8D2)));


extern volatile __bit ADRPT3 __attribute__((address(0xE8D3)));


extern volatile __bit ADRPT4 __attribute__((address(0xE8D4)));


extern volatile __bit ADRPT5 __attribute__((address(0xE8D5)));


extern volatile __bit ADRPT6 __attribute__((address(0xE8D6)));


extern volatile __bit ADRPT7 __attribute__((address(0xE8D7)));


extern volatile __bit ADSOI __attribute__((address(0xE94B)));


extern volatile __bit ADSTAT0 __attribute__((address(0xE950)));


extern volatile __bit ADSTAT1 __attribute__((address(0xE951)));


extern volatile __bit ADSTAT2 __attribute__((address(0xE952)));


extern volatile __bit ADSTPT0 __attribute__((address(0xE890)));


extern volatile __bit ADSTPT1 __attribute__((address(0xE891)));


extern volatile __bit ADSTPT10 __attribute__((address(0xE89A)));


extern volatile __bit ADSTPT11 __attribute__((address(0xE89B)));


extern volatile __bit ADSTPT12 __attribute__((address(0xE89C)));


extern volatile __bit ADSTPT13 __attribute__((address(0xE89D)));


extern volatile __bit ADSTPT14 __attribute__((address(0xE89E)));


extern volatile __bit ADSTPT15 __attribute__((address(0xE89F)));


extern volatile __bit ADSTPT2 __attribute__((address(0xE892)));


extern volatile __bit ADSTPT3 __attribute__((address(0xE893)));


extern volatile __bit ADSTPT4 __attribute__((address(0xE894)));


extern volatile __bit ADSTPT5 __attribute__((address(0xE895)));


extern volatile __bit ADSTPT6 __attribute__((address(0xE896)));


extern volatile __bit ADSTPT7 __attribute__((address(0xE897)));


extern volatile __bit ADSTPT8 __attribute__((address(0xE898)));


extern volatile __bit ADSTPT9 __attribute__((address(0xE899)));


extern volatile __bit ADTIE __attribute__((address(0x4D4)));


extern volatile __bit ADTIF __attribute__((address(0x484)));


extern volatile __bit ADTMD0 __attribute__((address(0xE948)));


extern volatile __bit ADTMD1 __attribute__((address(0xE949)));


extern volatile __bit ADTMD2 __attribute__((address(0xE94A)));


extern volatile __bit ADUTH0 __attribute__((address(0xE870)));


extern volatile __bit ADUTH1 __attribute__((address(0xE871)));


extern volatile __bit ADUTH10 __attribute__((address(0xE87A)));


extern volatile __bit ADUTH11 __attribute__((address(0xE87B)));


extern volatile __bit ADUTH12 __attribute__((address(0xE87C)));


extern volatile __bit ADUTH13 __attribute__((address(0xE87D)));


extern volatile __bit ADUTH14 __attribute__((address(0xE87E)));


extern volatile __bit ADUTH15 __attribute__((address(0xE87F)));


extern volatile __bit ADUTH2 __attribute__((address(0xE872)));


extern volatile __bit ADUTH3 __attribute__((address(0xE873)));


extern volatile __bit ADUTH4 __attribute__((address(0xE874)));


extern volatile __bit ADUTH5 __attribute__((address(0xE875)));


extern volatile __bit ADUTH6 __attribute__((address(0xE876)));


extern volatile __bit ADUTH7 __attribute__((address(0xE877)));


extern volatile __bit ADUTH8 __attribute__((address(0xE878)));


extern volatile __bit ADUTH9 __attribute__((address(0xE879)));


extern volatile __bit ADUTHR __attribute__((address(0xE956)));


extern volatile __bit ANSA0 __attribute__((address(0xF460)));


extern volatile __bit ANSA1 __attribute__((address(0xF461)));


extern volatile __bit ANSA2 __attribute__((address(0xF462)));


extern volatile __bit ANSA4 __attribute__((address(0xF464)));


extern volatile __bit ANSA5 __attribute__((address(0xF465)));


extern volatile __bit ANSELA0 __attribute__((address(0xF460)));


extern volatile __bit ANSELA1 __attribute__((address(0xF461)));


extern volatile __bit ANSELA2 __attribute__((address(0xF462)));


extern volatile __bit ANSELA4 __attribute__((address(0xF464)));


extern volatile __bit ANSELA5 __attribute__((address(0xF465)));


extern volatile __bit AOV __attribute__((address(0xE957)));


extern volatile __bit AS0E __attribute__((address(0x2898)));


extern volatile __bit AS1E __attribute__((address(0x2899)));


extern volatile __bit AS2E __attribute__((address(0x289A)));


extern volatile __bit AS3E __attribute__((address(0x289B)));


extern volatile __bit AS4E __attribute__((address(0x289C)));


extern volatile __bit AS5E __attribute__((address(0x289D)));


extern volatile __bit AS6E __attribute__((address(0x289E)));


extern volatile __bit AS7E __attribute__((address(0x289F)));


extern volatile __bit ASYNC __attribute__((address(0xCFC)));


extern volatile __bit BCL1IE __attribute__((address(0x4CF)));


extern volatile __bit BCL1IF __attribute__((address(0x47F)));


extern volatile __bit BCL2IE __attribute__((address(0x4D1)));


extern volatile __bit BCL2IF __attribute__((address(0x481)));


extern volatile __bit BF1 __attribute__((address(0x3478)));


extern volatile __bit BF2 __attribute__((address(0x34C8)));


extern volatile __bit BORRDY __attribute__((address(0xC68)));


extern volatile __bit BSR0 __attribute__((address(0x40)));


extern volatile __bit BSR1 __attribute__((address(0x41)));


extern volatile __bit BSR2 __attribute__((address(0x42)));


extern volatile __bit BSR3 __attribute__((address(0x43)));


extern volatile __bit BSR4 __attribute__((address(0x44)));


extern volatile __bit BSR5 __attribute__((address(0x45)));


extern volatile __bit C1EN __attribute__((address(0x8467)));


extern volatile __bit C1HYS __attribute__((address(0x8461)));


extern volatile __bit C1INTN __attribute__((address(0x8468)));


extern volatile __bit C1INTP __attribute__((address(0x8469)));


extern volatile __bit C1NCH0 __attribute__((address(0x8470)));


extern volatile __bit C1NCH1 __attribute__((address(0x8471)));


extern volatile __bit C1NCH2 __attribute__((address(0x8472)));


extern volatile __bit C1PCH0 __attribute__((address(0x8478)));


extern volatile __bit C1PCH1 __attribute__((address(0x8479)));


extern volatile __bit C1PCH2 __attribute__((address(0x847A)));


extern volatile __bit C1POL __attribute__((address(0x8464)));


extern volatile __bit C1SYNC __attribute__((address(0x8460)));


extern volatile __bit CAL01 __attribute__((address(0x1460)));


extern volatile __bit CAL03 __attribute__((address(0x1490)));


extern volatile __bit CAL11 __attribute__((address(0x1461)));


extern volatile __bit CAL13 __attribute__((address(0x1491)));


extern volatile __bit CAL21 __attribute__((address(0x1462)));


extern volatile __bit CAL23 __attribute__((address(0x1492)));


extern volatile __bit CAL31 __attribute__((address(0x1463)));


extern volatile __bit CAL33 __attribute__((address(0x1493)));


extern volatile __bit CAL41 __attribute__((address(0x1464)));


extern volatile __bit CAL43 __attribute__((address(0x1494)));


extern volatile __bit CAL51 __attribute__((address(0x1465)));


extern volatile __bit CAL53 __attribute__((address(0x1495)));


extern volatile __bit CAL61 __attribute__((address(0x1466)));


extern volatile __bit CAL63 __attribute__((address(0x1496)));


extern volatile __bit CAL71 __attribute__((address(0x1467)));


extern volatile __bit CAL73 __attribute__((address(0x1497)));


extern volatile __bit CALC0 __attribute__((address(0xE94C)));


extern volatile __bit CALC1 __attribute__((address(0xE94D)));


extern volatile __bit CALC2 __attribute__((address(0xE94E)));


extern volatile __bit CARRY __attribute__((address(0x18)));


extern volatile __bit CCP1CTS0 __attribute__((address(0x1C78)));


extern volatile __bit CCP1CTS1 __attribute__((address(0x1C79)));


extern volatile __bit CCP1CTS2 __attribute__((address(0x1C7A)));


extern volatile __bit CCP1CTS3 __attribute__((address(0x1C7B)));


extern volatile __bit CCP1EN __attribute__((address(0x1C77)));


extern volatile __bit CCP1FMT __attribute__((address(0x1C74)));


extern volatile __bit CCP1IE __attribute__((address(0x4C2)));


extern volatile __bit CCP1IF __attribute__((address(0x472)));


extern volatile __bit CCP1MODE0 __attribute__((address(0x1C70)));


extern volatile __bit CCP1MODE1 __attribute__((address(0x1C71)));


extern volatile __bit CCP1MODE2 __attribute__((address(0x1C72)));


extern volatile __bit CCP1MODE3 __attribute__((address(0x1C73)));


extern volatile __bit CCP1OUT __attribute__((address(0x1C75)));


extern volatile __bit CCP1PPS0 __attribute__((address(0xF0F0)));


extern volatile __bit CCP1PPS1 __attribute__((address(0xF0F1)));


extern volatile __bit CCP1PPS2 __attribute__((address(0xF0F2)));


extern volatile __bit CCP1PPS3 __attribute__((address(0xF0F3)));


extern volatile __bit CCP1PPS4 __attribute__((address(0xF0F4)));


extern volatile __bit CCP2CTS0 __attribute__((address(0x1C98)));


extern volatile __bit CCP2CTS1 __attribute__((address(0x1C99)));


extern volatile __bit CCP2CTS2 __attribute__((address(0x1C9A)));


extern volatile __bit CCP2CTS3 __attribute__((address(0x1C9B)));


extern volatile __bit CCP2EN __attribute__((address(0x1C97)));


extern volatile __bit CCP2FMT __attribute__((address(0x1C94)));


extern volatile __bit CCP2IE __attribute__((address(0x4C3)));


extern volatile __bit CCP2IF __attribute__((address(0x473)));


extern volatile __bit CCP2MODE0 __attribute__((address(0x1C90)));


extern volatile __bit CCP2MODE1 __attribute__((address(0x1C91)));


extern volatile __bit CCP2MODE2 __attribute__((address(0x1C92)));


extern volatile __bit CCP2MODE3 __attribute__((address(0x1C93)));


extern volatile __bit CCP2OUT __attribute__((address(0x1C95)));


extern volatile __bit CCP2PPS0 __attribute__((address(0xF0F8)));


extern volatile __bit CCP2PPS1 __attribute__((address(0xF0F9)));


extern volatile __bit CCP2PPS2 __attribute__((address(0xF0FA)));


extern volatile __bit CCP2PPS3 __attribute__((address(0xF0FB)));


extern volatile __bit CCP2PPS4 __attribute__((address(0xF0FC)));


extern volatile __bit CDAFVR0 __attribute__((address(0x8062)));


extern volatile __bit CDAFVR1 __attribute__((address(0x8063)));


extern volatile __bit CGA0 __attribute__((address(0xE970)));


extern volatile __bit CGA1 __attribute__((address(0xE971)));


extern volatile __bit CGA2 __attribute__((address(0xE972)));


extern volatile __bit CGA4 __attribute__((address(0xE974)));


extern volatile __bit CGA5 __attribute__((address(0xE975)));


extern volatile __bit CKE1 __attribute__((address(0x347E)));


extern volatile __bit CKE2 __attribute__((address(0x34CE)));


extern volatile __bit CKP1 __attribute__((address(0x3484)));


extern volatile __bit CKP2 __attribute__((address(0x34D4)));


extern volatile __bit CKPS3 __attribute__((address(0xCFB)));


extern volatile __bit CKS0 __attribute__((address(0x2498)));


extern volatile __bit CKS1 __attribute__((address(0x2499)));


extern volatile __bit CKS2 __attribute__((address(0x249A)));


extern volatile __bit CKS3 __attribute__((address(0x249B)));


extern volatile __bit CLC1IE __attribute__((address(0x4C6)));


extern volatile __bit CLC1IF __attribute__((address(0x476)));


extern volatile __bit CLC1OUT __attribute__((address(0x2CB8)));


extern volatile __bit CLC2IE __attribute__((address(0x4C7)));


extern volatile __bit CLC2IF __attribute__((address(0x477)));


extern volatile __bit CLC2OUT __attribute__((address(0x2CB9)));


extern volatile __bit CLC3IE __attribute__((address(0x4C8)));


extern volatile __bit CLC3IF __attribute__((address(0x478)));


extern volatile __bit CLC3OUT __attribute__((address(0x2CBA)));


extern volatile __bit CLC4IE __attribute__((address(0x4C9)));


extern volatile __bit CLC4IF __attribute__((address(0x479)));


extern volatile __bit CLC4OUT __attribute__((address(0x2CBB)));


extern volatile __bit CLCIN0PPS0 __attribute__((address(0xF1E8)));


extern volatile __bit CLCIN0PPS1 __attribute__((address(0xF1E9)));


extern volatile __bit CLCIN0PPS2 __attribute__((address(0xF1EA)));


extern volatile __bit CLCIN0PPS3 __attribute__((address(0xF1EB)));


extern volatile __bit CLCIN0PPS4 __attribute__((address(0xF1EC)));


extern volatile __bit CLCIN1PPS0 __attribute__((address(0xF1F0)));


extern volatile __bit CLCIN1PPS1 __attribute__((address(0xF1F1)));


extern volatile __bit CLCIN1PPS2 __attribute__((address(0xF1F2)));


extern volatile __bit CLCIN1PPS3 __attribute__((address(0xF1F3)));


extern volatile __bit CLCIN1PPS4 __attribute__((address(0xF1F4)));


extern volatile __bit CLCIN2PPS0 __attribute__((address(0xF1F8)));


extern volatile __bit CLCIN2PPS1 __attribute__((address(0xF1F9)));


extern volatile __bit CLCIN2PPS2 __attribute__((address(0xF1FA)));


extern volatile __bit CLCIN2PPS3 __attribute__((address(0xF1FB)));


extern volatile __bit CLCIN2PPS4 __attribute__((address(0xF1FC)));


extern volatile __bit CLCIN3PPS0 __attribute__((address(0xF200)));


extern volatile __bit CLCIN3PPS1 __attribute__((address(0xF201)));


extern volatile __bit CLCIN3PPS2 __attribute__((address(0xF202)));


extern volatile __bit CLCIN3PPS3 __attribute__((address(0xF203)));


extern volatile __bit CLCIN3PPS4 __attribute__((address(0xF204)));


extern volatile __bit CM1IE __attribute__((address(0x4D2)));


extern volatile __bit CM1IF __attribute__((address(0x482)));


extern volatile __bit CNT0 __attribute__((address(0xE8C8)));


extern volatile __bit CNT1 __attribute__((address(0xE8C9)));


extern volatile __bit CNT2 __attribute__((address(0xE8CA)));


extern volatile __bit CNT3 __attribute__((address(0xE8CB)));


extern volatile __bit CNT4 __attribute__((address(0xE8CC)));


extern volatile __bit CNT5 __attribute__((address(0xE8CD)));


extern volatile __bit CNT6 __attribute__((address(0xE8CE)));


extern volatile __bit CNT7 __attribute__((address(0xE8CF)));


extern volatile __bit CONT __attribute__((address(0xE936)));


extern volatile __bit COSC0 __attribute__((address(0x1074)));


extern volatile __bit COSC1 __attribute__((address(0x1075)));


extern volatile __bit COSC2 __attribute__((address(0x1076)));


extern volatile __bit CPOS __attribute__((address(0x806D)));


extern volatile __bit CPRDY __attribute__((address(0x8068)));


extern volatile __bit CPREQ __attribute__((address(0x806A)));


extern volatile __bit CPT __attribute__((address(0x8069)));


extern volatile __bit CRS0 __attribute__((address(0xE944)));


extern volatile __bit CRS1 __attribute__((address(0xE945)));


extern volatile __bit CRS2 __attribute__((address(0xE946)));


extern volatile __bit CS5 __attribute__((address(0xE96D)));


extern volatile __bit CWG1CS __attribute__((address(0x2860)));


extern volatile __bit CWG1DBF0 __attribute__((address(0x2878)));


extern volatile __bit CWG1DBF1 __attribute__((address(0x2879)));


extern volatile __bit CWG1DBF2 __attribute__((address(0x287A)));


extern volatile __bit CWG1DBF3 __attribute__((address(0x287B)));


extern volatile __bit CWG1DBF4 __attribute__((address(0x287C)));


extern volatile __bit CWG1DBF5 __attribute__((address(0x287D)));


extern volatile __bit CWG1DBR0 __attribute__((address(0x2870)));


extern volatile __bit CWG1DBR1 __attribute__((address(0x2871)));


extern volatile __bit CWG1DBR2 __attribute__((address(0x2872)));


extern volatile __bit CWG1DBR3 __attribute__((address(0x2873)));


extern volatile __bit CWG1DBR4 __attribute__((address(0x2874)));


extern volatile __bit CWG1DBR5 __attribute__((address(0x2875)));


extern volatile __bit CWG1EN __attribute__((address(0x2887)));


extern volatile __bit CWG1IE __attribute__((address(0x4C5)));


extern volatile __bit CWG1IF __attribute__((address(0x475)));


extern volatile __bit CWG1IN __attribute__((address(0x288D)));


extern volatile __bit CWG1LD __attribute__((address(0x2886)));


extern volatile __bit CWG1LSAC0 __attribute__((address(0x2892)));


extern volatile __bit CWG1LSAC1 __attribute__((address(0x2893)));


extern volatile __bit CWG1LSBD0 __attribute__((address(0x2894)));


extern volatile __bit CWG1LSBD1 __attribute__((address(0x2895)));


extern volatile __bit CWG1MODE0 __attribute__((address(0x2880)));


extern volatile __bit CWG1MODE1 __attribute__((address(0x2881)));


extern volatile __bit CWG1MODE2 __attribute__((address(0x2882)));


extern volatile __bit CWG1OVRA __attribute__((address(0x28A4)));


extern volatile __bit CWG1OVRB __attribute__((address(0x28A5)));


extern volatile __bit CWG1OVRC __attribute__((address(0x28A6)));


extern volatile __bit CWG1OVRD __attribute__((address(0x28A7)));


extern volatile __bit CWG1POLA __attribute__((address(0x2888)));


extern volatile __bit CWG1POLB __attribute__((address(0x2889)));


extern volatile __bit CWG1POLC __attribute__((address(0x288A)));


extern volatile __bit CWG1POLD __attribute__((address(0x288B)));


extern volatile __bit CWG1PPS0 __attribute__((address(0xF1C8)));


extern volatile __bit CWG1PPS1 __attribute__((address(0xF1C9)));


extern volatile __bit CWG1PPS2 __attribute__((address(0xF1CA)));


extern volatile __bit CWG1PPS3 __attribute__((address(0xF1CB)));


extern volatile __bit CWG1PPS4 __attribute__((address(0xF1CC)));


extern volatile __bit CWG1STRA __attribute__((address(0x28A0)));


extern volatile __bit CWG1STRB __attribute__((address(0x28A1)));


extern volatile __bit CWG1STRC __attribute__((address(0x28A2)));


extern volatile __bit CWG1STRD __attribute__((address(0x28A3)));


extern volatile __bit D1S0 __attribute__((address(0x2C70)));


extern volatile __bit D1S1 __attribute__((address(0x2C71)));


extern volatile __bit D1S2 __attribute__((address(0x2C72)));


extern volatile __bit D1S3 __attribute__((address(0x2C73)));


extern volatile __bit D1S4 __attribute__((address(0x2C74)));


extern volatile __bit D1S5 __attribute__((address(0x2C75)));


extern volatile __bit D2S0 __attribute__((address(0x2C78)));


extern volatile __bit D2S1 __attribute__((address(0x2C79)));


extern volatile __bit D2S2 __attribute__((address(0x2C7A)));


extern volatile __bit D2S3 __attribute__((address(0x2C7B)));


extern volatile __bit D2S4 __attribute__((address(0x2C7C)));


extern volatile __bit D2S5 __attribute__((address(0x2C7D)));


extern volatile __bit D3S0 __attribute__((address(0x2C80)));


extern volatile __bit D3S1 __attribute__((address(0x2C81)));


extern volatile __bit D3S2 __attribute__((address(0x2C82)));


extern volatile __bit D3S3 __attribute__((address(0x2C83)));


extern volatile __bit D3S4 __attribute__((address(0x2C84)));


extern volatile __bit D3S5 __attribute__((address(0x2C85)));


extern volatile __bit D4S0 __attribute__((address(0x2C88)));


extern volatile __bit D4S1 __attribute__((address(0x2C89)));


extern volatile __bit D4S2 __attribute__((address(0x2C8A)));


extern volatile __bit D4S3 __attribute__((address(0x2C8B)));


extern volatile __bit D4S4 __attribute__((address(0x2C8C)));


extern volatile __bit D4S5 __attribute__((address(0x2C8D)));


extern volatile __bit DA1 __attribute__((address(0x347D)));


extern volatile __bit DA2 __attribute__((address(0x34CD)));


extern volatile __bit DAT0 __attribute__((address(0x2868)));


extern volatile __bit DAT1 __attribute__((address(0x2869)));


extern volatile __bit DAT2 __attribute__((address(0x286A)));


extern volatile __bit DAT3 __attribute__((address(0x286B)));


extern volatile __bit DATA_ADDRESS1 __attribute__((address(0x347D)));


extern volatile __bit DATA_ADDRESS2 __attribute__((address(0x34CD)));


extern volatile __bit DBF0 __attribute__((address(0x2878)));


extern volatile __bit DBF1 __attribute__((address(0x2879)));


extern volatile __bit DBF2 __attribute__((address(0x287A)));


extern volatile __bit DBF3 __attribute__((address(0x287B)));


extern volatile __bit DBF4 __attribute__((address(0x287C)));


extern volatile __bit DBF5 __attribute__((address(0x287D)));


extern volatile __bit DBR0 __attribute__((address(0x2870)));


extern volatile __bit DBR1 __attribute__((address(0x2871)));


extern volatile __bit DBR2 __attribute__((address(0x2872)));


extern volatile __bit DBR3 __attribute__((address(0x2873)));


extern volatile __bit DBR4 __attribute__((address(0x2874)));


extern volatile __bit DBR5 __attribute__((address(0x2875)));


extern volatile __bit DC __attribute__((address(0x19)));


extern volatile __bit DONE __attribute__((address(0xE930)));


extern volatile __bit DSEN __attribute__((address(0xE938)));


extern volatile __bit D_A1 __attribute__((address(0x347D)));


extern volatile __bit D_A2 __attribute__((address(0x34CD)));


extern volatile __bit D_nA1 __attribute__((address(0x347D)));


extern volatile __bit D_nA2 __attribute__((address(0x34CD)));


extern volatile __bit ERR0 __attribute__((address(0xE880)));


extern volatile __bit ERR1 __attribute__((address(0xE881)));


extern volatile __bit ERR10 __attribute__((address(0xE88A)));


extern volatile __bit ERR11 __attribute__((address(0xE88B)));


extern volatile __bit ERR12 __attribute__((address(0xE88C)));


extern volatile __bit ERR13 __attribute__((address(0xE88D)));


extern volatile __bit ERR14 __attribute__((address(0xE88E)));


extern volatile __bit ERR15 __attribute__((address(0xE88F)));


extern volatile __bit ERR2 __attribute__((address(0xE882)));


extern volatile __bit ERR3 __attribute__((address(0xE883)));


extern volatile __bit ERR4 __attribute__((address(0xE884)));


extern volatile __bit ERR5 __attribute__((address(0xE885)));


extern volatile __bit ERR6 __attribute__((address(0xE886)));


extern volatile __bit ERR7 __attribute__((address(0xE887)));


extern volatile __bit ERR8 __attribute__((address(0xE888)));


extern volatile __bit ERR9 __attribute__((address(0xE889)));


extern volatile __bit FLTR0 __attribute__((address(0xE8A0)));


extern volatile __bit FLTR1 __attribute__((address(0xE8A1)));


extern volatile __bit FLTR10 __attribute__((address(0xE8AA)));


extern volatile __bit FLTR11 __attribute__((address(0xE8AB)));


extern volatile __bit FLTR12 __attribute__((address(0xE8AC)));


extern volatile __bit FLTR13 __attribute__((address(0xE8AD)));


extern volatile __bit FLTR14 __attribute__((address(0xE8AE)));


extern volatile __bit FLTR15 __attribute__((address(0xE8AF)));


extern volatile __bit FLTR2 __attribute__((address(0xE8A2)));


extern volatile __bit FLTR3 __attribute__((address(0xE8A3)));


extern volatile __bit FLTR4 __attribute__((address(0xE8A4)));


extern volatile __bit FLTR5 __attribute__((address(0xE8A5)));


extern volatile __bit FLTR6 __attribute__((address(0xE8A6)));


extern volatile __bit FLTR7 __attribute__((address(0xE8A7)));


extern volatile __bit FLTR8 __attribute__((address(0xE8A8)));


extern volatile __bit FLTR9 __attribute__((address(0xE8A9)));


extern volatile __bit FM0 __attribute__((address(0xE932)));


extern volatile __bit FREE __attribute__((address(0xE484)));


extern volatile __bit FRQ0 __attribute__((address(0x1098)));


extern volatile __bit FRQ1 __attribute__((address(0x1099)));


extern volatile __bit FRQ2 __attribute__((address(0x109A)));


extern volatile __bit FVREN __attribute__((address(0x8067)));


extern volatile __bit FVRRDY __attribute__((address(0x8066)));


extern volatile __bit G1D1N __attribute__((address(0x2C90)));


extern volatile __bit G1D1T __attribute__((address(0x2C91)));


extern volatile __bit G1D2N __attribute__((address(0x2C92)));


extern volatile __bit G1D2T __attribute__((address(0x2C93)));


extern volatile __bit G1D3N __attribute__((address(0x2C94)));


extern volatile __bit G1D3T __attribute__((address(0x2C95)));


extern volatile __bit G1D4N __attribute__((address(0x2C96)));


extern volatile __bit G1D4T __attribute__((address(0x2C97)));


extern volatile __bit G1EN __attribute__((address(0x2887)));


extern volatile __bit G1POL __attribute__((address(0x2C68)));


extern volatile __bit G2D1N __attribute__((address(0x2C98)));


extern volatile __bit G2D1T __attribute__((address(0x2C99)));


extern volatile __bit G2D2N __attribute__((address(0x2C9A)));


extern volatile __bit G2D2T __attribute__((address(0x2C9B)));


extern volatile __bit G2D3N __attribute__((address(0x2C9C)));


extern volatile __bit G2D3T __attribute__((address(0x2C9D)));


extern volatile __bit G2D4N __attribute__((address(0x2C9E)));


extern volatile __bit G2D4T __attribute__((address(0x2C9F)));


extern volatile __bit G2POL __attribute__((address(0x2C69)));


extern volatile __bit G3D1N __attribute__((address(0x2CA0)));


extern volatile __bit G3D1T __attribute__((address(0x2CA1)));


extern volatile __bit G3D2N __attribute__((address(0x2CA2)));


extern volatile __bit G3D2T __attribute__((address(0x2CA3)));


extern volatile __bit G3D3N __attribute__((address(0x2CA4)));


extern volatile __bit G3D3T __attribute__((address(0x2CA5)));


extern volatile __bit G3D4N __attribute__((address(0x2CA6)));


extern volatile __bit G3D4T __attribute__((address(0x2CA7)));


extern volatile __bit G3POL __attribute__((address(0x2C6A)));


extern volatile __bit G4D1N __attribute__((address(0x2CA8)));


extern volatile __bit G4D1T __attribute__((address(0x2CA9)));


extern volatile __bit G4D2N __attribute__((address(0x2CAA)));


extern volatile __bit G4D2T __attribute__((address(0x2CAB)));


extern volatile __bit G4D3N __attribute__((address(0x2CAC)));


extern volatile __bit G4D3T __attribute__((address(0x2CAD)));


extern volatile __bit G4D4N __attribute__((address(0x2CAE)));


extern volatile __bit G4D4T __attribute__((address(0x2CAF)));


extern volatile __bit G4POL __attribute__((address(0x2C6B)));


extern volatile __bit GCEN1 __attribute__((address(0x348F)));


extern volatile __bit GCEN2 __attribute__((address(0x34DF)));


extern volatile __bit GIE __attribute__((address(0x5F)));


extern volatile __bit GO __attribute__((address(0xE930)));


extern volatile __bit GO_nDONE __attribute__((address(0xE930)));


extern volatile __bit HFOEN __attribute__((address(0x108E)));


extern volatile __bit HFOR __attribute__((address(0x1086)));


extern volatile __bit HYS __attribute__((address(0x8461)));


extern volatile __bit I2C_DAT1 __attribute__((address(0x347D)));


extern volatile __bit I2C_DAT2 __attribute__((address(0x34CD)));


extern volatile __bit I2C_READ1 __attribute__((address(0x347A)));


extern volatile __bit I2C_READ2 __attribute__((address(0x34CA)));


extern volatile __bit I2C_START1 __attribute__((address(0x347B)));


extern volatile __bit I2C_START2 __attribute__((address(0x34CB)));


extern volatile __bit IN __attribute__((address(0x288D)));


extern volatile __bit INC0 __attribute__((address(0x2478)));


extern volatile __bit INC1 __attribute__((address(0x2479)));


extern volatile __bit INC10 __attribute__((address(0x2482)));


extern volatile __bit INC11 __attribute__((address(0x2483)));


extern volatile __bit INC12 __attribute__((address(0x2484)));


extern volatile __bit INC13 __attribute__((address(0x2485)));


extern volatile __bit INC14 __attribute__((address(0x2486)));


extern volatile __bit INC15 __attribute__((address(0x2487)));


extern volatile __bit INC16 __attribute__((address(0x2488)));


extern volatile __bit INC17 __attribute__((address(0x2489)));


extern volatile __bit INC18 __attribute__((address(0x248A)));


extern volatile __bit INC19 __attribute__((address(0x248B)));


extern volatile __bit INC2 __attribute__((address(0x247A)));


extern volatile __bit INC3 __attribute__((address(0x247B)));


extern volatile __bit INC4 __attribute__((address(0x247C)));


extern volatile __bit INC5 __attribute__((address(0x247D)));


extern volatile __bit INC6 __attribute__((address(0x247E)));


extern volatile __bit INC7 __attribute__((address(0x247F)));


extern volatile __bit INC8 __attribute__((address(0x2480)));


extern volatile __bit INC9 __attribute__((address(0x2481)));


extern volatile __bit INLVLA0 __attribute__((address(0xF480)));


extern volatile __bit INLVLA1 __attribute__((address(0xF481)));


extern volatile __bit INLVLA2 __attribute__((address(0xF482)));


extern volatile __bit INLVLA4 __attribute__((address(0xF484)));


extern volatile __bit INLVLA5 __attribute__((address(0xF485)));


extern volatile __bit INTE __attribute__((address(0x4B0)));


extern volatile __bit INTEDG __attribute__((address(0x58)));


extern volatile __bit INTF __attribute__((address(0x460)));


extern volatile __bit INTPPS0 __attribute__((address(0xF068)));


extern volatile __bit INTPPS1 __attribute__((address(0xF069)));


extern volatile __bit INTPPS2 __attribute__((address(0xF06A)));


extern volatile __bit INTPPS3 __attribute__((address(0xF06B)));


extern volatile __bit INTPPS4 __attribute__((address(0xF06C)));


extern volatile __bit IOCAF0 __attribute__((address(0xF498)));


extern volatile __bit IOCAF1 __attribute__((address(0xF499)));


extern volatile __bit IOCAF2 __attribute__((address(0xF49A)));


extern volatile __bit IOCAF3 __attribute__((address(0xF49B)));


extern volatile __bit IOCAF4 __attribute__((address(0xF49C)));


extern volatile __bit IOCAF5 __attribute__((address(0xF49D)));


extern volatile __bit IOCAN0 __attribute__((address(0xF490)));


extern volatile __bit IOCAN1 __attribute__((address(0xF491)));


extern volatile __bit IOCAN2 __attribute__((address(0xF492)));


extern volatile __bit IOCAN3 __attribute__((address(0xF493)));


extern volatile __bit IOCAN4 __attribute__((address(0xF494)));


extern volatile __bit IOCAN5 __attribute__((address(0xF495)));


extern volatile __bit IOCAP0 __attribute__((address(0xF488)));


extern volatile __bit IOCAP1 __attribute__((address(0xF489)));


extern volatile __bit IOCAP2 __attribute__((address(0xF48A)));


extern volatile __bit IOCAP3 __attribute__((address(0xF48B)));


extern volatile __bit IOCAP4 __attribute__((address(0xF48C)));


extern volatile __bit IOCAP5 __attribute__((address(0xF48D)));


extern volatile __bit IOCIE __attribute__((address(0x4B4)));


extern volatile __bit IOCIF __attribute__((address(0x464)));


extern volatile __bit IPEN __attribute__((address(0xE93E)));


extern volatile __bit ISM0 __attribute__((address(0x2868)));


extern volatile __bit ISM1 __attribute__((address(0x2869)));


extern volatile __bit ISM2 __attribute__((address(0x286A)));


extern volatile __bit ISM3 __attribute__((address(0x286B)));


extern volatile __bit LATA0 __attribute__((address(0xC0)));


extern volatile __bit LATA1 __attribute__((address(0xC1)));


extern volatile __bit LATA2 __attribute__((address(0xC2)));


extern volatile __bit LATA4 __attribute__((address(0xC4)));


extern volatile __bit LATA5 __attribute__((address(0xC5)));


extern volatile __bit LD __attribute__((address(0x2886)));


extern volatile __bit LFOEN __attribute__((address(0x108C)));


extern volatile __bit LFOR __attribute__((address(0x1084)));


extern volatile __bit LSAC0 __attribute__((address(0x2892)));


extern volatile __bit LSBD0 __attribute__((address(0x2894)));


extern volatile __bit LSBD1 __attribute__((address(0x2895)));


extern volatile __bit LSCA1 __attribute__((address(0x2893)));


extern volatile __bit LTH0 __attribute__((address(0xE860)));


extern volatile __bit LTH1 __attribute__((address(0xE861)));


extern volatile __bit LTH10 __attribute__((address(0xE86A)));


extern volatile __bit LTH11 __attribute__((address(0xE86B)));


extern volatile __bit LTH12 __attribute__((address(0xE86C)));


extern volatile __bit LTH13 __attribute__((address(0xE86D)));


extern volatile __bit LTH14 __attribute__((address(0xE86E)));


extern volatile __bit LTH15 __attribute__((address(0xE86F)));


extern volatile __bit LTH2 __attribute__((address(0xE862)));


extern volatile __bit LTH3 __attribute__((address(0xE863)));


extern volatile __bit LTH4 __attribute__((address(0xE864)));


extern volatile __bit LTH5 __attribute__((address(0xE865)));


extern volatile __bit LTH6 __attribute__((address(0xE866)));


extern volatile __bit LTH7 __attribute__((address(0xE867)));


extern volatile __bit LTH8 __attribute__((address(0xE868)));


extern volatile __bit LTH9 __attribute__((address(0xE869)));


extern volatile __bit LTHR __attribute__((address(0xE955)));


extern volatile __bit LWLO __attribute__((address(0xE485)));


extern volatile __bit MATH __attribute__((address(0xE954)));


extern volatile __bit MD0 __attribute__((address(0xE940)));


extern volatile __bit MD1 __attribute__((address(0xE941)));


extern volatile __bit MD16 __attribute__((address(0xCF4)));


extern volatile __bit MD2 __attribute__((address(0xE942)));


extern volatile __bit MFOEN __attribute__((address(0x108D)));


extern volatile __bit MFOR __attribute__((address(0x1085)));


extern volatile __bit MSK01 __attribute__((address(0x3468)));


extern volatile __bit MSK02 __attribute__((address(0x34B8)));


extern volatile __bit MSK11 __attribute__((address(0x3469)));


extern volatile __bit MSK12 __attribute__((address(0x34B9)));


extern volatile __bit MSK21 __attribute__((address(0x346A)));


extern volatile __bit MSK22 __attribute__((address(0x34BA)));


extern volatile __bit MSK31 __attribute__((address(0x346B)));


extern volatile __bit MSK32 __attribute__((address(0x34BB)));


extern volatile __bit MSK41 __attribute__((address(0x346C)));


extern volatile __bit MSK42 __attribute__((address(0x34BC)));


extern volatile __bit MSK51 __attribute__((address(0x346D)));


extern volatile __bit MSK52 __attribute__((address(0x34BD)));


extern volatile __bit MSK61 __attribute__((address(0x346E)));


extern volatile __bit MSK62 __attribute__((address(0x34BE)));


extern volatile __bit MSK71 __attribute__((address(0x346F)));


extern volatile __bit MSK72 __attribute__((address(0x34BF)));


extern volatile __bit NCH0 __attribute__((address(0x8470)));


extern volatile __bit NCH1 __attribute__((address(0x8471)));


extern volatile __bit NCH2 __attribute__((address(0x8472)));


extern volatile __bit NCO1ACC0 __attribute__((address(0x2460)));


extern volatile __bit NCO1ACC1 __attribute__((address(0x2461)));


extern volatile __bit NCO1ACC10 __attribute__((address(0x246A)));


extern volatile __bit NCO1ACC11 __attribute__((address(0x246B)));


extern volatile __bit NCO1ACC12 __attribute__((address(0x246C)));


extern volatile __bit NCO1ACC13 __attribute__((address(0x246D)));


extern volatile __bit NCO1ACC14 __attribute__((address(0x246E)));


extern volatile __bit NCO1ACC15 __attribute__((address(0x246F)));


extern volatile __bit NCO1ACC16 __attribute__((address(0x2470)));


extern volatile __bit NCO1ACC17 __attribute__((address(0x2471)));


extern volatile __bit NCO1ACC18 __attribute__((address(0x2472)));


extern volatile __bit NCO1ACC19 __attribute__((address(0x2473)));


extern volatile __bit NCO1ACC2 __attribute__((address(0x2462)));


extern volatile __bit NCO1ACC3 __attribute__((address(0x2463)));


extern volatile __bit NCO1ACC4 __attribute__((address(0x2464)));


extern volatile __bit NCO1ACC5 __attribute__((address(0x2465)));


extern volatile __bit NCO1ACC6 __attribute__((address(0x2466)));


extern volatile __bit NCO1ACC7 __attribute__((address(0x2467)));


extern volatile __bit NCO1ACC8 __attribute__((address(0x2468)));


extern volatile __bit NCO1ACC9 __attribute__((address(0x2469)));


extern volatile __bit NCO1CKS0 __attribute__((address(0x2498)));


extern volatile __bit NCO1CKS1 __attribute__((address(0x2499)));


extern volatile __bit NCO1CKS2 __attribute__((address(0x249A)));


extern volatile __bit NCO1CKS3 __attribute__((address(0x249B)));


extern volatile __bit NCO1EN __attribute__((address(0x2497)));


extern volatile __bit NCO1IE __attribute__((address(0x4C4)));


extern volatile __bit NCO1IF __attribute__((address(0x474)));


extern volatile __bit NCO1INC0 __attribute__((address(0x2478)));


extern volatile __bit NCO1INC1 __attribute__((address(0x2479)));


extern volatile __bit NCO1INC10 __attribute__((address(0x2482)));


extern volatile __bit NCO1INC11 __attribute__((address(0x2483)));


extern volatile __bit NCO1INC12 __attribute__((address(0x2484)));


extern volatile __bit NCO1INC13 __attribute__((address(0x2485)));


extern volatile __bit NCO1INC14 __attribute__((address(0x2486)));


extern volatile __bit NCO1INC15 __attribute__((address(0x2487)));


extern volatile __bit NCO1INC16 __attribute__((address(0x2488)));


extern volatile __bit NCO1INC17 __attribute__((address(0x2489)));


extern volatile __bit NCO1INC18 __attribute__((address(0x248A)));


extern volatile __bit NCO1INC19 __attribute__((address(0x248B)));


extern volatile __bit NCO1INC2 __attribute__((address(0x247A)));


extern volatile __bit NCO1INC3 __attribute__((address(0x247B)));


extern volatile __bit NCO1INC4 __attribute__((address(0x247C)));


extern volatile __bit NCO1INC5 __attribute__((address(0x247D)));


extern volatile __bit NCO1INC6 __attribute__((address(0x247E)));


extern volatile __bit NCO1INC7 __attribute__((address(0x247F)));


extern volatile __bit NCO1INC8 __attribute__((address(0x2480)));


extern volatile __bit NCO1INC9 __attribute__((address(0x2481)));


extern volatile __bit NCO1OUT __attribute__((address(0x2495)));


extern volatile __bit NCO1PFM __attribute__((address(0x2490)));


extern volatile __bit NCO1POL __attribute__((address(0x2494)));


extern volatile __bit NCO1PWS0 __attribute__((address(0x249D)));


extern volatile __bit NCO1PWS1 __attribute__((address(0x249E)));


extern volatile __bit NCO1PWS2 __attribute__((address(0x249F)));


extern volatile __bit NSS __attribute__((address(0x8860)));


extern volatile __bit NVMADR0 __attribute__((address(0xE460)));


extern volatile __bit NVMADR1 __attribute__((address(0xE461)));


extern volatile __bit NVMADR10 __attribute__((address(0xE46A)));


extern volatile __bit NVMADR11 __attribute__((address(0xE46B)));


extern volatile __bit NVMADR12 __attribute__((address(0xE46C)));


extern volatile __bit NVMADR13 __attribute__((address(0xE46D)));


extern volatile __bit NVMADR14 __attribute__((address(0xE46E)));


extern volatile __bit NVMADR2 __attribute__((address(0xE462)));


extern volatile __bit NVMADR3 __attribute__((address(0xE463)));


extern volatile __bit NVMADR4 __attribute__((address(0xE464)));


extern volatile __bit NVMADR5 __attribute__((address(0xE465)));


extern volatile __bit NVMADR6 __attribute__((address(0xE466)));


extern volatile __bit NVMADR7 __attribute__((address(0xE467)));


extern volatile __bit NVMADR8 __attribute__((address(0xE468)));


extern volatile __bit NVMADR9 __attribute__((address(0xE469)));


extern volatile __bit NVMDAT0 __attribute__((address(0xE470)));


extern volatile __bit NVMDAT1 __attribute__((address(0xE471)));


extern volatile __bit NVMDAT10 __attribute__((address(0xE47A)));


extern volatile __bit NVMDAT11 __attribute__((address(0xE47B)));


extern volatile __bit NVMDAT12 __attribute__((address(0xE47C)));


extern volatile __bit NVMDAT13 __attribute__((address(0xE47D)));


extern volatile __bit NVMDAT2 __attribute__((address(0xE472)));


extern volatile __bit NVMDAT3 __attribute__((address(0xE473)));


extern volatile __bit NVMDAT4 __attribute__((address(0xE474)));


extern volatile __bit NVMDAT5 __attribute__((address(0xE475)));


extern volatile __bit NVMDAT6 __attribute__((address(0xE476)));


extern volatile __bit NVMDAT7 __attribute__((address(0xE477)));


extern volatile __bit NVMDAT8 __attribute__((address(0xE478)));


extern volatile __bit NVMDAT9 __attribute__((address(0xE479)));


extern volatile __bit NVMIE __attribute__((address(0x4B8)));


extern volatile __bit NVMIF __attribute__((address(0x468)));


extern volatile __bit NVMREGS __attribute__((address(0xE486)));


extern volatile __bit ODCA0 __attribute__((address(0xF470)));


extern volatile __bit ODCA1 __attribute__((address(0xF471)));


extern volatile __bit ODCA2 __attribute__((address(0xF472)));


extern volatile __bit ODCA4 __attribute__((address(0xF474)));


extern volatile __bit ODCA5 __attribute__((address(0xF475)));


extern volatile __bit OE0 __attribute__((address(0x8864)));


extern volatile __bit OE1 __attribute__((address(0x8865)));


extern volatile __bit OV __attribute__((address(0xE957)));


extern volatile __bit OVRA __attribute__((address(0x28A4)));


extern volatile __bit OVRB __attribute__((address(0x28A5)));


extern volatile __bit OVRC __attribute__((address(0x28A6)));


extern volatile __bit OVRD __attribute__((address(0x28A7)));


extern volatile __bit P1M1 __attribute__((address(0x1C77)));


extern volatile __bit P2M1 __attribute__((address(0x1C97)));


extern volatile __bit PCH0 __attribute__((address(0x8478)));


extern volatile __bit PCH1 __attribute__((address(0x8479)));


extern volatile __bit PCH2 __attribute__((address(0x847A)));


extern volatile __bit PCSC __attribute__((address(0xE939)));


extern volatile __bit PEIE __attribute__((address(0x5E)));


extern volatile __bit PEN1 __attribute__((address(0x348A)));


extern volatile __bit PEN2 __attribute__((address(0x34DA)));


extern volatile __bit PFM __attribute__((address(0x2490)));


extern volatile __bit POLA __attribute__((address(0x2888)));


extern volatile __bit POLB __attribute__((address(0x2889)));


extern volatile __bit POLC __attribute__((address(0x288A)));


extern volatile __bit POLD __attribute__((address(0x288B)));


extern volatile __bit PPOL __attribute__((address(0xE93F)));


extern volatile __bit PPSLOCKED __attribute__((address(0xF060)));


extern volatile __bit PRE0 __attribute__((address(0xE920)));


extern volatile __bit PRE1 __attribute__((address(0xE921)));


extern volatile __bit PRE10 __attribute__((address(0xE92A)));


extern volatile __bit PRE11 __attribute__((address(0xE92B)));


extern volatile __bit PRE12 __attribute__((address(0xE92C)));


extern volatile __bit PRE2 __attribute__((address(0xE922)));


extern volatile __bit PRE3 __attribute__((address(0xE923)));


extern volatile __bit PRE4 __attribute__((address(0xE924)));


extern volatile __bit PRE5 __attribute__((address(0xE925)));


extern volatile __bit PRE6 __attribute__((address(0xE926)));


extern volatile __bit PRE7 __attribute__((address(0xE927)));


extern volatile __bit PRE8 __attribute__((address(0xE928)));


extern volatile __bit PRE9 __attribute__((address(0xE929)));


extern volatile __bit PREF0 __attribute__((address(0xE958)));


extern volatile __bit PREF1 __attribute__((address(0xE959)));


extern volatile __bit PREV0 __attribute__((address(0xE8D8)));


extern volatile __bit PREV1 __attribute__((address(0xE8D9)));


extern volatile __bit PREV10 __attribute__((address(0xE8E2)));


extern volatile __bit PREV11 __attribute__((address(0xE8E3)));


extern volatile __bit PREV12 __attribute__((address(0xE8E4)));


extern volatile __bit PREV13 __attribute__((address(0xE8E5)));


extern volatile __bit PREV14 __attribute__((address(0xE8E6)));


extern volatile __bit PREV15 __attribute__((address(0xE8E7)));


extern volatile __bit PREV2 __attribute__((address(0xE8DA)));


extern volatile __bit PREV3 __attribute__((address(0xE8DB)));


extern volatile __bit PREV4 __attribute__((address(0xE8DC)));


extern volatile __bit PREV5 __attribute__((address(0xE8DD)));


extern volatile __bit PREV6 __attribute__((address(0xE8DE)));


extern volatile __bit PREV7 __attribute__((address(0xE8DF)));


extern volatile __bit PREV8 __attribute__((address(0xE8E0)));


extern volatile __bit PREV9 __attribute__((address(0xE8E1)));


extern volatile __bit PS0 __attribute__((address(0xC61)));


extern volatile __bit PS1 __attribute__((address(0xC62)));


extern volatile __bit PS2 __attribute__((address(0xC63)));


extern volatile __bit PS3 __attribute__((address(0xC64)));


extern volatile __bit PS4 __attribute__((address(0xC65)));


extern volatile __bit PSIS __attribute__((address(0xE947)));


extern volatile __bit PSS0 __attribute__((address(0x8862)));


extern volatile __bit PSS1 __attribute__((address(0x8863)));


extern volatile __bit PWM3EN __attribute__((address(0x2077)));


extern volatile __bit PWM3OUT __attribute__((address(0x2075)));


extern volatile __bit PWM3POL __attribute__((address(0x2074)));


extern volatile __bit PWM4EN __attribute__((address(0x208F)));


extern volatile __bit PWM4OUT __attribute__((address(0x208D)));


extern volatile __bit PWM4POL __attribute__((address(0x208C)));


extern volatile __bit PWM5EN __attribute__((address(0x20A7)));


extern volatile __bit PWM5OUT __attribute__((address(0x20A5)));


extern volatile __bit PWM5POL __attribute__((address(0x20A4)));


extern volatile __bit PWS0 __attribute__((address(0x249D)));


extern volatile __bit PWS1 __attribute__((address(0x249E)));


extern volatile __bit PWS2 __attribute__((address(0x249F)));


extern volatile __bit RA0 __attribute__((address(0x60)));


extern volatile __bit RA1 __attribute__((address(0x61)));


extern volatile __bit RA2 __attribute__((address(0x62)));


extern volatile __bit RA3 __attribute__((address(0x63)));


extern volatile __bit RA4 __attribute__((address(0x64)));


extern volatile __bit RA5 __attribute__((address(0x65)));


extern volatile __bit RC1IE __attribute__((address(0x4CB)));


extern volatile __bit RC1IF __attribute__((address(0x47B)));


extern volatile __bit RC2IE __attribute__((address(0x4CD)));


extern volatile __bit RC2IF __attribute__((address(0x47D)));


extern volatile __bit RCEN1 __attribute__((address(0x348B)));


extern volatile __bit RCEN2 __attribute__((address(0x34DB)));


extern volatile __bit RD __attribute__((address(0xE480)));


extern volatile __bit RD161 __attribute__((address(0x1471)));


extern volatile __bit RD163 __attribute__((address(0x14A1)));


extern volatile __bit RDY __attribute__((address(0x8066)));


extern volatile __bit READ_WRITE1 __attribute__((address(0x347A)));


extern volatile __bit READ_WRITE2 __attribute__((address(0x34CA)));


extern volatile __bit REFRNG __attribute__((address(0x8866)));


extern volatile __bit REN __attribute__((address(0x2896)));


extern volatile __bit RES0 __attribute__((address(0xE8E8)));


extern volatile __bit RES1 __attribute__((address(0xE8E9)));


extern volatile __bit RES10 __attribute__((address(0xE8F2)));


extern volatile __bit RES11 __attribute__((address(0xE8F3)));


extern volatile __bit RES12 __attribute__((address(0xE8F4)));


extern volatile __bit RES13 __attribute__((address(0xE8F5)));


extern volatile __bit RES14 __attribute__((address(0xE8F6)));


extern volatile __bit RES15 __attribute__((address(0xE8F7)));


extern volatile __bit RES2 __attribute__((address(0xE8EA)));


extern volatile __bit RES3 __attribute__((address(0xE8EB)));


extern volatile __bit RES4 __attribute__((address(0xE8EC)));


extern volatile __bit RES5 __attribute__((address(0xE8ED)));


extern volatile __bit RES6 __attribute__((address(0xE8EE)));


extern volatile __bit RES7 __attribute__((address(0xE8EF)));


extern volatile __bit RES8 __attribute__((address(0xE8F0)));


extern volatile __bit RES9 __attribute__((address(0xE8F1)));


extern volatile __bit RPT0 __attribute__((address(0xE8D0)));


extern volatile __bit RPT1 __attribute__((address(0xE8D1)));


extern volatile __bit RPT2 __attribute__((address(0xE8D2)));


extern volatile __bit RPT3 __attribute__((address(0xE8D3)));


extern volatile __bit RPT4 __attribute__((address(0xE8D4)));


extern volatile __bit RPT5 __attribute__((address(0xE8D5)));


extern volatile __bit RPT6 __attribute__((address(0xE8D6)));


extern volatile __bit RPT7 __attribute__((address(0xE8D7)));


extern volatile __bit RSEN1 __attribute__((address(0x3489)));


extern volatile __bit RSEN2 __attribute__((address(0x34D9)));


extern volatile __bit RW1 __attribute__((address(0x347A)));


extern volatile __bit RW2 __attribute__((address(0x34CA)));


extern volatile __bit R_W1 __attribute__((address(0x347A)));


extern volatile __bit R_W2 __attribute__((address(0x34CA)));


extern volatile __bit R_nW1 __attribute__((address(0x347A)));


extern volatile __bit R_nW2 __attribute__((address(0x34CA)));


extern volatile __bit SBOREN __attribute__((address(0xC6F)));


extern volatile __bit SEN1 __attribute__((address(0x3488)));


extern volatile __bit SEN2 __attribute__((address(0x34D8)));


extern volatile __bit SFOR __attribute__((address(0x1081)));


extern volatile __bit SHUTDOWN __attribute__((address(0x2897)));


extern volatile __bit SLCT0 __attribute__((address(0x2CB0)));


extern volatile __bit SLCT1 __attribute__((address(0x2CB1)));


extern volatile __bit SLCT2 __attribute__((address(0x2CB2)));


extern volatile __bit SLCT3 __attribute__((address(0x2CB3)));


extern volatile __bit SLRA0 __attribute__((address(0xF478)));


extern volatile __bit SLRA1 __attribute__((address(0xF479)));


extern volatile __bit SLRA2 __attribute__((address(0xF47A)));


extern volatile __bit SLRA4 __attribute__((address(0xF47C)));


extern volatile __bit SLRA5 __attribute__((address(0xF47D)));


extern volatile __bit SMP1 __attribute__((address(0x347F)));


extern volatile __bit SMP2 __attribute__((address(0x34CF)));


extern volatile __bit SOI __attribute__((address(0xE94B)));


extern volatile __bit SOR __attribute__((address(0x1083)));


extern volatile __bit SOSCEN __attribute__((address(0x108B)));


extern volatile __bit SOSCPWR __attribute__((address(0x107E)));


extern volatile __bit SSP1CLKPPS0 __attribute__((address(0xF238)));


extern volatile __bit SSP1CLKPPS1 __attribute__((address(0xF239)));


extern volatile __bit SSP1CLKPPS2 __attribute__((address(0xF23A)));


extern volatile __bit SSP1CLKPPS3 __attribute__((address(0xF23B)));


extern volatile __bit SSP1CLKPPS4 __attribute__((address(0xF23C)));


extern volatile __bit SSP1DATPPS0 __attribute__((address(0xF240)));


extern volatile __bit SSP1DATPPS1 __attribute__((address(0xF241)));


extern volatile __bit SSP1DATPPS2 __attribute__((address(0xF242)));


extern volatile __bit SSP1DATPPS3 __attribute__((address(0xF243)));


extern volatile __bit SSP1DATPPS4 __attribute__((address(0xF244)));


extern volatile __bit SSP1IE __attribute__((address(0x4CE)));


extern volatile __bit SSP1IF __attribute__((address(0x47E)));


extern volatile __bit SSP1SSPPS0 __attribute__((address(0xF248)));


extern volatile __bit SSP1SSPPS1 __attribute__((address(0xF249)));


extern volatile __bit SSP1SSPPS2 __attribute__((address(0xF24A)));


extern volatile __bit SSP1SSPPS3 __attribute__((address(0xF24B)));


extern volatile __bit SSP1SSPPS4 __attribute__((address(0xF24C)));


extern volatile __bit SSP2IE __attribute__((address(0x4D0)));


extern volatile __bit SSP2IF __attribute__((address(0x480)));


extern volatile __bit SSPEN1 __attribute__((address(0x3485)));


extern volatile __bit SSPEN2 __attribute__((address(0x34D5)));


extern volatile __bit SSPM01 __attribute__((address(0x3480)));


extern volatile __bit SSPM02 __attribute__((address(0x34D0)));


extern volatile __bit SSPM11 __attribute__((address(0x3481)));


extern volatile __bit SSPM12 __attribute__((address(0x34D1)));


extern volatile __bit SSPM21 __attribute__((address(0x3482)));


extern volatile __bit SSPM22 __attribute__((address(0x34D2)));


extern volatile __bit SSPM31 __attribute__((address(0x3483)));


extern volatile __bit SSPM32 __attribute__((address(0x34D3)));


extern volatile __bit SSPOV1 __attribute__((address(0x3486)));


extern volatile __bit SSPOV2 __attribute__((address(0x34D6)));


extern volatile __bit START1 __attribute__((address(0x347B)));


extern volatile __bit START2 __attribute__((address(0x34CB)));


extern volatile __bit STAT0 __attribute__((address(0xE950)));


extern volatile __bit STAT1 __attribute__((address(0xE951)));


extern volatile __bit STAT2 __attribute__((address(0xE952)));


extern volatile __bit STATUS_SHAD0 __attribute__((address(0xFF20)));


extern volatile __bit STATUS_SHAD1 __attribute__((address(0xFF21)));


extern volatile __bit STATUS_SHAD2 __attribute__((address(0xFF22)));


extern volatile __bit STKOVF __attribute__((address(0xC77)));


extern volatile __bit STKPTR0 __attribute__((address(0xFF68)));


extern volatile __bit STKPTR1 __attribute__((address(0xFF69)));


extern volatile __bit STKPTR2 __attribute__((address(0xFF6A)));


extern volatile __bit STKPTR3 __attribute__((address(0xFF6B)));


extern volatile __bit STKPTR4 __attribute__((address(0xFF6C)));


extern volatile __bit STKUNF __attribute__((address(0xC76)));


extern volatile __bit STOP1 __attribute__((address(0x347C)));


extern volatile __bit STOP2 __attribute__((address(0x34CC)));


extern volatile __bit STPT0 __attribute__((address(0xE890)));


extern volatile __bit STPT1 __attribute__((address(0xE891)));


extern volatile __bit STPT10 __attribute__((address(0xE89A)));


extern volatile __bit STPT11 __attribute__((address(0xE89B)));


extern volatile __bit STPT12 __attribute__((address(0xE89C)));


extern volatile __bit STPT13 __attribute__((address(0xE89D)));


extern volatile __bit STPT15 __attribute__((address(0xE89E)));


extern volatile __bit STPT16 __attribute__((address(0xE89F)));


extern volatile __bit STPT2 __attribute__((address(0xE892)));


extern volatile __bit STPT3 __attribute__((address(0xE893)));


extern volatile __bit STPT4 __attribute__((address(0xE894)));


extern volatile __bit STPT5 __attribute__((address(0xE895)));


extern volatile __bit STPT6 __attribute__((address(0xE896)));


extern volatile __bit STPT7 __attribute__((address(0xE897)));


extern volatile __bit STPT8 __attribute__((address(0xE898)));


extern volatile __bit STPT9 __attribute__((address(0xE899)));


extern volatile __bit STRA __attribute__((address(0x28A0)));


extern volatile __bit STRB __attribute__((address(0x28A1)));


extern volatile __bit STRC __attribute__((address(0x28A2)));


extern volatile __bit STRD __attribute__((address(0x28A3)));


extern volatile __bit SWDTEN __attribute__((address(0xC60)));


extern volatile __bit T016BIT __attribute__((address(0xCF4)));


extern volatile __bit T0ASYNC __attribute__((address(0xCFC)));


extern volatile __bit T0CKIPPS0 __attribute__((address(0xF070)));


extern volatile __bit T0CKIPPS1 __attribute__((address(0xF071)));


extern volatile __bit T0CKIPPS2 __attribute__((address(0xF072)));


extern volatile __bit T0CKIPPS3 __attribute__((address(0xF073)));


extern volatile __bit T0CKIPPS4 __attribute__((address(0xF074)));


extern volatile __bit T0CKPS0 __attribute__((address(0xCF8)));


extern volatile __bit T0CKPS1 __attribute__((address(0xCF9)));


extern volatile __bit T0CKPS2 __attribute__((address(0xCFA)));


extern volatile __bit T0CKPS3 __attribute__((address(0xCFB)));


extern volatile __bit T0CS0 __attribute__((address(0xCFD)));


extern volatile __bit T0CS1 __attribute__((address(0xCFE)));


extern volatile __bit T0CS2 __attribute__((address(0xCFF)));


extern volatile __bit T0EN __attribute__((address(0xCF7)));


extern volatile __bit T0MD16 __attribute__((address(0xCF4)));


extern volatile __bit T0OUT __attribute__((address(0xCF5)));


extern volatile __bit T0OUTPS0 __attribute__((address(0xCF0)));


extern volatile __bit T0OUTPS1 __attribute__((address(0xCF1)));


extern volatile __bit T0OUTPS2 __attribute__((address(0xCF2)));


extern volatile __bit T0OUTPS3 __attribute__((address(0xCF3)));


extern volatile __bit T0PR0 __attribute__((address(0xCE8)));


extern volatile __bit T0PR1 __attribute__((address(0xCE9)));


extern volatile __bit T0PR2 __attribute__((address(0xCEA)));


extern volatile __bit T0PR3 __attribute__((address(0xCEB)));


extern volatile __bit T0PR4 __attribute__((address(0xCEC)));


extern volatile __bit T0PR5 __attribute__((address(0xCED)));


extern volatile __bit T0PR6 __attribute__((address(0xCEE)));


extern volatile __bit T0PR7 __attribute__((address(0xCEF)));


extern volatile __bit T1CKIPPS0 __attribute__((address(0xF078)));


extern volatile __bit T1CKIPPS1 __attribute__((address(0xF079)));


extern volatile __bit T1CKIPPS2 __attribute__((address(0xF07A)));


extern volatile __bit T1CKIPPS3 __attribute__((address(0xF07B)));


extern volatile __bit T1CKIPPS4 __attribute__((address(0xF07C)));


extern volatile __bit T1CKPS0 __attribute__((address(0x1474)));


extern volatile __bit T1CKPS1 __attribute__((address(0x1475)));


extern volatile __bit T1CS0 __attribute__((address(0x1488)));


extern volatile __bit T1CS1 __attribute__((address(0x1489)));


extern volatile __bit T1CS2 __attribute__((address(0x148A)));


extern volatile __bit T1CS3 __attribute__((address(0x148B)));


extern volatile __bit T1CS4 __attribute__((address(0x148C)));


extern volatile __bit T1GE __attribute__((address(0x147F)));


extern volatile __bit T1GGO __attribute__((address(0x147B)));


extern volatile __bit T1GGO_nDONE __attribute__((address(0x147B)));


extern volatile __bit T1GPOL __attribute__((address(0x147E)));


extern volatile __bit T1GPPS0 __attribute__((address(0xF080)));


extern volatile __bit T1GPPS1 __attribute__((address(0xF081)));


extern volatile __bit T1GPPS2 __attribute__((address(0xF082)));


extern volatile __bit T1GPPS3 __attribute__((address(0xF083)));


extern volatile __bit T1GPPS4 __attribute__((address(0xF084)));


extern volatile __bit T1GSPM __attribute__((address(0x147C)));


extern volatile __bit T1GSS0 __attribute__((address(0x1480)));


extern volatile __bit T1GSS1 __attribute__((address(0x1481)));


extern volatile __bit T1GSS2 __attribute__((address(0x1482)));


extern volatile __bit T1GSS3 __attribute__((address(0x1483)));


extern volatile __bit T1GSS4 __attribute__((address(0x1484)));


extern volatile __bit T1GTM __attribute__((address(0x147D)));


extern volatile __bit T1GVAL __attribute__((address(0x147A)));


extern volatile __bit T1RD16 __attribute__((address(0x1471)));


extern volatile __bit T2CKPS0 __attribute__((address(0x1874)));


extern volatile __bit T2CKPS1 __attribute__((address(0x1875)));


extern volatile __bit T2CKPS2 __attribute__((address(0x1876)));


extern volatile __bit T2CPOL __attribute__((address(0x187E)));


extern volatile __bit T2CS0 __attribute__((address(0x1880)));


extern volatile __bit T2CS1 __attribute__((address(0x1881)));


extern volatile __bit T2CS2 __attribute__((address(0x1882)));


extern volatile __bit T2CS3 __attribute__((address(0x1883)));


extern volatile __bit T2CSYNC __attribute__((address(0x187D)));


extern volatile __bit T2INPPS0 __attribute__((address(0xF0C8)));


extern volatile __bit T2INPPS1 __attribute__((address(0xF0C9)));


extern volatile __bit T2INPPS2 __attribute__((address(0xF0CA)));


extern volatile __bit T2INPPS3 __attribute__((address(0xF0CB)));


extern volatile __bit T2INPPS4 __attribute__((address(0xF0CC)));


extern volatile __bit T2MODE0 __attribute__((address(0x1878)));


extern volatile __bit T2MODE1 __attribute__((address(0x1879)));


extern volatile __bit T2MODE2 __attribute__((address(0x187A)));


extern volatile __bit T2MODE3 __attribute__((address(0x187B)));


extern volatile __bit T2MODE4 __attribute__((address(0x187C)));


extern volatile __bit T2ON __attribute__((address(0x1877)));


extern volatile __bit T2OUTPS0 __attribute__((address(0x1870)));


extern volatile __bit T2OUTPS1 __attribute__((address(0x1871)));


extern volatile __bit T2OUTPS2 __attribute__((address(0x1872)));


extern volatile __bit T2OUTPS3 __attribute__((address(0x1873)));


extern volatile __bit T2PSYNC __attribute__((address(0x187F)));


extern volatile __bit T2RSEL0 __attribute__((address(0x1888)));


extern volatile __bit T2RSEL1 __attribute__((address(0x1889)));


extern volatile __bit T2RSEL2 __attribute__((address(0x188A)));


extern volatile __bit T2RSEL3 __attribute__((address(0x188B)));


extern volatile __bit T3CKPS0 __attribute__((address(0x14A4)));


extern volatile __bit T3CKPS1 __attribute__((address(0x14A5)));


extern volatile __bit T3CS0 __attribute__((address(0x14B8)));


extern volatile __bit T3CS1 __attribute__((address(0x14B9)));


extern volatile __bit T3CS2 __attribute__((address(0x14BA)));


extern volatile __bit T3CS3 __attribute__((address(0x14BB)));


extern volatile __bit T3CS4 __attribute__((address(0x14BC)));


extern volatile __bit T3GE __attribute__((address(0x14AF)));


extern volatile __bit T3GGO __attribute__((address(0x14AB)));


extern volatile __bit T3GGO_nDONE __attribute__((address(0x14AB)));


extern volatile __bit T3GPOL __attribute__((address(0x14AE)));


extern volatile __bit T3GSPM __attribute__((address(0x14AC)));


extern volatile __bit T3GSS0 __attribute__((address(0x14B0)));


extern volatile __bit T3GSS1 __attribute__((address(0x14B1)));


extern volatile __bit T3GSS2 __attribute__((address(0x14B2)));


extern volatile __bit T3GSS3 __attribute__((address(0x14B3)));


extern volatile __bit T3GSS4 __attribute__((address(0x14B4)));


extern volatile __bit T3GTM __attribute__((address(0x14AD)));


extern volatile __bit T3GVAL __attribute__((address(0x14AA)));


extern volatile __bit T3RD16 __attribute__((address(0x14A1)));


extern volatile __bit T4CKPS0 __attribute__((address(0x18A4)));


extern volatile __bit T4CKPS1 __attribute__((address(0x18A5)));


extern volatile __bit T4CKPS2 __attribute__((address(0x18A6)));


extern volatile __bit T4CPOL __attribute__((address(0x18AE)));


extern volatile __bit T4CS0 __attribute__((address(0x18B0)));


extern volatile __bit T4CS1 __attribute__((address(0x18B1)));


extern volatile __bit T4CS2 __attribute__((address(0x18B2)));


extern volatile __bit T4CS3 __attribute__((address(0x18B3)));


extern volatile __bit T4CSYNC __attribute__((address(0x18AD)));


extern volatile __bit T4MODE0 __attribute__((address(0x18A8)));


extern volatile __bit T4MODE1 __attribute__((address(0x18A9)));


extern volatile __bit T4MODE2 __attribute__((address(0x18AA)));


extern volatile __bit T4MODE3 __attribute__((address(0x18AB)));


extern volatile __bit T4MODE4 __attribute__((address(0x18AC)));


extern volatile __bit T4ON __attribute__((address(0x18A7)));


extern volatile __bit T4OUTPS0 __attribute__((address(0x18A0)));


extern volatile __bit T4OUTPS1 __attribute__((address(0x18A1)));


extern volatile __bit T4OUTPS2 __attribute__((address(0x18A2)));


extern volatile __bit T4OUTPS3 __attribute__((address(0x18A3)));


extern volatile __bit T4PSYNC __attribute__((address(0x18AF)));


extern volatile __bit T4RSEL0 __attribute__((address(0x18B8)));


extern volatile __bit T4RSEL1 __attribute__((address(0x18B9)));


extern volatile __bit T4RSEL2 __attribute__((address(0x18BA)));


extern volatile __bit T4RSEL3 __attribute__((address(0x18BB)));


extern volatile __bit T6CKPS0 __attribute__((address(0x18D4)));


extern volatile __bit T6CKPS1 __attribute__((address(0x18D5)));


extern volatile __bit T6CKPS2 __attribute__((address(0x18D6)));


extern volatile __bit T6CPOL __attribute__((address(0x18DE)));


extern volatile __bit T6CS0 __attribute__((address(0x18E0)));


extern volatile __bit T6CS1 __attribute__((address(0x18E1)));


extern volatile __bit T6CS2 __attribute__((address(0x18E2)));


extern volatile __bit T6CS3 __attribute__((address(0x18E3)));


extern volatile __bit T6CSYNC __attribute__((address(0x18DD)));


extern volatile __bit T6MODE0 __attribute__((address(0x18D8)));


extern volatile __bit T6MODE1 __attribute__((address(0x18D9)));


extern volatile __bit T6MODE2 __attribute__((address(0x18DA)));


extern volatile __bit T6MODE3 __attribute__((address(0x18DB)));


extern volatile __bit T6MODE4 __attribute__((address(0x18DC)));


extern volatile __bit T6ON __attribute__((address(0x18D7)));


extern volatile __bit T6OUTPS0 __attribute__((address(0x18D0)));


extern volatile __bit T6OUTPS1 __attribute__((address(0x18D1)));


extern volatile __bit T6OUTPS2 __attribute__((address(0x18D2)));


extern volatile __bit T6OUTPS3 __attribute__((address(0x18D3)));


extern volatile __bit T6PSYNC __attribute__((address(0x18DF)));


extern volatile __bit T6RSEL0 __attribute__((address(0x18E8)));


extern volatile __bit T6RSEL1 __attribute__((address(0x18E9)));


extern volatile __bit T6RSEL2 __attribute__((address(0x18EA)));


extern volatile __bit T6RSEL3 __attribute__((address(0x18EB)));


extern volatile __bit TMD0 __attribute__((address(0xE948)));


extern volatile __bit TMD1 __attribute__((address(0xE949)));


extern volatile __bit TMD2 __attribute__((address(0xE94A)));


extern volatile __bit TMR0H0 __attribute__((address(0xCE8)));


extern volatile __bit TMR0H1 __attribute__((address(0xCE9)));


extern volatile __bit TMR0H2 __attribute__((address(0xCEA)));


extern volatile __bit TMR0H3 __attribute__((address(0xCEB)));


extern volatile __bit TMR0H4 __attribute__((address(0xCEC)));


extern volatile __bit TMR0H5 __attribute__((address(0xCED)));


extern volatile __bit TMR0H6 __attribute__((address(0xCEE)));


extern volatile __bit TMR0H7 __attribute__((address(0xCEF)));


extern volatile __bit TMR0IE __attribute__((address(0x4B5)));


extern volatile __bit TMR0IF __attribute__((address(0x465)));


extern volatile __bit TMR0L0 __attribute__((address(0xCE0)));


extern volatile __bit TMR0L1 __attribute__((address(0xCE1)));


extern volatile __bit TMR0L2 __attribute__((address(0xCE2)));


extern volatile __bit TMR0L3 __attribute__((address(0xCE3)));


extern volatile __bit TMR0L4 __attribute__((address(0xCE4)));


extern volatile __bit TMR0L5 __attribute__((address(0xCE5)));


extern volatile __bit TMR0L6 __attribute__((address(0xCE6)));


extern volatile __bit TMR0L7 __attribute__((address(0xCE7)));


extern volatile __bit TMR10 __attribute__((address(0x1460)));


extern volatile __bit TMR11 __attribute__((address(0x1461)));


extern volatile __bit TMR110 __attribute__((address(0x146A)));


extern volatile __bit TMR111 __attribute__((address(0x146B)));


extern volatile __bit TMR112 __attribute__((address(0x146C)));


extern volatile __bit TMR113 __attribute__((address(0x146D)));


extern volatile __bit TMR114 __attribute__((address(0x146E)));


extern volatile __bit TMR115 __attribute__((address(0x146F)));


extern volatile __bit TMR12 __attribute__((address(0x1462)));


extern volatile __bit TMR13 __attribute__((address(0x1463)));


extern volatile __bit TMR14 __attribute__((address(0x1464)));


extern volatile __bit TMR15 __attribute__((address(0x1465)));


extern volatile __bit TMR16 __attribute__((address(0x1466)));


extern volatile __bit TMR17 __attribute__((address(0x1467)));


extern volatile __bit TMR18 __attribute__((address(0x1468)));


extern volatile __bit TMR19 __attribute__((address(0x1469)));


extern volatile __bit TMR1GIE __attribute__((address(0x4BC)));


extern volatile __bit TMR1GIF __attribute__((address(0x46C)));


extern volatile __bit TMR1H0 __attribute__((address(0x1468)));


extern volatile __bit TMR1H1 __attribute__((address(0x1469)));


extern volatile __bit TMR1H2 __attribute__((address(0x146A)));


extern volatile __bit TMR1H3 __attribute__((address(0x146B)));


extern volatile __bit TMR1H4 __attribute__((address(0x146C)));


extern volatile __bit TMR1H5 __attribute__((address(0x146D)));


extern volatile __bit TMR1H6 __attribute__((address(0x146E)));


extern volatile __bit TMR1H7 __attribute__((address(0x146F)));


extern volatile __bit TMR1IE __attribute__((address(0x4BB)));


extern volatile __bit TMR1IF __attribute__((address(0x46B)));


extern volatile __bit TMR1L0 __attribute__((address(0x1460)));


extern volatile __bit TMR1L1 __attribute__((address(0x1461)));


extern volatile __bit TMR1L2 __attribute__((address(0x1462)));


extern volatile __bit TMR1L3 __attribute__((address(0x1463)));


extern volatile __bit TMR1L4 __attribute__((address(0x1464)));


extern volatile __bit TMR1L5 __attribute__((address(0x1465)));


extern volatile __bit TMR1L6 __attribute__((address(0x1466)));


extern volatile __bit TMR1L7 __attribute__((address(0x1467)));


extern volatile __bit TMR1ON __attribute__((address(0x1470)));


extern volatile __bit TMR2IE __attribute__((address(0x4BF)));


extern volatile __bit TMR2IF __attribute__((address(0x46F)));


extern volatile __bit TMR2ON __attribute__((address(0x1877)));


extern volatile __bit TMR30 __attribute__((address(0x1490)));


extern volatile __bit TMR31 __attribute__((address(0x1491)));


extern volatile __bit TMR310 __attribute__((address(0x149A)));


extern volatile __bit TMR311 __attribute__((address(0x149B)));


extern volatile __bit TMR312 __attribute__((address(0x149C)));


extern volatile __bit TMR313 __attribute__((address(0x149D)));


extern volatile __bit TMR314 __attribute__((address(0x149E)));


extern volatile __bit TMR315 __attribute__((address(0x149F)));


extern volatile __bit TMR32 __attribute__((address(0x1492)));


extern volatile __bit TMR33 __attribute__((address(0x1493)));


extern volatile __bit TMR34 __attribute__((address(0x1494)));


extern volatile __bit TMR35 __attribute__((address(0x1495)));


extern volatile __bit TMR36 __attribute__((address(0x1496)));


extern volatile __bit TMR37 __attribute__((address(0x1497)));


extern volatile __bit TMR38 __attribute__((address(0x1498)));


extern volatile __bit TMR39 __attribute__((address(0x1499)));


extern volatile __bit TMR3GIE __attribute__((address(0x4BE)));


extern volatile __bit TMR3GIF __attribute__((address(0x46E)));


extern volatile __bit TMR3H0 __attribute__((address(0x1498)));


extern volatile __bit TMR3H1 __attribute__((address(0x1499)));


extern volatile __bit TMR3H2 __attribute__((address(0x149A)));


extern volatile __bit TMR3H3 __attribute__((address(0x149B)));


extern volatile __bit TMR3H4 __attribute__((address(0x149C)));


extern volatile __bit TMR3H5 __attribute__((address(0x149D)));


extern volatile __bit TMR3H6 __attribute__((address(0x149E)));


extern volatile __bit TMR3H7 __attribute__((address(0x149F)));


extern volatile __bit TMR3IE __attribute__((address(0x4BD)));


extern volatile __bit TMR3IF __attribute__((address(0x46D)));


extern volatile __bit TMR3L0 __attribute__((address(0x1490)));


extern volatile __bit TMR3L1 __attribute__((address(0x1491)));


extern volatile __bit TMR3L2 __attribute__((address(0x1492)));


extern volatile __bit TMR3L3 __attribute__((address(0x1493)));


extern volatile __bit TMR3L4 __attribute__((address(0x1494)));


extern volatile __bit TMR3L5 __attribute__((address(0x1495)));


extern volatile __bit TMR3L6 __attribute__((address(0x1496)));


extern volatile __bit TMR3L7 __attribute__((address(0x1497)));


extern volatile __bit TMR3ON __attribute__((address(0x14A0)));


extern volatile __bit TMR4IE __attribute__((address(0x4C0)));


extern volatile __bit TMR4IF __attribute__((address(0x470)));


extern volatile __bit TMR4ON __attribute__((address(0x18A7)));


extern volatile __bit TMR6IE __attribute__((address(0x4C1)));


extern volatile __bit TMR6IF __attribute__((address(0x471)));


extern volatile __bit TMR6ON __attribute__((address(0x18D7)));


extern volatile __bit TOSH0 __attribute__((address(0xFF78)));


extern volatile __bit TOSH1 __attribute__((address(0xFF79)));


extern volatile __bit TOSH2 __attribute__((address(0xFF7A)));


extern volatile __bit TOSH3 __attribute__((address(0xFF7B)));


extern volatile __bit TOSH4 __attribute__((address(0xFF7C)));


extern volatile __bit TOSH5 __attribute__((address(0xFF7D)));


extern volatile __bit TOSH6 __attribute__((address(0xFF7E)));


extern volatile __bit TOSH7 __attribute__((address(0xFF7F)));


extern volatile __bit TOSL0 __attribute__((address(0xFF70)));


extern volatile __bit TOSL1 __attribute__((address(0xFF71)));


extern volatile __bit TOSL2 __attribute__((address(0xFF72)));


extern volatile __bit TOSL3 __attribute__((address(0xFF73)));


extern volatile __bit TOSL4 __attribute__((address(0xFF74)));


extern volatile __bit TOSL5 __attribute__((address(0xFF75)));


extern volatile __bit TOSL6 __attribute__((address(0xFF76)));


extern volatile __bit TOSL7 __attribute__((address(0xFF77)));


extern volatile __bit TRISA0 __attribute__((address(0x90)));


extern volatile __bit TRISA1 __attribute__((address(0x91)));


extern volatile __bit TRISA2 __attribute__((address(0x92)));


extern volatile __bit TRISA4 __attribute__((address(0x94)));


extern volatile __bit TRISA5 __attribute__((address(0x95)));


extern volatile __bit TSEN __attribute__((address(0x8065)));


extern volatile __bit TSRNG __attribute__((address(0x8064)));


extern volatile __bit TUN0 __attribute__((address(0x1090)));


extern volatile __bit TUN1 __attribute__((address(0x1091)));


extern volatile __bit TUN2 __attribute__((address(0x1092)));


extern volatile __bit TUN3 __attribute__((address(0x1093)));


extern volatile __bit TUN4 __attribute__((address(0x1094)));


extern volatile __bit TUN5 __attribute__((address(0x1095)));


extern volatile __bit TX1IE __attribute__((address(0x4CA)));


extern volatile __bit TX1IF __attribute__((address(0x47A)));


extern volatile __bit TX2IE __attribute__((address(0x4CC)));


extern volatile __bit TX2IF __attribute__((address(0x47C)));


extern volatile __bit UA1 __attribute__((address(0x3479)));


extern volatile __bit UA2 __attribute__((address(0x34C9)));


extern volatile __bit UTH0 __attribute__((address(0xE870)));


extern volatile __bit UTH1 __attribute__((address(0xE871)));


extern volatile __bit UTH10 __attribute__((address(0xE87A)));


extern volatile __bit UTH11 __attribute__((address(0xE87B)));


extern volatile __bit UTH12 __attribute__((address(0xE87C)));


extern volatile __bit UTH13 __attribute__((address(0xE87D)));


extern volatile __bit UTH14 __attribute__((address(0xE87E)));


extern volatile __bit UTH15 __attribute__((address(0xE87F)));


extern volatile __bit UTH2 __attribute__((address(0xE872)));


extern volatile __bit UTH3 __attribute__((address(0xE873)));


extern volatile __bit UTH4 __attribute__((address(0xE874)));


extern volatile __bit UTH5 __attribute__((address(0xE875)));


extern volatile __bit UTH6 __attribute__((address(0xE876)));


extern volatile __bit UTH7 __attribute__((address(0xE877)));


extern volatile __bit UTH8 __attribute__((address(0xE878)));


extern volatile __bit UTH9 __attribute__((address(0xE879)));


extern volatile __bit UTHR __attribute__((address(0xE956)));


extern volatile __bit WCOL1 __attribute__((address(0x3487)));


extern volatile __bit WCOL2 __attribute__((address(0x34D7)));


extern volatile __bit WDTCS __attribute__((address(0xC67)));


extern volatile __bit WDTPS0 __attribute__((address(0xC61)));


extern volatile __bit WDTPS1 __attribute__((address(0xC62)));


extern volatile __bit WDTPS2 __attribute__((address(0xC63)));


extern volatile __bit WDTPS3 __attribute__((address(0xC64)));


extern volatile __bit WDTPS4 __attribute__((address(0xC65)));


extern volatile __bit WPUA0 __attribute__((address(0xF468)));


extern volatile __bit WPUA1 __attribute__((address(0xF469)));


extern volatile __bit WPUA2 __attribute__((address(0xF46A)));


extern volatile __bit WPUA3 __attribute__((address(0xF46B)));


extern volatile __bit WPUA4 __attribute__((address(0xF46C)));


extern volatile __bit WPUA5 __attribute__((address(0xF46D)));


extern volatile __bit WR __attribute__((address(0xE481)));


extern volatile __bit WREN __attribute__((address(0xE482)));


extern volatile __bit WRERR __attribute__((address(0xE483)));


extern volatile __bit ZCDIE __attribute__((address(0x4D5)));


extern volatile __bit ZCDIF __attribute__((address(0x485)));


extern volatile __bit ZCDINTN __attribute__((address(0x80F8)));


extern volatile __bit ZCDINTP __attribute__((address(0x80F9)));


extern volatile __bit ZCDOUT __attribute__((address(0x80FD)));


extern volatile __bit ZCDPOL __attribute__((address(0x80FC)));


extern volatile __bit ZCDSEN __attribute__((address(0x80FF)));


extern volatile __bit ZERO __attribute__((address(0x1A)));


extern volatile __bit nADDRESS1 __attribute__((address(0x347D)));


extern volatile __bit nADDRESS2 __attribute__((address(0x34CD)));


extern volatile __bit nBOR __attribute__((address(0xC70)));


extern volatile __bit nMEMV __attribute__((address(0xC79)));


extern volatile __bit nPD __attribute__((address(0x1B)));


extern volatile __bit nPOR __attribute__((address(0xC71)));


extern volatile __bit nRI __attribute__((address(0xC72)));


extern volatile __bit nRMCLR __attribute__((address(0xC73)));


extern volatile __bit nRWDT __attribute__((address(0xC74)));


extern volatile __bit nT1SYNC __attribute__((address(0x1472)));


extern volatile __bit nT3SYNC __attribute__((address(0x14A2)));


extern volatile __bit nTO __attribute__((address(0x1C)));


extern volatile __bit nWRITE1 __attribute__((address(0x347A)));


extern volatile __bit nWRITE2 __attribute__((address(0x34CA)));
# 260 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\pic_chip_select.h" 2 3
# 14 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\pic.h" 2 3
# 76 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\pic.h" 3
__attribute__((__unsupported__("The " "FLASH_READ" " macro function is no longer supported. Please use the MPLAB X MCC."))) unsigned char __flash_read(unsigned short addr);

__attribute__((__unsupported__("The " "FLASH_WRITE" " macro function is no longer supported. Please use the MPLAB X MCC."))) void __flash_write(unsigned short addr, unsigned short data);

__attribute__((__unsupported__("The " "FLASH_ERASE" " macro function is no longer supported. Please use the MPLAB X MCC."))) void __flash_erase(unsigned short addr);



# 1 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\eeprom_routines.h" 1 3
# 114 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\eeprom_routines.h" 3
extern void eeprom_write(unsigned char addr, unsigned char value);
extern unsigned char eeprom_read(unsigned char addr);
# 84 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\pic.h" 2 3
# 118 "C:/Users/Nono/.mchp_packs/Microchip/PIC16F1xxxx_DFP/1.22.376/xc8\\pic\\include\\pic.h" 3
extern __bank0 unsigned char __resetbits;
extern __bank0 __bit __powerdown;
extern __bank0 __bit __timeout;
# 28 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\xc.h" 2 3
# 50 "main.c" 2





# 1 "./watchdog_driver.h" 1
void WDT_setup(void);
void SLEEP_start(void);

void WDT_setup(void){


    WDTCONbits.PS = 0b10000;
}

void SLEEP_start(void){

    __asm("CLRWDT");

    __asm("SLEEP");
}
# 55 "main.c" 2

# 1 "./eusart_driver.h" 1
void EUSART_setup(void);
void EUSART_write(uint8_t txData);
void EUSART_print(const char* string);
void EUSART_print_num(uint8_t number);

void EUSART_setup(void){

    BAUD2CONbits.BRG16 = 1;

    TX2STAbits.SYNC = 0;

    TX2STAbits.TXEN = 1;

    TX2STAbits.BRGH = 1;

    RC2STAbits.CREN = 1;

    RC2STAbits.SPEN = 1;

    SP2BRGL = 25;
    SP2BRGH = 0;

    RA5PPS = 0x11;

    RX2PPS = 0x4;

    TRISAbits.TRISA5 = 0;

    TRISAbits.TRISA4 = 1;
}

void EUSART_write(uint8_t txData){
    while(PIR3bits.TX2IF == 0){}
    TX2REG = txData;
}

uint8_t EUSART_read_wait(void){
    while(!PIR3bits.RC2IF);
    return RC2REG;
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
# 56 "main.c" 2

# 1 "./i2c_driver.h" 1
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
void I2C_MCP23008_write(uint8_t address, uint8_t reg_addr, uint8_t data);
void I2C_MCP23008_read(void);


void I2C_setup(void){


    uint8_t i2c_freq = 100;

    TRISAbits.TRISA1 = 1;
    TRISAbits.TRISA2 = 1;
    ODCONAbits.ODCA1 = 1;
    ODCONAbits.ODCA2 = 1;
    RA1PPS = 0x15;
    RA2PPS = 0x16;
    SSP1CLKPPS = 0x1;
    SSP1DATPPS = 0x2;

    SSP1STATbits.SMP = 1;

    SSP1CON1bits.SSPM = 0b1000;

    SSP1CON2 = 0;

    SSP1ADD = 3;

    SSP1CON1bits.SSPEN = 1;

}

void I2C_wait(void){
   while ((SSP1STAT & 0b00000100) || (SSP1CON2 & 0x00011111));
}

void I2C_start(void){
  I2C_wait();
  SSP1CON2bits.SEN = 1;
}

void I2C_RepeatedStart(void){
  I2C_wait();
  SSP1CON2bits.RSEN = 1;
}

void I2C_stop(void){
  I2C_wait();
  SSP1CON2bits.PEN = 1;
}

void I2C_write(uint8_t data){
  I2C_wait();
  SSP1BUF = data;
}

uint8_t I2C_read(){
  uint8_t tmp;
  I2C_wait();
  SSP1CON2bits.RCEN = 1;
  I2C_wait();
  tmp = SSP1BUF;
  I2C_wait();
  SSP1CON2bits.ACKDT = 0;
  SSP1CON2bits.ACKEN = 1;
  return tmp;
}

uint8_t I2C_write_query(uint8_t address, uint8_t data){
    I2C_start();
    address = (address << 1)&0b11111110;
    I2C_write(address & 0b11111110);
    I2C_write(data);
    I2C_stop();
    return 0;
}

uint8_t I2C_read_query(uint8_t address, uint8_t *data, uint8_t number_of_bytes){
    I2C_start();
    address = (address << 1)&0b11111110;
    I2C_write(address | 0b00000001);
    for(int i=0; i<number_of_bytes; i++){
        data[i] = I2C_read();
    }
    I2C_stop();
    return 0;
}

uint8_t I2C_SHT4x_read(float *t_degC, float *rh_pRH){
    I2C_write_query(0x44,0xFD);
    _delay((unsigned long)((10)*(1000000/4000.0)));
    uint8_t rx_data[6];
    I2C_read_query(0x44, rx_data, 6);
    uint16_t raw_temp = (rx_data[0]<<8) + rx_data[1];

    uint16_t raw_rh = (rx_data[3]<<8) + rx_data[4];

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

void I2C_MCP23008_write(uint8_t address, uint8_t reg_addr, uint8_t data){
    I2C_start();
    address = (address << 1)&0b11111110;
    I2C_write(address & 0b11111110);
    I2C_write(reg_addr);
    I2C_write(data);
    I2C_stop();
}

void I2C_MCP23008_read(void){
    uint8_t rx_data[11];
    I2C_read_query(0x27, rx_data, 11);
}
# 57 "main.c" 2

# 1 "./analog_driver.h" 1



void Analog_setup(void);
uint16_t Analog_read(void);

void Analog_setup(void){

    TRISA |= (1<<0);
    ANSELA |= (1<<0);
    ADPCH = 0 & 0b00111111;
    ADREF = 0x0;
    ADCON0bits.CS = 1;
    ADCON0bits.FM = 1;
}

uint16_t Analog_read_raw(void){
    ADCON0bits.ON = 1;
    ADCON0bits.GO = 1;
    while(ADCON0bits.GO);

    return ADRES;
}

uint16_t Analog_read_voltage(void){
    uint16_t a = Analog_read();
    uint16_t r = a*(330.0/1023.0);
    return ADRES;
}
# 58 "main.c" 2

# 1 "./lora_driver.h" 1
void LoRa_setup(void);

void LoRa_setup(void){






}
# 59 "main.c" 2
# 68 "main.c"
void main(void) {

    TRISA &= !(1<<4);

    ANSELA = 0x0;


    I2C_setup();


    EUSART_setup();


    Analog_setup();


    WDT_setup();


    uint8_t sleep_counter = 0;



    float temp;
    float humidity;

    while(1){
# 105 "main.c"
        EUSART_print("Hello ! I am a PIC ! ");
# 118 "main.c"
        I2C_MCP23008_read();
        LoRa_setup();
        _delay((unsigned long)((1000)*(1000000/4000.0)));
    }
    return;
}
