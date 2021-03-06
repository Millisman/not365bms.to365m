#ifndef powerConstants_h
#define powerConstants_h

/* --- system prescaler --- */
enum prescalers_t {
    PRESCALER_1,
    PRESCALER_2,
    PRESCALER_4,
    PRESCALER_8,
    PRESCALER_16,
    PRESCALER_32,
    PRESCALER_64,
    PRESCALER_128,
    PRESCALER_256,
};

/* --- sleep periods --- */
enum sleepprds_t {
    SLEEP_16MS,
    SLEEP_32MS,
    SLEEP_64MS,
    SLEEP_128MS,
    SLEEP_256MS,
    SLEEP_512MS,
    SLEEP_1024MS,
    SLEEP_2048MS,
    SLEEP_4096MS,
    SLEEP_8192MS,
    SLEEP_FOREVER,
};

/* --- sleep modes --- */
enum sleepmodes_t {
    IDLE_SLEEP	= SLEEP_MODE_IDLE,
    ADC_SLEEP = SLEEP_MODE_ADC,
    POWERDOWN_SLEEP = SLEEP_MODE_PWR_DOWN,
#if defined (SLEEP_MODE_EXT_STANDBY)
    EXTSTANDBY_SLEEP = SLEEP_MODE_EXT_STANDBY,
#endif 
#if defined (SLEEP_MODE_PWR_SAVE)
    POWERSAVE_SLEEP = SLEEP_MODE_PWR_SAVE,
#endif 
#if defined (SLEEP_MODE_STANDBY)
    STANDBY_SLEEP = SLEEP_MODE_STANDBY
#endif 
};

/* --- periphery disable / enable --- */
#if defined(__AVR_ATtiny85__) || defined(__AVR_ATtiny167__) || defined(__AVR_ATtiny84__)
#define PWR_ALL 0xFFFF
#define PWR_LIN _BV(5)
#define PWR_SPI _BV(4)
#define PWR_TIMER1 _BV(3)
#define PWR_TIMER0 _BV(2)
#define PWR_USI _BV(1)
#define PWR_ADC _BV(0)
#else
#define PWR_ALL 0xFFFF
#define PWR_USB _BV(15)
#define PWR_TIMER5 _BV(13)
#define PWR_TIMER4 _BV(12)
#define PWR_TIMER3 _BV(11)
#define PWR_UART3 _BV(10)
#define PWR_UART2 _BV(9)
#define PWR_UART1 _BV(8)
#define PWR_I2C _BV(7)
#define PWR_TIMER2 _BV(6)
#define PWR_TIMER0 _BV(5)
#define PWR_TIMER1 _BV(3)
#define PWR_SPI _BV(2)
#define PWR_UART0 _BV(1)
#define PWR_ADC _BV(0)
#endif

/* Спец. переопределения */
#if defined (WDTIE)
#define WDIE WDTIE
#endif

#if defined (WDTCR)
#define WDTCSR WDTCR
#endif

/* Поддержка millis-correct */
#if defined(__AVR_ATmega328P__) \
|| defined(__AVR_ATmega168__) \
|| defined(__AVR_ATmega1280__) \
|| defined(__AVR_ATmega2560__)
///#define MILLIS_CORRECT_IS_SUPPURT
#endif

#endif
