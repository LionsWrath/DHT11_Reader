#define F_CPU 16000000L 
#define BAUD 9600

#include <avr/io.h>
#include <util/delay.h>
#include <string.h>
#include <util/setbaud.h>
#include <avr/interrupt.h>

#define DHT_PIN     PB0
#define DHT_PORT    PORTB
#define DHT_DDR     DDRB
#define DHT_IN      PINB

#define tstmask(reg, bit) (reg&(1<<bit))
#define setmask(reg, bit) (reg|=(1<<bit))
#define clrmask(reg, bit) (reg&=~(1<< bit))
#define cplmask(reg, bit) (reg^=(1<<bit))

// Support Functions -------------------------------------------------

void printInt(int n) {
  char num[9];
  
  itoa(n, num, 10);
  
  uart_putstr(num);
}

void printTemperature(int d, int n) {
    uart_putstr("T = ");
    printInt(d);
    uart_putchar('.');
    printInt(n);
    uart_putstr("*C");
}

void printHumidity(int d, int n) {
    uart_putstr("H = ");
    printInt(d);
    uart_putchar('.');
    printInt(n);
    uart_putchar('%');
}

void reset(int dataArray[]) {
    int i;
    for (i=0; i<4; i++)
        dataArray[i] = 0;
}

// Delay for the given number of microseconds. For 8 or 16 MHz
void _delay(unsigned int us) {
        // calling avrlib's delay_us() function with low values (e.g. 1 or
        // 2 microseconds) gives delays longer than desired.
        //delay_us(us);
        // for the 16 MHz clock on most Arduino boards

        // for a one-microsecond delay, simply return.  the overhead
        // of the function call yields a delay of approximately 1 1/8 us.
        if (--us == 0)
                return;

        // the following loop takes a quarter of a microsecond (4 cycles)
        // per iteration, so execute it four times for each microsecond of
        // delay requested.
        us <<= 2;

        // Account for the time taken in the preceeding commands.
        us -= 2;

        // busy wait
        __asm__ __volatile__ (
                "1: sbiw %0,1" "\n\t" // 2 cycles
                "brne 1b" : "=w" (us) : "0" (us) // 2 cycles
        );
}

// UART --------------------------------------------------------------

// Serial initial configuration
void uart_init(void) {
    UBRR0H = UBRRH_VALUE;
    UBRR0L = UBRRL_VALUE;

#if USE_2X
    UCSR0A |= _BV(U2X0);
#else
    UCSR0A &= ~(_BV(U2X0));
#endif

    UCSR0C = _BV(UCSZ01) | _BV(UCSZ00);         // 8-bit data 
    UCSR0B = _BV(RXEN0) | _BV(TXEN0);           // Enable RX and TX 
}

// Send a char via Serial
void uart_putchar(char c) {
    loop_until_bit_is_set(UCSR0A, UDRE0);       // Wait until data register empty. 
    UDR0 = c;
}

// Receive a char via Serial
char uart_getchar(void) {
    loop_until_bit_is_set(UCSR0A, RXC0);        // Wait until data exists. 
    return UDR0;
}

// Send a string via Serial
void uart_putstr(char str[]) {
    int i;
    for (i = 0; i < strlen(str); i++){ 
        uart_putchar(str[i]);
    }
}

// DHT ---------------------------------------------------------------

void dht_init(void) {
    TCCR0A = 0;                                 // Normal Mode
    TCCR0B = (1 << CS11);                       // Prescaler clk/8
    
    _delay_ms(2000);                            // Wait sensor setup
}

int fetchData(int dataArray[4]) {
    int i, j, data[5][8];

    // INITIALIZATION ------------------------------------------------

    clrmask(DHT_DDR, DHT_PIN);                  // Set DHT_PIN as input
    clrmask(DHT_PORT, DHT_PIN);                 // Pull-up raise - begin
    
    _delay_ms(250);                             // Wait for 250 ms
    
    // MCU OUTPUT PART -----------------------------------------------
   
    setmask(DHT_DDR, DHT_PIN);                  // Set DHT_PIN as output
    clrmask(DHT_PORT, DHT_PIN);                 // Set DHT_PIN LOW
    
    _delay_ms(20);                              // Wait for 20 ms
    
    cli();                                      // Deactivate interrupts

    // Begin of Time Critical Code
    // End Start signal
    setmask(DHT_PORT, DHT_PIN);                 // Set DHT_PIN HIGH
    
    _delay(40);                                 // Wait for 40 us
    
    // Input Pullup
    clrmask(DHT_DDR, DHT_PIN);                  // Set DHT_PIN as input
    clrmask(DHT_PORT, DHT_PIN);                 // Pull-up raise - begin
    
    _delay(10);                                 // Wait for 10 us
    
    // DHT11 PREAMBLE ------------------------------------------------

    while (!tstmask(DHT_IN, DHT_PIN));          // Wait for 80 us - LOW

    while (tstmask(DHT_IN, DHT_PIN));           // Wait for 80 us - HIGH

    // DHT11 DATA RECEIVE --------------------------------------------
    
    for (i=0; i<5; i++) {
        for (j=7; j>=0; j--) {     
            
            // Data define period - Usually 50 us LOW
            while (!tstmask(DHT_IN, DHT_PIN));
        
            TCNT0 = 0;                          // Timer 0 counter reset
            
            while (tstmask(DHT_IN, DHT_PIN));   // Receive bit            
        
            data[i][j] = TCNT0;                 // Get counter value
        }
    }

    // End of Time Critical Code
    sei();                                      // Activate interrupts
   
    // Bit Check
    // 26-28 us(bit 0) or 70 us(bit 1)
    for (i=0; i<5; i++) {
        for (j=7; j>=0; j--) {
            if (data[i][j] < 14)       clrmask(dataArray[i], j);
            else if (data[i][j] >= 14) setmask(dataArray[i], j);
        }
    }

    // DATA Checking -------------------------------------------------
    
    // Checksum verification
    if (dataArray[4] == 
       ((dataArray[0] + 
         dataArray[1] + 
         dataArray[2] + 
         dataArray[3]) 
         & 0xFF)) {
        return 0;
    }

    return 1;
}

int main(void) {
    int dt[4];
    
    init();
    uart_init();

    while (1) {
        reset(dt);
        if (fetchData(dt)) {
            // Humidity
            printHumidity(dt[0], dt[1]);

            uart_putstr("    ");

            // Temperature
            printTemperature(dt[2], dt[3]);
            
            uart_putchar('\n');
            
        } else uart_putstr("ERRO\n");
        _delay_ms(500);
    }
}
