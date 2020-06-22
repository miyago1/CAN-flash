#define FUNC_READ 1
#define FUNC_WRITE 1


#define OPTIBOOT_MAJVER 8
#define OPTIBOOT_MINVER 1

/*
 * OPTIBOOT_CUSTOMVER should be defined (by the makefile) for custom edits
 * of optiboot.  That way you don't wind up with very different code that
 * matches the version number of a "released" optiboot.
 */

#if !defined(OPTIBOOT_CUSTOMVER)
#define OPTIBOOT_CUSTOMVER 0
#endif

unsigned const int __attribute__ ( ( section ( ".version" ) ) )
optiboot_version = 256* ( OPTIBOOT_MAJVER + OPTIBOOT_CUSTOMVER ) + OPTIBOOT_MINVER;


#include <inttypes.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>

/********************CAN related declarations***********************/
#include "can_libs/mh_can.h"
#include "can_libs/mh_spi.h"
#include "can_libs/mh_j1939.h"
#include "can_libs/flash_utils.h"

#include <avr/wdt.h>
#include <avr/interrupt.h> // Used in functions: boot_program_page

void can_main();

static uint8_t data_buffer[128];


/*******************************************************************/


/*
 * optiboot uses several "address" variables that are sometimes byte pointers,
 * sometimes word pointers. sometimes 16bit quantities, and sometimes built
 * up from 8bit input characters.  avr-gcc is not great at optimizing the
 * assembly of larger words from bytes, but we can use the usual union to
 * do this manually.  Expanding it a little, we can also get rid of casts.
 */
typedef union {
    uint8_t  *bptr;
    uint16_t *wptr;
    uint16_t word;
    uint8_t bytes[2];
} addr16_t;

/*
 * Note that we use a replacement of "boot.h"
 * <avr/boot.h> uses sts instructions, but this version uses out instructions
 * This saves cycles and program memory, if possible.
 * boot_opt.h pulls in the standard boot.h for the odd target (?)
 */
#include "boot_opt.h"


// We don't use <avr/wdt.h> as those routines have interrupt overhead we don't need.

/*
 * pin_defs.h
 * This contains most of the rather ugly defines that implement our
 * ability to use UART=n and LED=D3, and some avr family bit name differences.
 */
#include "pin_defs.h"

/*
 * stk500.h contains the constant definitions for the stk500v1 comm protocol
 */
#include "stk500.h"

#ifndef LED_START_FLASHES
#define LED_START_FLASHES 0
#endif

/* set the UART baud rate defaults */
#ifndef BAUD_RATE
#if F_CPU >= 8000000L
#define BAUD_RATE   115200L // Highest rate Avrdude win32 will support
#elif F_CPU >= 1000000L
#define BAUD_RATE   9600L   // 19200 also supported, but with significant error
#elif F_CPU >= 128000L
#define BAUD_RATE   4800L   // Good for 128kHz internal RC
#else
#define BAUD_RATE 1200L     // Good even at 32768Hz
#endif
#endif

#ifndef UART
#define UART 0
#endif

#ifndef SOFT_UART
#ifdef SINGLESPEED
/* Single speed option */
#define BAUD_SETTING (( (F_CPU + BAUD_RATE * 8L) / ((BAUD_RATE * 16L))) - 1 )
#define BAUD_ACTUAL (F_CPU/(16 * ((BAUD_SETTING)+1)))
#else
/* Normal U2X usage */
#define BAUD_SETTING (( (F_CPU + BAUD_RATE * 4L) / ((BAUD_RATE * 8L))) - 1 )
#define BAUD_ACTUAL (F_CPU/(8 * ((BAUD_SETTING)+1)))
#endif
#if BAUD_ACTUAL <= BAUD_RATE
#define BAUD_ERROR (( 100*(BAUD_RATE - BAUD_ACTUAL) ) / BAUD_RATE)
#if BAUD_ERROR >= 5
#error BAUD_RATE off by greater than -5%
#elif BAUD_ERROR >= 2  && !defined(PRODUCTION)
#warning BAUD_RATE off by greater than -2%
#endif
#else
#define BAUD_ERROR (( 100*(BAUD_ACTUAL - BAUD_RATE) ) / BAUD_RATE)
#if BAUD_ERROR >= 5
#error BAUD_RATE off by greater than 5%
#elif BAUD_ERROR >= 2  && !defined(PRODUCTION)
#warning BAUD_RATE off by greater than 2%
#endif
#endif

#if BAUD_SETTING > 250
#error Unachievable baud rate (too slow) BAUD_RATE
#endif // baud rate slow check
#if (BAUD_SETTING - 1) < 3
#if BAUD_ERROR != 0 // permit high bitrates (ie 1Mbps@16MHz) if error is zero
#error Unachievable baud rate (too fast) BAUD_RATE
#endif
#endif // baud rate fast check
#endif // SOFT_UART

/* Watchdog settings */
#define WATCHDOG_OFF    (0)
#define WATCHDOG_16MS   (_BV(WDE))
#define WATCHDOG_32MS   (_BV(WDP0) | _BV(WDE))
#define WATCHDOG_64MS   (_BV(WDP1) | _BV(WDE))
#define WATCHDOG_125MS  (_BV(WDP1) | _BV(WDP0) | _BV(WDE))
#define WATCHDOG_250MS  (_BV(WDP2) | _BV(WDE))
#define WATCHDOG_500MS  (_BV(WDP2) | _BV(WDP0) | _BV(WDE))
#define WATCHDOG_1S     (_BV(WDP2) | _BV(WDP1) | _BV(WDE))
#define WATCHDOG_2S     (_BV(WDP2) | _BV(WDP1) | _BV(WDP0) | _BV(WDE))
#ifndef __AVR_ATmega8__
#define WATCHDOG_4S     (_BV(WDP3) | _BV(WDE))
#define WATCHDOG_8S     (_BV(WDP3) | _BV(WDP0) | _BV(WDE))
#endif


/*
 * We can never load flash with more than 1 page at a time, so we can save
 * some code space on parts with smaller pagesize by using a smaller int.
 */
#if SPM_PAGESIZE > 255
typedef uint16_t pagelen_t ;
#define GETLENGTH(len) len = getch()<<8; len |= getch()
#else
typedef uint8_t pagelen_t;
#define GETLENGTH(len) (void) getch() /* skip high byte */; len = getch()
#endif


/* Function Prototypes
 * The main() function is in init9, which removes the interrupt vector table
 * we don't need. It is also 'OS_main', which means the compiler does not
 * generate any entry or exit code itself (but unlike 'naked', it doesn't
 * supress some compile-time options we want.)
 */

void pre_main ( void ) __attribute__ ( ( naked ) ) __attribute__ ( ( section ( ".init8" ) ) );
int main ( void ) __attribute__ ( ( OS_main ) ) __attribute__ ( ( section ( ".init9" ) ) ) __attribute__ ( ( used ) );

void __attribute__ ( ( noinline ) ) __attribute__ ( ( leaf ) ) putch ( char );
uint8_t __attribute__ ( ( noinline ) ) __attribute__ ( ( leaf ) ) getch ( void ) ;
void __attribute__ ( ( noinline ) ) verifySpace();
void __attribute__ ( ( noinline ) ) watchdogConfig ( uint8_t x );

static void getNch ( uint8_t );
#if LED_START_FLASHES > 0
static inline void flash_led ( uint8_t );
#endif
static inline void watchdogReset();
static inline void writebuffer ( int8_t memtype, addr16_t mybuff,
                                 addr16_t address, pagelen_t len );
static inline void read_mem ( uint8_t memtype,
                              addr16_t, pagelen_t len );


#ifdef SOFT_UART
void uartDelay() __attribute__ ( ( naked ) );
#endif

/*
 * RAMSTART should be self-explanatory.  It's bigger on parts with a
 * lot of peripheral registers.  Let 0x100 be the default
 * Note that RAMSTART (for optiboot) need not be exactly at the start of RAM.
 */
#if !defined(RAMSTART)  // newer versions of gcc avr-libc define RAMSTART
#define RAMSTART 0x100
#if defined (__AVR_ATmega644P__)
// correct for a bug in avr-libc
#undef SIGNATURE_2
#define SIGNATURE_2 0x0A
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
#undef RAMSTART
#define RAMSTART (0x200)
#endif
#endif

/* C zero initialises all global variables. However, that requires */
/* These definitions are NOT zero initialised, but that doesn't matter */
/* This allows us to drop the zero init code, saving us memory */
static addr16_t buff = { ( uint8_t * ) ( RAMSTART ) };


/* Virtual boot partition support */
#ifdef VIRTUAL_BOOT_PARTITION

// RAM locations to save vector info (temporarilly)
#define rstVect0_sav (*(uint8_t*)(RAMSTART+SPM_PAGESIZE*2+4))
#define rstVect1_sav (*(uint8_t*)(RAMSTART+SPM_PAGESIZE*2+5))
#define saveVect0_sav (*(uint8_t*)(RAMSTART+SPM_PAGESIZE*2+6))
#define saveVect1_sav (*(uint8_t*)(RAMSTART+SPM_PAGESIZE*2+7))

#define RSTVEC_ADDRESS 0  // flash address where vectors start.
// Vector to save original reset jump:
//   SPM Ready is least probably used, so it's default
//   if not, use old way WDT_vect_num,
//   or simply set custom save_vect_num in Makefile using vector name
//   or even raw number.
#if !defined (save_vect_num)
#if defined (SPM_RDY_vect_num)
#define save_vect_num (SPM_RDY_vect_num)
#elif defined (SPM_READY_vect_num)
#define save_vect_num (SPM_READY_vect_num)
#elif defined (EE_RDY_vect_num)
#define save_vect_num (EE_RDY_vect_num)
#elif defined (EE_READY_vect_num)
#define save_vect_num (EE_READY_vect_num)
#elif defined (WDT_vect_num)
#define save_vect_num (WDT_vect_num)
#else
#error Cant find SPM or WDT interrupt vector for this CPU
#endif
#endif //save_vect_num

// check if it's on the same page (code assumes that)

#if FLASHEND > 8192
// AVRs with more than 8k of flash have 4-byte vectors, and use jmp.
//  We save only 16 bits of address, so devices with more than 128KB
//  may behave wrong for upper part of address space.
#define rstVect0 2
#define rstVect1 3
#define saveVect0 (save_vect_num*4+2)
#define saveVect1 (save_vect_num*4+3)
#define appstart_vec (save_vect_num*2)

// address of page that will have the saved vector
#define SAVVEC_ADDRESS (SPM_PAGESIZE*(save_vect_num/(SPM_PAGESIZE/4)))

#else

// AVRs with up to 8k of flash have 2-byte vectors, and use rjmp.
#define rstVect0 0
#define rstVect1 1
#define saveVect0 (save_vect_num*2)
#define saveVect1 (save_vect_num*2+1)
#define appstart_vec (save_vect_num)

// address of page that will have the saved vector
#define SAVVEC_ADDRESS (SPM_PAGESIZE*(save_vect_num/(SPM_PAGESIZE/2)))
#endif

#else

#define appstart_vec (0)

#endif // VIRTUAL_BOOT_PARTITION

/* everything that needs to run VERY early */
void pre_main ( void )
{

    // Allow convenient way of calling do_spm function - jump table,
    //   so entry to this function will always be here, indepedent of compilation,
    //   features etc
    __asm__ volatile (
        "	rjmp	1f\n"
#ifndef APP_NOSPM
        "	rjmp	do_spm\n"
#else
        "   ret\n"   // if do_spm isn't include, return without doing anything
#endif
        "1:\n"
    );
}


/* main program starts here */
int main ( void )
{
    uint8_t ch;


    /*
     * Making these local and in registers prevents the need for initializing
     * them, and also saves space because code no longer stores to memory.
     * (initializing address keeps the compiler happy, but isn't really
     *  necessary, and uses 4 bytes of flash.)
     */
    register addr16_t address;
    register pagelen_t  length;

    // After the zero init loop, this is the first code to run.
    //
    // This code makes the following assumptions:
    //  No interrupts will execute
    //  SP points to RAMEND
    //  r1 contains zero
    //
    // If not, uncomment the following instructions:
    cli();
    __asm__ volatile ( "clr __zero_reg__" );

#if defined(__AVR_ATmega8__) || defined(__AVR_ATmega8515__) ||		\
    defined(__AVR_ATmega8535__) || defined (__AVR_ATmega16__) || 	\
    defined (__AVR_ATmega32__) || defined (__AVR_ATmega64__)  ||	\
    defined (__AVR_ATmega128__) || defined (__AVR_ATmega162__)
    SP=RAMEND;  // This is done by hardware reset
#endif

#if defined(OSCCAL_VALUE)
    OSCCAL = OSCCAL_VALUE;
#endif

    /*
     * Protect as much from MCUSR as possible for application
     * and still skip bootloader if not necessary
     *
     * Code by MarkG55
     * see discusion in https://github.com/Optiboot/optiboot/issues/97
     */
#if defined(__AVR_ATmega8515__) || defined(__AVR_ATmega8535__) ||	\
    defined(__AVR_ATmega16__)   || defined(__AVR_ATmega162__) ||	\
    defined (__AVR_ATmega128__)
    ch = MCUCSR;
#else
    ch = MCUSR;
#endif

    watchdogReset();

    const uint8_t flash_status = eeprom_read_byte((uint8_t*)FLASH_STATUS_ADDR);
    if( (flash_status == 1) | (ch & ( _BV ( PORF ) ) )) {   /* Start bootloader if flash with CAN is active or power-on-reset */
        ch = 0;
    }

    // Skip all logic and run bootloader if MCUSR is cleared (application request)
    if ( ch != 0 ) {
        /*
         * To run the boot loader, External Reset Flag must be set.
         * If not, we could make shortcut and jump directly to application code.
         * Also WDRF set with EXTRF is a result of Optiboot timeout, so we
         * shouldn't run bootloader in loop :-) That's why:
         *  1. application is running if WDRF is cleared
         *  2. we clear WDRF if it's set with EXTRF to avoid loops
         * One problematic scenario: broken application code sets watchdog timer
         * without clearing MCUSR before and triggers it quickly. But it's
         * recoverable by power-on with pushed reset button.
         */
        if ( ( ch & ( _BV ( WDRF ) | _BV ( EXTRF ) ) ) != _BV ( EXTRF ) ) {
            if ( ch & _BV ( EXTRF ) ) {
                /*
                 * Clear WDRF because it was most probably set by wdr in bootloader.
                 * It's also needed to avoid loop by broken application which could
                 * prevent entering bootloader.
                 * '&' operation is skipped to spare few bytes as bits in MCUSR
                 * can only be cleared.
                 */
#if defined(__AVR_ATmega8515__) || defined(__AVR_ATmega8535__) ||	\
    defined(__AVR_ATmega16__)   || defined(__AVR_ATmega162__) ||	\
    defined(__AVR_ATmega128__)
                // Fix missing definitions in avr-libc
                MCUCSR = ~ ( _BV ( WDRF ) );
#else
                MCUSR = ~ ( _BV ( WDRF ) );
#endif
            }
            /*
             * save the reset flags in the designated register
             * This can be saved in a main program by putting code in .init0 (which
             * executes before normal c init code) to save R2 to a global variable.
             */
            __asm__ __volatile__ ( "mov r2, %0\n" :: "r" ( ch ) );

            // switch off watchdog
            watchdogConfig ( WATCHDOG_OFF );
            // Note that appstart_vec is defined so that this works with either
            // real or virtual boot partitions.
            __asm__ __volatile__ (
                // Jump to 'save' or RST vector
#ifdef VIRTUAL_BOOT_PARTITION
                // full code version for virtual boot partition
                "ldi r30,%[rstvec]\n"
                "clr r31\n"
                "ijmp\n"::[rstvec] "M" ( appstart_vec )
#else
#ifdef RAMPZ
                // use absolute jump for devices with lot of flash
                "jmp 0\n"::
#else
                // use rjmp to go around end of flash to address 0
                // it uses fact that optiboot_version constant is 2 bytes before end of flash
                "rjmp optiboot_version+2\n"
#endif //RAMPZ
#endif //VIRTUAL_BOOT_PARTITION
            );
        }
    }


#if LED_START_FLASHES > 0
    // Set up Timer 1 for timeout counter
#if defined(__AVR_ATtiny261__)||defined(__AVR_ATtiny461__)||defined(__AVR_ATtiny861__)
    TCCR1B = 0x0E; //div 8196 - we could divide by less since it's a 10-bit counter, but why?
#elif defined(__AVR_ATtiny25__)||defined(__AVR_ATtiny45__)||defined(__AVR_ATtiny85__)
    TCCR1 = 0x0E; //div 8196 - it's an 8-bit timer.
#elif defined(__AVR_ATtiny43__)
#error "LED flash for Tiny43 not yet supported"
#else
    TCCR1B = _BV ( CS12 ) | _BV ( CS10 ); // div 1024
#endif
#endif


#ifndef SOFT_UART
#if defined(__AVR_ATmega8__) || defined (__AVR_ATmega8515__) ||	\
      defined (__AVR_ATmega8535__) || defined (__AVR_ATmega16__) ||	\
      defined (__AVR_ATmega32__)
#ifndef SINGLESPEED
    UCSRA = _BV ( U2X ); //Double speed mode USART
#endif //singlespeed
    UCSRB = _BV ( RXEN ) | _BV ( TXEN ); // enable Rx & Tx
    UCSRC = _BV ( URSEL ) | _BV ( UCSZ1 ) | _BV ( UCSZ0 ); // config USART; 8N1
    UBRRL = ( uint8_t ) BAUD_SETTING;
#else // mega8/etc
#ifdef LIN_UART
    //DDRB|=3;
    LINCR = ( 1 << LSWRES );
    //LINBRRL = (((F_CPU * 10L / 32L / BAUD_RATE) + 5L) / 10L) - 1;
    LINBRRL= ( uint8_t ) BAUD_SETTING;
    LINBTR = ( 1 << LDISR ) | ( 8 << LBT0 );
    LINCR = _BV ( LENA ) | _BV ( LCMD2 ) | _BV ( LCMD1 ) | _BV ( LCMD0 );
    LINDAT=0;
#else
#ifndef SINGLESPEED
    UART_SRA = _BV ( U2X0 ); //Double speed mode USART0
#endif
    UART_SRB = _BV ( RXEN0 ) | _BV ( TXEN0 );
    UART_SRC = _BV ( UCSZ00 ) | _BV ( UCSZ01 );
    UART_SRL = ( uint8_t ) BAUD_SETTING;
#endif // LIN_UART
#endif // mega8/etc
#endif // soft_uart

    // Set up watchdog to trigger after 2s
    watchdogConfig(WATCHDOG_2S); 

#if (LED_START_FLASHES > 0) || defined(LED_DATA_FLASH) || defined(LED_START_ON)
    /* Set LED pin as output */
    LED_DDR |= _BV ( LED );
#endif

#ifdef SOFT_UART
    /* Set TX pin as output */
    UART_DDR |= _BV ( UART_TX_BIT );
#endif

#if LED_START_FLASHES > 0
    /* Flash onboard LED to signal entering of bootloader */
    flash_led ( LED_START_FLASHES * 2 );
#else
#if defined(LED_START_ON)
    /* Turn on LED to indicate starting bootloader (less code!) */
    LED_PORT |= _BV ( LED );
#endif
#endif

    watchdogReset();

    if( (flash_status == 1) | (MCUSR & ( _BV ( PORF ) ))) {
        if(MCUSR & ( _BV ( PORF ))) {
            MCUSR &= ~(_BV ( PORF ));   /* Clear MCUSR to start app after watchdog timeout (in case of power on reset) */
        }
        can_main();
    }

    /* Forever loop: exits by causing WDT reset */
    for ( ;; ) {
        /* get character from UART */
        ch = getch();

        if ( ch == STK_GET_PARAMETER ) {
            unsigned char which = getch();
            verifySpace();
            /*
             * Send optiboot version as "SW version"
             * Note that the references to memory are optimized away.
             */
            if ( which == STK_SW_MINOR ) {
                putch ( optiboot_version & 0xFF );
            } else if ( which == STK_SW_MAJOR ) {
                putch ( optiboot_version >> 8 );
            } else {
                /*
                 * GET PARAMETER returns a generic 0x03 reply for
                     * other parameters - enough to keep Avrdude happy
                 */
                putch ( 0x03 );
            }
        } else if ( ch == STK_SET_DEVICE ) {
            // SET DEVICE is ignored
            getNch ( 20 );
        } else if ( ch == STK_SET_DEVICE_EXT ) {
            // SET DEVICE EXT is ignored
            getNch ( 5 );
        } else if ( ch == STK_LOAD_ADDRESS ) {
            // LOAD ADDRESS
            address.bytes[0] = getch();
            address.bytes[1] = getch();
#ifdef RAMPZ
            // Transfer top bit to LSB in RAMPZ
            if ( address.bytes[1] & 0x80 ) {
                RAMPZ |= 0x01;
            } else {
                RAMPZ &= 0xFE;
            }
#endif
            address.word *= 2; // Convert from word address to byte address
            verifySpace();
        } else if ( ch == STK_UNIVERSAL ) {
#ifdef RAMPZ
            // LOAD_EXTENDED_ADDRESS is needed in STK_UNIVERSAL for addressing more than 128kB
            if ( AVR_OP_LOAD_EXT_ADDR == getch() ) {
                // get address
                getch();  // get '0'
                RAMPZ = ( RAMPZ & 0x01 ) | ( ( getch() << 1 ) & 0xff ); // get address and put it in RAMPZ
                getNch ( 1 ); // get last '0'
                // response
                putch ( 0x00 );
            } else {
                // everything else is ignored
                getNch ( 3 );
                putch ( 0x00 );
            }
#else
            // UNIVERSAL command is ignored
            getNch ( 4 );
            putch ( 0x00 );
#endif
        }
        /* Write memory, length is big endian and is in bytes */
        else if ( ch == STK_PROG_PAGE ) {
            // PROGRAM PAGE - we support flash programming only, not EEPROM
            uint8_t desttype;
            uint8_t *bufPtr;
            pagelen_t savelength;

            GETLENGTH ( length );
            savelength = length;
            desttype = getch();

            // read a page worth of contents
            bufPtr = buff.bptr;
            do *bufPtr++ = getch();
            while ( --length );

            // Read command terminator, start reply
            verifySpace();

#ifdef VIRTUAL_BOOT_PARTITION
            /*
             *              How the Virtual Boot Partition works:
             * At the beginning of a normal AVR program are a set of vectors that
             * implement the interrupt mechanism.  Each vector is usually a single
             * instruction that dispatches to the appropriate ISR.
             * The instruction is normally an rjmp (on AVRs with 8k or less of flash)
             * or jmp instruction, and the 0th vector is executed on reset and jumps
             * to the start of the user program:
             * vectors: jmp startup
             *          jmp ISR1
             *          jmp ISR2
             *             :      ;; etc
             *          jmp lastvector
             * To implement the "Virtual Boot Partition", Optiboot detects when the
             * flash page containing the vectors is being programmed, and replaces the
             * startup vector with a jump to te beginning of Optiboot.  Then it saves
             * the applications's startup vector in another (must be unused by the
             * application), and finally programs the page with the changed vectors.
             * Thereafter, on reset, the vector will dispatch to the beginning of
             * Optiboot.  When Optiboot decides that it will run the user application,
             * it fetches the saved start address from the unused vector, and jumps
             * there.
             * The logic is dependent on size of flash, and whether the reset vector is
             * on the same flash page as the saved start address.
             */

#if FLASHEND > 8192
            /*
             * AVR with 4-byte ISR Vectors and "jmp"
             * WARNING: this works only up to 128KB flash!
             */
#if FLASHEND > (128*1024)
#error "Can't use VIRTUAL_BOOT_PARTITION with more than 128k of Flash"
#endif
            if ( address.word == RSTVEC_ADDRESS ) {
                // This is the reset vector page. We need to live-patch the
                // code so the bootloader runs first.
                //
                // Save jmp targets (for "Verify")
                rstVect0_sav = buff.bptr[rstVect0];
                rstVect1_sav = buff.bptr[rstVect1];

                // Add "jump to Optiboot" at RESET vector
                // WARNING: this works as long as 'main' is in first section
                buff.bptr[rstVect0] = ( ( uint16_t ) pre_main ) & 0xFF;
                buff.bptr[rstVect1] = ( ( uint16_t ) pre_main ) >> 8;

#if (SAVVEC_ADDRESS != RSTVEC_ADDRESS)
// the save_vector is not necessarilly on the same flash page as the reset
//  vector.  If it isn't, we've waiting to actually write it.
            } else if ( address.word == SAVVEC_ADDRESS ) {
                // Save old values for Verify
                saveVect0_sav = buff.bptr[saveVect0 - SAVVEC_ADDRESS];
                saveVect1_sav = buff.bptr[saveVect1 - SAVVEC_ADDRESS];

                // Move RESET jmp target to 'save' vector
                buff.bptr[saveVect0 - SAVVEC_ADDRESS] = rstVect0_sav;
                buff.bptr[saveVect1 - SAVVEC_ADDRESS] = rstVect1_sav;
            }
#else
                // Save old values for Verify
                saveVect0_sav = buff.bptr[saveVect0];
                saveVect1_sav = buff.bptr[saveVect1];

                // Move RESET jmp target to 'save' vector
                buff.bptr[saveVect0] = rstVect0_sav;
                buff.bptr[saveVect1] = rstVect1_sav;
            }
#endif

#else
            /*
             * AVR with 2-byte ISR Vectors and rjmp
             */
            if ( address.word == rstVect0 ) {
                // This is the reset vector page. We need to live-patch
                // the code so the bootloader runs first.
                //
                // Move RESET vector to 'save' vector
                // Save jmp targets (for "Verify")
                rstVect0_sav = buff.bptr[rstVect0];
                rstVect1_sav = buff.bptr[rstVect1];
                addr16_t vect;
                vect.word = ( ( uint16_t ) pre_main-1 );
                // Instruction is a relative jump (rjmp), so recalculate.
                // an RJMP instruction is 0b1100xxxx xxxxxxxx, so we should be able to
                // do math on the offsets without masking it off first.
                // Note that rjmp is relative to the already incremented PC, so the
                //  offset is one less than you might expect.
                buff.bptr[0] = vect.bytes[0]; // rjmp to start of bootloader
                buff.bptr[1] = vect.bytes[1] | 0xC0;  // make an "rjmp"
#if (SAVVEC_ADDRESS != RSTVEC_ADDRESS)
            } else if ( address.word == SAVVEC_ADDRESS ) {
                addr16_t vect;
                vect.bytes[0] = rstVect0_sav;
                vect.bytes[1] = rstVect1_sav;
                // Save old values for Verify
                saveVect0_sav = buff.bptr[saveVect0 - SAVVEC_ADDRESS];
                saveVect1_sav = buff.bptr[saveVect1 - SAVVEC_ADDRESS];

                vect.word = ( vect.word-save_vect_num ); //substract 'save' interrupt position
                // Move RESET jmp target to 'save' vector
                buff.bptr[saveVect0 - SAVVEC_ADDRESS] = vect.bytes[0];
                buff.bptr[saveVect1 - SAVVEC_ADDRESS] = ( vect.bytes[1] & 0x0F ) | 0xC0; // make an "rjmp"
            }
#else

                // Save old values for Verify
                saveVect0_sav = buff.bptr[saveVect0];
                saveVect1_sav = buff.bptr[saveVect1];

                vect.bytes[0] = rstVect0_sav;
                vect.bytes[1] = rstVect1_sav;
                vect.word = ( vect.word-save_vect_num ); //substract 'save' interrupt position
                // Move RESET jmp target to 'save' vector
                buff.bptr[saveVect0] = vect.bytes[0];
                buff.bptr[saveVect1] = ( vect.bytes[1] & 0x0F ) | 0xC0; // make an "rjmp"
                // Add rjmp to bootloader at RESET vector
                vect.word = ( ( uint16_t ) pre_main-1 ); // (main) is always <= 0x0FFF; no masking needed.
                buff.bptr[0] = vect.bytes[0]; // rjmp 0x1c00 instruction
            }

#endif

#endif // FLASHEND
#endif // VBP

            writebuffer ( desttype, buff, address, savelength );


        }
        /* Read memory block mode, length is big endian.  */
        else if ( ch == STK_READ_PAGE )
        {
            uint8_t desttype;
            GETLENGTH ( length );

            desttype = getch();

            verifySpace();

            read_mem ( desttype, address, length );
        }

        /* Get device signature bytes  */
        else if ( ch == STK_READ_SIGN )
        {
            // READ SIGN - return what Avrdude wants to hear
            verifySpace();
            putch ( SIGNATURE_0 );
            putch ( SIGNATURE_1 );
            putch ( SIGNATURE_2 );
        } else if ( ch == STK_LEAVE_PROGMODE ) /* 'Q' */
        {
            // Adaboot no-wait mod
            watchdogConfig ( WATCHDOG_16MS );
            verifySpace();
        } else
        {
            // This covers the response to commands like STK_ENTER_PROGMODE
            verifySpace();
        }
        putch ( STK_OK );
    }
}

void putch ( char ch )
{
#ifndef SOFT_UART
#ifndef LIN_UART
    while ( ! ( UART_SRA & _BV ( UDRE0 ) ) ) {  /* Spin */ }
#else
    while ( ! ( LINSIR & _BV ( LTXOK ) ) )   {  /* Spin */ }
#endif

    UART_UDR = ch;

#else
    __asm__ __volatile__ (
        "   com %[ch]\n" // ones complement, carry set
        "   sec\n"
        "1: brcc 2f\n"
        "   cbi %[uartPort],%[uartBit]\n"
        "   rjmp 3f\n"
        "2: sbi %[uartPort],%[uartBit]\n"
        "   nop\n"
        "3: rcall uartDelay\n"
        "   rcall uartDelay\n"
        "   lsr %[ch]\n"
        "   dec %[bitcnt]\n"
        "   brne 1b\n"
        :
        :
        [bitcnt] "d" ( 10 ),
        [ch] "r" ( ch ),
        [uartPort] "I" ( _SFR_IO_ADDR ( UART_PORT ) ),
        [uartBit] "I" ( UART_TX_BIT )
        :
        "r25"
    );
#endif
}

uint8_t getch ( void )
{
    uint8_t ch;

#ifdef LED_DATA_FLASH
#if defined(__AVR_ATmega8__)    || defined(__AVR_ATmega8515__) ||	\
    defined(__AVR_ATmega8535__) || defined(__AVR_ATmega16__)   ||	\
    defined(__AVR_ATmega162__)  || defined(__AVR_ATmega32__)   ||	\
    defined(__AVR_ATmega64__)   || defined(__AVR_ATmega128__)
    LED_PORT ^= _BV ( LED );
#else
    LED_PIN |= _BV ( LED );
#endif
#endif

#ifdef SOFT_UART
    watchdogReset();
    __asm__ __volatile__ (
        "1: sbic  %[uartPin],%[uartBit]\n"  // Wait for start edge
        "   rjmp  1b\n"
        "   rcall uartDelay\n"          // Get to middle of start bit
        "2: rcall uartDelay\n"              // Wait 1 bit period
        "   rcall uartDelay\n"              // Wait 1 bit period
        "   clc\n"
        "   sbic  %[uartPin],%[uartBit]\n"
        "   sec\n"
        "   dec   %[bitCnt]\n"
        "   breq  3f\n"
        "   ror   %[ch]\n"
        "   rjmp  2b\n"
        "3:\n"
        :
        [ch] "=r" ( ch )
        :
        [bitCnt] "d" ( 9 ),
        [uartPin] "I" ( _SFR_IO_ADDR ( UART_PIN ) ),
        [uartBit] "I" ( UART_RX_BIT )
        :
        "r25"
    );
#else
#ifndef LIN_UART
    while ( ! ( UART_SRA & _BV ( RXC0 ) ) )  {  /* Spin */ }
    if ( ! ( UART_SRA & _BV ( FE0 ) ) ) {
#else
    while ( ! ( LINSIR & _BV ( LRXOK ) ) )  {  /* Spin */ }
    if ( ! ( LINSIR & _BV ( LFERR ) ) ) {
#endif
        /*
         * A Framing Error indicates (probably) that something is talking
         * to us at the wrong bit rate.  Assume that this is because it
         * expects to be talking to the application, and DON'T reset the
         * watchdog.  This should cause the bootloader to abort and run
         * the application "soon", if it keeps happening.  (Note that we
         * don't care that an invalid char is returned...)
         */
        watchdogReset();
    }

    ch = UART_UDR;
#endif

#ifdef LED_DATA_FLASH
#if defined(__AVR_ATmega8__)    || defined(__AVR_ATmega8515__) ||	\
    defined(__AVR_ATmega8535__) || defined(__AVR_ATmega16__)   ||	\
    defined(__AVR_ATmega162__)  || defined(__AVR_ATmega32__)   ||	\
    defined(__AVR_ATmega64__)   || defined(__AVR_ATmega128__)
    LED_PORT ^= _BV ( LED );
#else
    LED_PIN |= _BV ( LED );
#endif
#endif

    return ch;
}

#ifdef SOFT_UART
// AVR305 equation: #define UART_B_VALUE (((F_CPU/BAUD_RATE)-23)/6)
// Adding 3 to numerator simulates nearest rounding for more accurate baud rates
#define UART_B_VALUE (((F_CPU/BAUD_RATE)-20)/6)
#if UART_B_VALUE > 255
#error Baud rate too slow for soft UART
#endif

#if UART_B_VALUE < 6
// (this value is a "guess" at when loop/call overhead might become too
//  significant for the soft uart to work.  It tests OK with the popular
//  "ATtinycore" chips that need SOFT_UART, at the usual clock/baud combos.)
#error Baud rate too high for soft UART
#endif


void uartDelay()
{
    __asm__ __volatile__ (
        "ldi r25,%[count]\n"
        "1:dec r25\n"
        "brne 1b\n"
        "ret\n"
        ::[count] "M" ( UART_B_VALUE )
    );
}
#endif

void getNch ( uint8_t count )
{
    do getch();
    while ( --count );
    verifySpace();
}

void verifySpace()
{
    if ( getch() != CRC_EOP ) {
        watchdogConfig ( WATCHDOG_16MS ); // shorten WD timeout
        while ( 1 )			    // and busy-loop so that WD causes
            ;				      //  a reset and app start.
    }
    putch ( STK_INSYNC );
}

#if LED_START_FLASHES > 0
void flash_led ( uint8_t count )
{
    do {
#if defined(__AVR_ATtiny261__)||defined(__AVR_ATtiny461__)||defined(__AVR_ATtiny861__) || defined(__AVR_ATtiny25__)||defined(__AVR_ATtiny45__)||defined(__AVR_ATtiny85__)
        TCNT1 = - ( F_CPU/ ( 8196*16 ) );
        TIFR = _BV ( TOV1 );
        while ( ! ( TIFR & _BV ( TOV1 ) ) );
#elif defined(__AVR_ATtiny43__)
#error "LED flash for Tiny43 not yet supported"
#else
        TCNT1 = - ( F_CPU/ ( 1024*16 ) );
        TIFR1 = _BV ( TOV1 );
        while ( ! ( TIFR1 & _BV ( TOV1 ) ) );
#endif

#if defined(__AVR_ATmega8__)    || defined(__AVR_ATmega8515__) ||	\
    defined(__AVR_ATmega8535__) || defined(__AVR_ATmega16__)   ||	\
    defined(__AVR_ATmega162__)  || defined(__AVR_ATmega32__)   ||	\
    defined(__AVR_ATmega64__)   || defined(__AVR_ATmega128__)
        LED_PORT ^= _BV ( LED );
#else
        LED_PIN |= _BV ( LED );
#endif
        watchdogReset();
#ifndef SOFT_UART
        /*
         * While in theory, the STK500 initial commands would be buffered
         *  by the UART hardware, avrdude sends several attempts in rather
         *  quick succession, some of which will be lost and cause us to
         *  get out of sync.  So if we see any data; stop blinking.
         */
#ifndef LIN_UART
        if ( UART_SRA & _BV ( RXC0 ) )
#else
        if ( LINSIR & _BV ( LRXOK ) )
#endif
            break;
#else
// This doesn't seem to work?
//    if ((UART_PIN & (1<<UART_RX_BIT)) == 0)
//	break;  // detect start bit on soft uart too.
#endif
    } while ( --count );
}
#endif

// Watchdog functions. These are only safe with interrupts turned off.
void watchdogReset()
{
    __asm__ __volatile__ (
        "wdr\n"
    );
}

void watchdogConfig ( uint8_t x )
{
#ifdef WDCE //does it have a Watchdog Change Enable?
#ifdef WDTCSR
    WDTCSR = _BV ( WDCE ) | _BV ( WDE );
#else
    WDTCR= _BV ( WDCE ) | _BV ( WDE );
#endif
#else //then it must be one of those newfangled ones that use CCP
    CCP=0xD8; //so write this magic number to CCP
#endif

#ifdef WDTCSR
    WDTCSR = x;
#else
    WDTCR= x;
#endif
}


/*
 * void writebuffer(memtype, buffer, address, length)
 */
static inline void writebuffer ( int8_t memtype, addr16_t mybuff,
                                 addr16_t address, pagelen_t len )
{
    switch ( memtype ) {
    case 'E': // EEPROM
#if defined(SUPPORT_EEPROM) || defined(BIGBOOT)
        while ( len-- ) {
            eeprom_write_byte ( ( address.bptr++ ), * ( mybuff.bptr++ ) );
        }
#else
        /*
         * On systems where EEPROM write is not supported, just busy-loop
         * until the WDT expires, which will eventually cause an error on
         * host system (which is what it should do.)
         */
        while ( 1 )
            ; // Error: wait for WDT
#endif
        break;
    default:  // FLASH
        /*
         * Default to writing to Flash program memory.  By making this
         * the default rather than checking for the correct code, we save
         * space on chips that don't support any other memory types.
         */
    {
        // Copy buffer into programming buffer
        uint16_t addrPtr = address.word;

        /*
         * Start the page erase and wait for it to finish.  There
         * used to be code to do this while receiving the data over
         * the serial link, but the performance improvement was slight,
         * and we needed the space back.
         */
#ifdef FOURPAGEERASE
        if ( ( address.bytes[0] & ( ( SPM_PAGESIZE<<2 )-1 ) ) ==0 ) {
#endif
            __boot_page_erase_short ( address.word );
            boot_spm_busy_wait();
#ifdef FOURPAGEERASE
        }
#endif

        /*
         * Copy data from the buffer into the flash write buffer.
         */
        do {
            __boot_page_fill_short ( ( uint16_t ) ( void* ) addrPtr, * ( mybuff.wptr++ ) );
            addrPtr += 2;
        } while ( len -= 2 );

        /*
         * Actually Write the buffer to flash (and wait for it to finish.)
         */
        __boot_page_write_short ( address.word );
        boot_spm_busy_wait();
#if defined(RWWSRE)
        // Reenable read access to flash
        __boot_rww_enable_short();
#endif
    } // default block
    break;
    } // switch
}

static inline void read_mem ( uint8_t memtype, addr16_t address, pagelen_t length )
{
    uint8_t ch;

    switch ( memtype ) {

#if defined(SUPPORT_EEPROM) || defined(BIGBOOT)
    case 'E': // EEPROM
        do {
            putch ( eeprom_read_byte ( ( address.bptr++ ) ) );
        } while ( --length );
        break;
#endif
    default:
        do {
#ifdef VIRTUAL_BOOT_PARTITION
            // Undo vector patch in bottom page so verify passes
            if ( address.word == rstVect0 ) ch = rstVect0_sav;
            else if ( address.word == rstVect1 ) ch = rstVect1_sav;
            else if ( address.word == saveVect0 ) ch = saveVect0_sav;
            else if ( address.word == saveVect1 ) ch = saveVect1_sav;
            else ch = pgm_read_byte_near ( address.bptr );
            address.bptr++;
#elif defined(RAMPZ)
            // Since RAMPZ should already be set, we need to use EPLM directly.
            // Also, we can use the autoincrement version of lpm to update "address"
            //      do putch(pgm_read_byte_near(address++));
            //      while (--length);
            // read a Flash and increment the address (may increment RAMPZ)
            __asm__ ( "elpm %0,Z+\n" : "=r" ( ch ), "=z" ( address.bptr ) : "1" ( address ) );
#else
            // read a Flash byte and increment the address
            __asm__ ( "lpm %0,Z+\n" : "=r" ( ch ), "=z" ( address.bptr ) : "1" ( address ) );
#endif
            putch ( ch );
        } while ( --length );
        break;
    } // switch
}


#ifndef APP_NOSPM

/*
 * Separate function for doing spm stuff
 * It's needed for application to do SPM, as SPM instruction works only
 * from bootloader.
 *
 * How it works:
 * - do SPM
 * - wait for SPM to complete
 * - if chip have RWW/NRWW sections it does additionaly:
 *   - if command is WRITE or ERASE, AND data=0 then reenable RWW section
 *
 * In short:
 * If you play erase-fill-write, just set data to 0 in ERASE and WRITE
 * If you are brave, you have your code just below bootloader in NRWW section
 *   you could do fill-erase-write sequence with data!=0 in ERASE and
 *   data=0 in WRITE
 */
static void do_spm ( uint16_t address, uint8_t command, uint16_t data )  __attribute__ ( ( used ) );
static void do_spm ( uint16_t address, uint8_t command, uint16_t data )
{
    // Do spm stuff
    __asm__ volatile (
        "    movw  r0, %3\n"
        "    __wr_spmcsr %0, %1\n"
        "    spm\n"
        "    clr  r1\n"
        :
        : "i" ( _SFR_MEM_ADDR ( __SPM_REG ) ),
        "r" ( ( uint8_t ) command ),
        "z" ( ( uint16_t ) address ),
        "r" ( ( uint16_t ) data )
        : "r0"
    );

    // wait for spm to complete
    //   it doesn't have much sense for __BOOT_PAGE_FILL,
    //   but it doesn't hurt and saves some bytes on 'if'
    boot_spm_busy_wait();
#if defined(RWWSRE)
    // this 'if' condition should be: (command == __BOOT_PAGE_WRITE || command == __BOOT_PAGE_ERASE)...
    // but it's tweaked a little assuming that in every command we are interested in here, there
    // must be also SELFPRGEN set. If we skip checking this bit, we save here 4B
    if ( ( command & ( _BV ( PGWRT ) |_BV ( PGERS ) ) ) && ( data == 0 ) ) {
        // Reenable read access to flash
        __boot_rww_enable_short();
    }
#endif
}
#endif



#ifdef BIGBOOT
/*
 * Optiboot is designed to fit in 512 bytes, with a minimum feature set.
 * Some chips have a minimum bootloader size of 1024 bytes, and sometimes
 * it is desirable to add extra features even though 512bytes is exceedded.
 * In that case, the BIGBOOT can be used.
 * Our extra features so far don't come close to filling 1k, so we can
 * add extra "frivolous" data to the image.   In particular, we can add
 * information about how Optiboot was built (which options were selected,
 * what version, all in human-readable form (and extractable from the
 * binary with avr-strings.)
 *
 * This can always be removed or trimmed if more actual program space
 * is needed in the future.  Currently the data occupies about 160 bytes,
 */
#define xstr(s) str(s)
#define str(s) #s
#define OPTFLASHSECT __attribute__((section(".fini8")))
#define OPT2FLASH(o) OPTFLASHSECT const char f##o[] = #o "=" xstr(o)


#ifdef LED_START_FLASHES
OPT2FLASH ( LED_START_FLASHES );
#endif
#ifdef LED_DATA_FLASH
OPT2FLASH ( LED_DATA_FLASH );
#endif
#ifdef LED_START_ON
OPT2FLASH ( LED_START_ON );
#endif
#ifdef LED_NAME
OPTFLASHSECT const char f_LED[] = "LED=" LED_NAME;
#endif

#ifdef SUPPORT_EEPROM
OPT2FLASH ( SUPPORT_EEPROM );
#endif
#ifdef BAUD_RATE
OPT2FLASH ( BAUD_RATE );
#endif
#ifdef SOFT_UART
OPT2FLASH ( SOFT_UART );
#endif
#ifdef UART
OPT2FLASH ( UART );
#endif

OPTFLASHSECT const char f_date[] = "Built:" __DATE__ ":" __TIME__;
#ifdef BIGBOOT
OPT2FLASH ( BIGBOOT );
#endif
#ifdef VIRTUAL_BOOT_PARTITION
OPTFLASHSECT const char f_boot[] = "Virtual_Boot_Partition";
#endif
OPT2FLASH ( F_CPU );
OPTFLASHSECT const char f_device[] = "Device=" xstr ( __AVR_DEVICE_NAME__ );
#ifdef OPTIBOOT_CUSTOMVER
OPT2FLASH ( OPTIBOOT_CUSTOMVER );
#endif
OPTFLASHSECT const char f_version[] = "Version=" xstr ( OPTIBOOT_MAJVER ) "." xstr ( OPTIBOOT_MINVER );

#endif


void can_main()
{
    watchdogReset();

    const uint8_t can_baudrate = eeprom_read_byte((const uint8_t*)CAN_BAUDRATE_ADDR);
    const uint8_t node_address = eeprom_read_byte((const uint8_t*)J1939_SA_ADDR);

    mcp2515_initialize_module(node_address, can_baudrate);

    tp_cm_session_t tp_session;

    uint16_t firmware_size;

    union j1939_pdu message;
    union j1939_pdu rts_receipt;

    watchdogReset();

    for ( ;; ) {
        if ( mcp2515_read_message ( &message.can ) == ERROR_OK) { // No handeling for RTR or ERROR frames

            if ( message.j1939.pdu_format == PDU_FORMAT_FLASH ) {   //Proprietary A
                watchdogReset();

                switch(message.j1939.data[0]) {
                case PA_FLASH_REQUEST: {
                    create_ack(&message, CB_ACK, node_address);
                    mcp2515_send_message(&message.can);
                    break;
                }
                case PA_FLASH_INFO: {

                    mcp2515_bitmodify_register( MCP_RXB0CTRL,
                                                RXBnCTRL_RXM_MASK | RXB0CTRL_BUKT | RXB0CTRL_FILHIT_MASK,
                                                RXBnCTRL_RXM_STDEXT | RXB0CTRL_BUKT | RXB0CTRL_FILHIT );

                    // Filter and mask for RXB0
                    mcp2515_set_acceptance_mask( MASK0, true, MASK_EFF_MCU );              // J1939 filter
                    mcp2515_set_acceptance_filter( RXF0, true, (FILTER_EFF_MCU | (node_address << 8) | message.j1939.source_address));   // J1939 filter

                    mcp2515_set_operation_mode( CANCTRL_REQOP_NORMAL );

                    firmware_size = (message.can.data[2] << 8) | message.can.data[1];
                    create_ack(&message, CB_ACK, node_address);
                    mcp2515_send_message(&message.can);
                    break;
                }
                case PA_FLASH_VERIFY: {
                    uint32_t mcu_crc = 0;

                    for (int i = 0; i < firmware_size; i += SPM_PAGESIZE)
                    {
                        if (i + SPM_PAGESIZE < firmware_size)
                        {
                            memcpy_P(data_buffer, (void*)i, SPM_PAGESIZE);
                            mcu_crc = crc32c(mcu_crc, data_buffer, SPM_PAGESIZE);
                        }
                        else
                        {
                            memcpy_P(data_buffer, (void*)i, firmware_size - i);
                            mcu_crc = crc32c(mcu_crc, data_buffer, firmware_size - i);
                        }
                    }

                    if( message.j1939.data[4] == (mcu_crc & 0xFF) &&
                            message.j1939.data[5] == ((mcu_crc & 0xFF00) >> 8) &&
                            message.j1939.data[6] == ((mcu_crc & 0xFF0000) >> 16) &&
                            message.j1939.data[7] == ((mcu_crc & 0xFF000000) >> 24) ) {
                        create_ack(&message, CB_ACK, node_address);
                    } else {
                        create_ack(&message, CB_NACK, node_address);
                    }
                    mcp2515_send_message(&message.can);
                    break;
                }
                case PA_FLASH_DONE: {
                    create_ack(&message, CB_NACK, node_address);
                    eeprom_update_byte((uint8_t*)FLASH_STATUS_ADDR, FLASH_INACTIVE);
                    MCUSR = 0;
                    watchdogConfig(WATCHDOG_16MS);
                    while ( 1 );
                }
                default:
                    break;
                }

            }
            else if ( message.j1939.pdu_format == PDU_FORMAT_TP_CM ) {
                watchdogReset();
                if ( message.j1939.data[0] == TP_CM_RTS ) {

                    rts_receipt = message;     /* New connection, save for end of message ack */

                    tp_session.bytes_receieved = 0;
                    tp_session.packet_count = message.j1939.data[3];
                    tp_session.message_size = message.j1939.data[2] << 8 | message.j1939.data[1];
                    tp_session.sequence_number = 1;

                    create_cts(&message, &tp_session, node_address);
                    mcp2515_send_message ( &message.can );
                }
            } else if ( message.j1939.pdu_format == PDU_FORMAT_TP_DT ) { // 235 = TP.DT
                watchdogReset();
                uint8_t i = 1;
                if ( message.j1939.data[0] == 1 ) {
                    tp_session.page_address = ( message.j1939.data[2] << 8 ) | message.j1939.data[1];
                    i = 3;
                }
                for ( ; i < message.j1939.dlc; i++ ) {
                    data_buffer[tp_session.bytes_receieved++] = message.j1939.data[i];
                    if ( tp_session.bytes_receieved == ( tp_session.message_size - 2 ) ) {
                        break;
                    }
                }
                if ( message.j1939.data[0] == tp_session.packet_count ) {
                    rts_receipt.j1939.data[0] = TP_CM_EndOfMsgACK;
                    rts_receipt.j1939.pdu_specific = rts_receipt.j1939.source_address;
                    rts_receipt.j1939.source_address = node_address;
                    mcp2515_send_message ( &rts_receipt.can );
                    watchdogReset();
                    boot_program_page ( tp_session.page_address, data_buffer, tp_session.bytes_receieved );
                } else {
                    tp_session.sequence_number++;
                    create_cts(&message, &tp_session, node_address);
                    mcp2515_send_message ( &message.can );
                }
            }
        }
    }
}

