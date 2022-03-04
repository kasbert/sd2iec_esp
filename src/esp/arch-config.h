/* sd2iec - SD/MMC to Commodore serial bus interface/controller
   Copyright (C) 2022 Jarkko Sonninen <kasper@iki.fi>
   Copyright (C) 2007-2017  Ingo Korb <ingo@akana.de>

   Inspired by MMC2IEC by Lars Pontoppidan et al.

   FAT filesystem access based on code from ChaN and Jim Brain, see ff.c|h.

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; version 2 of the License only.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA


   arch-config.h: The main architecture-specific config header

*/

#ifndef ARCH_CONFIG_H
#define ARCH_CONFIG_H

#include <stdint.h>
#include <stdbool.h>
#define uint32 uint32_t
#undef bool
#undef false
#undef true
#undef BOOL
#undef FALSE
#undef TRUE
#include <ets_sys.h>
#include <driver/gpio16.h>
#define gpio_pin_wakeup_disable() gpio_pin_wakeup_disable(void)
#include <gpio.h>
#undef BOOL
#undef FALSE
#undef TRUE
#include <osapi.h>

void system_soft_wdt_feed(void);

// Functions (.text.*) and literals (.literal.*) go to flash by default
// .text and .literal goes to ram.
// see eagle.app.v6.ld linker script for details
#define IRAM_ATTR __attribute__((section(".text")))

#define _BV(x) (1<<(x))

#define _USE_IOCTL 0
#define _READONLY 0
#define _USE_LESS_BUF 0
#define _MCU_ENDIAN 2

#if CONFIG_HARDWARE_VARIANT==1

/*** Device address selection ***/
// device_hw_address() returns the hardware-selected device address
static inline uint8_t device_hw_address(void)
{
	// OPTIONAL: Attach and implement a 2-button DIP-switch.
	// If not we return always a hard coded number.
	return 8;
	// return 8 + not(PIND bitand _BV(PD7)) + 2 * not(PIND bitand _BV(PD5));
}

/* Configure hardware device address pins */
static inline void device_hw_address_init(void)
{
}


/*** LEDs ***/
// Please don't build single-LED hardware anymore...

#define SINGLE_LED

#define PIN_LED_BUILDIN 2

//#define PIN_LED_BUSY   25
//#define PIN_LED_DIRTY   26

// Initialize ports for all LEDs
//
static inline void leds_init(void)
{
  PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO2_U, FUNC_GPIO2); // Use as GPIO2.
}

extern int _led_state;
IRAM_ATTR
static inline void set_led(int state)
{
  if (state)
  {
      //Set GPIO2 to LOW - TURN ON the LED
      GPIO_OUTPUT_SET(PIN_LED_BUILDIN, 0);
  }
  else
  {
      //Set GPIO2 to HIGH - TURN OFF the LED
      GPIO_OUTPUT_SET(PIN_LED_BUILDIN, 1);
  }
  _led_state = state;
}

IRAM_ATTR
static inline void toggle_led(void)
{
  set_led(!_led_state);
}

/*
// --- "BUSY" led, recommended color: green (usage similiar to 1541 LED) ---
static inline void set_busy_led(uint8_t state)
{
  //GPIO_OUTPUT_SET(PIN_LED_BUSY, state);
}

// --- "DIRTY" led, recommended color: red (errors, unwritten data in memory) ---
static inline void set_dirty_led(uint8_t state)
{
  //GPIO_OUTPUT_SET(PIN_LED_DIRTY, state);
}

// Toggle function used for error blinking
static inline void toggle_dirty_led(void)
{
    if (GPIO_INPUT_GET(PIN_LED_DIRTY))
    {
        //Set GPIO2 to LOW - TURN OFF the LED
        GPIO_OUTPUT_SET(PIN_LED_DIRTY, 0);
    }
    else
    {
        //Set GPIO2 to HIGH - TURN ON the LED
        GPIO_OUTPUT_SET(PIN_LED_DIRTY, 0);
    }
}
*/


/*** IEC signals ***/

  // GPIO5 = SCL - IEC CLOCK
  // GPIO4 = SDA - IEC DATA
  // GPIO0 - ATN (pulled up, connected to FLASH button, boot fails if pulled LOW)
  // GPIO16 - SRQ (no interrupt, HIGH at boot)
  // GPIO2 - Internal LED (pulled up, LED on low)
  // GPIO3 = RX - Button 1 ?
  // GPIO1 = TX - LED 2 ?
  // ADC0 - Button 2 ?
  // GPIO 12,13,14,15 SD card

// Pins assigned for the IEC lines
#define IEC_PIN_ATN   0
#define IEC_PIN_DATA  4
#define IEC_PIN_CLOCK 5
//#define IEC_PIN_SRQ   16

/*** IEEE signals ***/
// not documented yet, look at petSD/XS-1541 for guidance

/*** User interface ***/
// Not implemeted in current hardware
/* Button NEXT changes to the next disk image and enables sleep mode (held) */
#  define BUTTON_NEXT _BV(0)

/* Button PREV changes to the previous disk image */
#  define BUTTON_PREV _BV(1)

/* Return value of buttons_read() */
typedef unsigned int rawbutton_t;

// Read the raw button state - a depressed button should read as 0
static inline rawbutton_t buttons_read(void)
{
	// OPTIONAL: Attach/Implement buttons on some GPIO.
  // Not implemented
	return BUTTON_NEXT | BUTTON_PREV;
}

static inline void buttons_init(void)
{
}

#define set_button_irq(x) // TODO

/*** board-specific initialisation ***/
#define HAVE_BOARD_INIT
static inline void board_init(void)
{
}

#else
#  error "CONFIG_HARDWARE_VARIANT is unset or set to an unknown value."
#endif


/* ---------------- End of user-configurable options ---------------- */

#ifdef CONFIG_ADD_SD
// Initialize all pins and interrupts related to SD - except SPI
static inline void sdcard_interface_init(void)
{
}

// sdcard_detect() must return non-zero while a card is inserted
// This must be a pin capable of generating interrupts.
static inline uint8_t sdcard_detect(void)
{
	return true;
}

// Returns non-zero when the currently inserted card is write-protected
static inline uint8_t sdcard_wp(void)
{
	// OPTIONAL: Add write protect pin for sd card, if the shield has such.
	// Otherwise simply return false (fake always write enabled).
	return false;
}
#endif


#if !defined(CONFIG_HAVE_IEC) && !defined(CONFIG_HAVE_IEEE)
#  error Need CONFIG_HAVE_IEC and/or CONFIG_HAVE_IEEE
// Please edit your config-<devicename> if this error occurs.
#endif

#if defined(CONFIG_HAVE_IEC) && defined(CONFIG_HAVE_IEEE)
#  error Sorry, dual-interface devices must select only one interface at compile time!
#endif


/* --- IEC --- */
#ifdef CONFIG_HAVE_IEC

/* Return type of iec_bus_read() */
typedef uint8_t iec_bus_t;

// Override iec_bus.h
#define IEC_BUS_H

#define IEC_BIT_ATN      _BV(IEC_PIN_ATN)
#define IEC_BIT_DATA     _BV(IEC_PIN_DATA)
#define IEC_BIT_CLOCK    _BV(IEC_PIN_CLOCK)
//#define IEC_BIT_SRQ      _BV(IEC_PIN_SRQ)

/* IEC output functions */
#ifdef IEC_OUTPUTS_INVERTED
#  define COND_INV(x) (!(x))
#define IEC_INPUT (gpio_input_get()^0xffff)
#else
#  define COND_INV(x) (x)
#define IEC_INPUT gpio_input_get()
#endif

#define IEC_ATN   (IEC_INPUT & IEC_BIT_ATN)
#define IEC_CLOCK (IEC_INPUT & IEC_BIT_CLOCK)
#define IEC_DATA  (IEC_INPUT & IEC_BIT_DATA)
//#define IEC_SRQ   (IEC_INPUT & IEC_BIT_SRQ)

void iec_interrupts_init(void);

static inline iec_bus_t iec_bus_read(void) {
  // TODO IEC_OUTPUTS_INVERTED
  // TODO | (gpio16_input_get()<<16);
  return IEC_INPUT & (IEC_BIT_ATN | IEC_BIT_DATA | IEC_BIT_CLOCK /*| IEC_BIT_SRQ*/);
}

#ifdef CONFIG_DEBUG_VERBOSE
extern uint8_t atn_state, clock_state, data_state;
#endif

static inline __attribute__((always_inline)) void set_atn(uint8_t state) {
#ifdef CONFIG_DEBUG_VERBOSE
  atn_state = state;
#endif
  if (COND_INV(state)) {
    // Set as input (with external pull up)
    //gpio_output_set(0, 0, 0, IEC_BIT_ATN);
    GPIO_REG_WRITE(GPIO_ENABLE_W1TC_ADDRESS, IEC_BIT_ATN); // GPIO input
    GPIO_REG_WRITE(GPIO_OUT_W1TS_ADDRESS, IEC_BIT_ATN);
  } else {
    // Set as low-level output.
    //gpio_output_set(0, IEC_BIT_ATN, IEC_BIT_ATN, 0);
    GPIO_REG_WRITE(GPIO_ENABLE_W1TS_ADDRESS, IEC_BIT_ATN); // GPIO output
    GPIO_REG_WRITE(GPIO_OUT_W1TC_ADDRESS, IEC_BIT_ATN);
  }
}

static inline __attribute__((always_inline)) void set_data(uint8_t state) {
#ifdef CONFIG_DEBUG_VERBOSE
  data_state = state;
#endif
  if (COND_INV(state)) {
    // Set as input (with external pull up)
    //gpio_output_set(0, 0, 0, IEC_BIT_DATA);
    GPIO_REG_WRITE(GPIO_ENABLE_W1TC_ADDRESS, IEC_BIT_DATA); // GPIO input
    GPIO_REG_WRITE(GPIO_OUT_W1TS_ADDRESS, IEC_BIT_DATA);
  } else {
    // Set as low-level output.
    //gpio_output_set(0, IEC_BIT_DATA, IEC_BIT_DATA, 0);
    GPIO_REG_WRITE(GPIO_ENABLE_W1TS_ADDRESS, IEC_BIT_DATA); // GPIO output
    GPIO_REG_WRITE(GPIO_OUT_W1TC_ADDRESS, IEC_BIT_DATA);
  }
}

static inline __attribute__((always_inline)) void set_clock(uint8_t state) {
#ifdef CONFIG_DEBUG_VERBOSE
  clock_state = state;
#endif
  if (COND_INV(state)) {
    // Set as input (with external pull up)
    //gpio_output_set(0, 0, 0, IEC_BIT_CLOCK);
    GPIO_REG_WRITE(GPIO_ENABLE_W1TC_ADDRESS, IEC_BIT_CLOCK); // GPIO input
    GPIO_REG_WRITE(GPIO_OUT_W1TS_ADDRESS, IEC_BIT_CLOCK);
  } else {
    // Set as low-level output.
    //gpio_output_set(0, IEC_BIT_CLOCK, IEC_BIT_CLOCK, 0);
    GPIO_REG_WRITE(GPIO_ENABLE_W1TS_ADDRESS, IEC_BIT_CLOCK); // GPIO output
    GPIO_REG_WRITE(GPIO_OUT_W1TC_ADDRESS, IEC_BIT_CLOCK);
  }
}

static inline __attribute__((always_inline)) void set_srq(uint8_t state) {
  if (COND_INV(state)) {
    gpio16_input_conf();
  } else {
    gpio16_output_conf();
    gpio16_output_set(0);
  }
}

#endif /* CONFIG_HAVE_IEC */


/* Interrupt handler for system tick */
#define SYSTEM_TICK_HANDLER IRAM_ATTR void systick_handler(void)

/* The assembler module needs the vector names, */
/* so the _HANDLER macros are created here.     */
#define SD_CHANGE_HANDLER  IRAM_ATTR void sdcard_change_handler(void)
#define IEC_ATN_HANDLER    IRAM_ATTR void iec_atn_handler(void)
#define IEC_CLOCK_HANDLER  IRAM_ATTR void iec_clock_handler(void)
#ifdef PARALLEL_ENABLED
#define PARALLEL_HANDLER   IRAM_ATTR void parallel_handler(void)
#endif

/* Enable/disable ATN interrupt */
static inline void set_atn_irq(uint8_t state) {
  //if (state)
  //  GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, 1);
  gpio_pin_intr_state_set(IEC_PIN_ATN, state ? GPIO_PIN_INTR_ANYEDGE : GPIO_PIN_INTR_DISABLE);
}

/* Enable/disable CLOCK interrupt */
/*
#define HAVE_CLOCK_IRQ
static inline void set_clock_irq(uint8_t state) {
  gpio_pin_intr_state_set(IEC_PIN_CLOCK, state ? GPIO_PIN_INTR_NEGEDGE : GPIO_PIN_INTR_DISABLE);
}
*/

/* Display interrupt pin */
#ifdef CONFIG_REMOTE_DISPLAY
static inline void display_intrq_init(void) {
  /* Enable pullup on the interrupt line */
  // TODO
}

static inline uint8_t display_intrq_active(void) {
  // TODO
}
#endif


// Minimal ctype functions
// Prevent undefined reference to __locale_ctype_ptr
#undef toupper
#define toupper(x) ((x)&~0x20)
#undef tolower
#define tolower(x) ((x)|0x20)
#undef isdigit
#define isdigit(x) ((x)>= '0' && (x) <= '9')
#undef isalnum
#define isalnum(c) \
    ( ( (c) >= '0' && (c) <= '9' ) ||  \
    ( (c) >= 'A' && (c) <= 'Z' ) ||  \
    ( (c) >= 'a' && (c) <= 'z' ) )


#if ((CONFIG_SPI_FLASH_SIZE_MAP == 0) || (CONFIG_SPI_FLASH_SIZE_MAP == 1))
#error "The flash map is not supported"
//"    0= 512KB( 256KB+ 256KB)"
//"    2=1024KB( 512KB+ 512KB)"
//"    3=2048KB( 512KB+ 512KB)"
//" *  4=4096KB( 512KB+ 512KB)"
//"    5=2048KB(1024KB+1024KB)"
//"    6=4096KB(1024KB+1024KB)"
//"    7=4096KB(2048KB+2048KB)"
//"    8=8192KB(1024KB+1024KB)"
//"    9=16384KB(1024KB+1024KB)"
#elif (CONFIG_SPI_FLASH_SIZE_MAP == 2)
#define SYSTEM_PARTITION_OTA_SIZE 0x6A000
#define SYSTEM_PARTITION_OTA_2_ADDR 0x81000
#define SYSTEM_PARTITION_RF_CAL_ADDR 0xfb000
#define SYSTEM_PARTITION_PHY_DATA_ADDR 0xfc000
#define SYSTEM_PARTITION_SYSTEM_PARAMETER_ADDR 0xfd000
#elif (CONFIG_SPI_FLASH_SIZE_MAP == 3)
#define SYSTEM_PARTITION_OTA_SIZE 0x6A000
#define SYSTEM_PARTITION_OTA_2_ADDR 0x81000
#define SYSTEM_PARTITION_RF_CAL_ADDR 0x1fb000
#define SYSTEM_PARTITION_PHY_DATA_ADDR 0x1fc000
#define SYSTEM_PARTITION_SYSTEM_PARAMETER_ADDR 0x1fd000
#elif (CONFIG_SPI_FLASH_SIZE_MAP == 4)
#define SYSTEM_PARTITION_OTA_SIZE 0x6A000
#define SYSTEM_PARTITION_OTA_2_ADDR 0x81000
#define SYSTEM_PARTITION_RF_CAL_ADDR 0x3fb000
#define SYSTEM_PARTITION_PHY_DATA_ADDR 0x3fc000
#define SYSTEM_PARTITION_SYSTEM_PARAMETER_ADDR 0x3fd000
#elif (CONFIG_SPI_FLASH_SIZE_MAP == 5)
#define SYSTEM_PARTITION_OTA_SIZE 0x6A000
#define SYSTEM_PARTITION_OTA_2_ADDR 0x101000
#define SYSTEM_PARTITION_RF_CAL_ADDR 0x1fb000 // ?
#define SYSTEM_PARTITION_PHY_DATA_ADDR 0x1fc000 // ?
#define SYSTEM_PARTITION_SYSTEM_PARAMETER_ADDR 0x1fd000 // ?
#elif (CONFIG_SPI_FLASH_SIZE_MAP == 6)
#define SYSTEM_PARTITION_OTA_SIZE 0x6A000
#define SYSTEM_PARTITION_OTA_2_ADDR 0x101000
#define SYSTEM_PARTITION_RF_CAL_ADDR 0x3fb000
#define SYSTEM_PARTITION_PHY_DATA_ADDR 0x3fc000
#define SYSTEM_PARTITION_SYSTEM_PARAMETER_ADDR 0x3fd000
#else
#error "The flash map is not supported"
#endif

#define EEPROM_PARTITION_ADDR (SYSTEM_PARTITION_RF_CAL_ADDR - 0x1000)
#define EEPROM_PARTITION_SIZE 0x1000
#define FS_PARTITION_ADDR SYSTEM_PARTITION_OTA_2_ADDR
#define FS_PARTITION_SIZE (EEPROM_PARTITION_ADDR-FS_PARTITION_ADDR)

#define FLASH_EEPROM_OFFSET EEPROM_PARTITION_ADDR
#define FLASH_EEPROM_SIZE EEPROM_PARTITION_SIZE


#ifndef DEBUG_H
#define DEBUG_H

#ifdef CONFIG_DEBUG_VERBOSE
#define ETS_DEBUG(fmt,args...) ets_debug(fmt,## args)
void ets_debug(const char *fmt, ...);
void debug_state(void);
void debug_print_buffer(const char *msg, unsigned const char *p, size_t size);

#else
#define ETS_DEBUG(fmt,args...) do{}while(0)
#define debug_state() do{}while(0)
#endif

void debug_atn_command(char *message, uint8_t cmd1);
void debug_print_buffer(const char *msg, unsigned const char *p, size_t size);
char *state2str(int bus_state);
char *dstate2str(int device_state);
char *atncmd2str(int cmd);
#endif

#endif
