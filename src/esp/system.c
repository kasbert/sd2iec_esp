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


   system.c: System-specific initialisation (ESP version)

*/

#include "system.h"
#include "atomic.h"
#include "config.h"
#include "led.h"

#include "ets_sys.h"
#include "mem.h"
#include "osapi.h"
#define system_upgrade_flag_check() system_upgrade_flag_check(void)
#define system_get_os_print() system_get_os_print(void)
#define wifi_status_led_uninstall() wifi_status_led_uninstall(void)
#include "user_interface.h"

int _led_state;

void main(void);
void bus_mainloop(void);

#define SIG_ATN 1
#define MAINLOOP_QUEUE_LEN 2
static os_event_t *testQueue;
static void pin_intr_handler(void *);
static os_timer_t status_timer;
static int in_mainloop;

/* Early system initialisation */
void system_init_early(void) {
  // disable_interrupts();
  return;
}

/* Late initialisation, increase CPU clock */
void system_init_late(void) {

  PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO0_U, FUNC_GPIO0); // Use as GPIO.
  PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO4_U, FUNC_GPIO4); // Use as GPIO.
  PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO5_U, FUNC_GPIO5); // Use as GPIO.

  // Not needed as there is an external pullup
  /*
  PIN_PULLUP_EN(PERIPHS_IO_MUX_GPIO0_U);
  PIN_PULLUP_EN(PERIPHS_IO_MUX_GPIO4_U);
  PIN_PULLUP_EN(PERIPHS_IO_MUX_GPIO5_U);
  // No pullup PIN_PULLUP_EN(16)
  */
}

/* Enable GPIO interrupt */
void iec_interrupts_init(void) {
  ETS_GPIO_INTR_DISABLE();
  ETS_GPIO_INTR_ATTACH(pin_intr_handler, 0);
  ETS_GPIO_INTR_ENABLE();
}

/* Put MCU in low-power mode */
// void system_sleep(void) {
// FIXME esp_sleep_enable_gpio_wakeup
//}

/* Reset MCU */
void system_reset(void) {
  ETS_DEBUG("system_reset");
  uart_flush();
  system_restart();
  while (1)
    ;
}

IRAM_ATTR
void disable_interrupts(void) {
  // ETS_DEBUG("disable_interrupts");
  __disable_irq();
}

IRAM_ATTR
void enable_interrupts(void) {
  // ETS_DEBUG("enable_interrupts");
  __enable_irq();
}

/*** Timer/GPIO interrupt demux ***/

/* Declare handler functions */
SD_CHANGE_HANDLER;
IEC_ATN_HANDLER;
IEC_CLOCK_HANDLER;

IRAM_ATTR
static void pin_intr_handler(void *ctx) {
  uint32_t gpio_status;
  gpio_status = GPIO_REG_READ(GPIO_STATUS_ADDRESS);
  // clear	interrupt	status
  GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, gpio_status);

  iec_atn_handler();

#ifdef CONFIG_LOADER_DREAMLOAD
  iec_clock_handler();
#endif

  if (!in_mainloop) {
    system_os_post(USER_TASK_PRIO_0, SIG_ATN, 0);
  }
}

void panic(const char *msg) {
  unsigned i;
  uart_puts_P("PANIC: ");
  uart_puts_P(msg);
  uart_putcrlf();
  uart_flush();

  for (i = 0;; i++) {
    /* Set GPIO */
    gpio_output_set((i & 0x10000) ? 0 : (1 << PIN_LED_BUILDIN),
                    (i & 0x10000) ? (1 << PIN_LED_BUILDIN) : 0,
                    (1 << PIN_LED_BUILDIN), 0);
    system_soft_wdt_feed();
  }
}

#ifdef CONFIG_NETWORK
char * wifistatus2str(int status) {
  switch (status) {
    case STATION_IDLE: return "STATION_IDLE";
    case STATION_CONNECTING: return "STATION_CONNECTING";
    case STATION_WRONG_PASSWORD: return "STATION_WRONG_PASSWORD";
    case STATION_NO_AP_FOUND: return "STATION_NO_AP_FOUND";
    case STATION_CONNECT_FAIL: return "STATION_CONNECT_FAIL";
    case STATION_GOT_IP: return "STATION_GOT_IP";
    default: return "UNKNOWN";
  };
}
#endif

// Just to indicate that the device is alive
// TODO remove
LOCAL void ICACHE_FLASH_ATTR status_timer_callback(void *arg) {
  if (!(led_state)) {
    toggle_led();
  }
#ifdef CONFIG_NETWORK
  ETS_DEBUG("STATION %s ", wifistatus2str(wifi_station_get_connect_status()));
#endif
  debug_state();
  /*
    input = IEC_INPUT;
    ETS_DEBUG("%lu (%d) %s %s %s %s\n", system_get_time(), getticks(),
    (input & IEC_BIT_ATN) ? "ATN:1 " : "ATN:_ ",
    (input & IEC_BIT_CLOCK) ? "CLK:1 " : "CLK:_ ",
    (input & IEC_BIT_DATA) ? "DAT:1 " : "DAT:_ ",
    gpio16_input_get() ? "SRQ:1 " : "SRQ:_ ");
  */
  uart_flush();
}

void mainloop_task(os_event_t *e) {
  switch (e->sig) {
  case SIG_ATN:
    //debug_state();
    uart_putc('>');
    uart_putcrlf();
    in_mainloop = 1;
    bus_mainloop();
    in_mainloop = 0;
    uart_putc('<');
    uart_putcrlf();
    debug_state();
    break;

  default:
    ETS_DEBUG("sig	%d\n", e->sig);
    break;
  }
  uart_flush();
}

void task_init(void) {
  testQueue = (os_event_t *)os_malloc(sizeof(os_event_t) * MAINLOOP_QUEUE_LEN);
  system_os_task(mainloop_task, USER_TASK_PRIO_0, testQueue,
                 MAINLOOP_QUEUE_LEN);
  // system_os_post(USER_TASK_PRIO_0, SIG_RX, 'a');
}

void ICACHE_FLASH_ATTR init_done(void) {
  ETS_DEBUG("SDK version: %s\n", system_get_sdk_version());
  system_print_meminfo();
  uart_putcrlf();
  ETS_DEBUG("system_get_time %d\n\r", system_get_time());

  os_timer_disarm(&status_timer);
  os_timer_setfn(&status_timer, &status_timer_callback, (void *)0);
  os_timer_arm(&status_timer, 5000, 1);
  task_init();

  main();

  set_atn_irq(1);
  ETS_DEBUG("init_done");
  uart_flush();
}


void ICACHE_FLASH_ATTR user_init(void) {
  uart_init();
  ets_install_putc1(uart_putc);
  ETS_DEBUG("user_init");
  gpio_init();
  system_update_cpu_freq(CONFIG_MCU_FREQ / 1000000); // 80 or 160
  system_init_done_cb(init_done);

  wifi_set_opmode(STATIONAP_MODE);	//Set	softAP	+	station	mode

  uart_flush();
}

static const partition_item_t at_partition_table[] = {
    {SYSTEM_PARTITION_BOOTLOADER, 0x0, 0x1000},
    {SYSTEM_PARTITION_OTA_1, 0x1000, SYSTEM_PARTITION_OTA_SIZE},
    /*
    {SYSTEM_PARTITION_OTA_2, SYSTEM_PARTITION_OTA_2_ADDR,
    SYSTEM_PARTITION_OTA_SIZE},
    */
    {SYSTEM_PARTITION_RF_CAL, SYSTEM_PARTITION_RF_CAL_ADDR, 0x1000},
    {SYSTEM_PARTITION_PHY_DATA, SYSTEM_PARTITION_PHY_DATA_ADDR, 0x1000},
    {SYSTEM_PARTITION_SYSTEM_PARAMETER, SYSTEM_PARTITION_SYSTEM_PARAMETER_ADDR,
     0x3000},
    {100, FS_PARTITION_ADDR, FS_PARTITION_SIZE},
    {101, EEPROM_PARTITION_ADDR, EEPROM_PARTITION_SIZE},
    /*
     */
};

void ICACHE_FLASH_ATTR user_pre_init(void) {
  ETS_DEBUG("user_pre_init");
  if (!system_partition_table_regist(at_partition_table,
                                     sizeof(at_partition_table) /
                                         sizeof(at_partition_table[0]),
                                     CONFIG_SPI_FLASH_SIZE_MAP)) {
    panic("system_partition_table_regist fail");
  }
}
