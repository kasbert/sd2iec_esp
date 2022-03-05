# architecture-dependent variables

#export ESP_SDK=/b/Projects/esp/ESP8266_NONOS_SDK/

#---------------- Source code ----------------
ASMSRC =

SRC += esp/crc.c
SRC += esp/iec-bus.c
SRC += esp/arch-eeprom.c
SRC += esp/debug.c
SRC += esp/llfl-common.c
SRC += esp/llfl-jiffydos.c
ifeq ($(CONFIG_LOADER_TURBODISK),y)
  SRC += esp/llfl-turbodisk.c
endif
ifeq ($(CONFIG_LOADER_DREAMLOAD),y)
  SRC += esp/llfl-dreamload.c
endif
ifeq ($(CONFIG_LOADER_FC3),y)
SRC += esp/llfl-fc3exos.c
endif
ifeq ($(CONFIG_LOADER_AR6),y)
SRC += esp/llfl-ar6.c
endif
ifeq ($(CONFIG_LOADER_ULOAD3),y)
SRC += esp/llfl-ulm3.c
endif
ifeq ($(CONFIG_LOADER_EPYXCART),y)
SRC += esp/llfl-epyxcart.c
endif
ifeq ($(CONFIG_LOADER_GEOS),y)
SRC += esp/llfl-geos.c
endif
ifeq ($(CONFIG_PARALLEL_DOLPHIN),y)
SRC += esp/llfl-parallel.c
endif
ifeq ($(CONFIG_LOADER_N0SDOS),y)
SRC += esp/llfl-n0sdos.c
endif

#---------------- Toolchain ----------------
CC = xtensa-lx106-elf-gcc
OBJCOPY = xtensa-lx106-elf-objcopy
OBJDUMP = xtensa-lx106-elf-objdump
SIZE = xtensa-lx106-elf-size
NM = xtensa-lx106-elf-nm
ESPTOOLS = esptool

#---------------- Bootloader, fuses etc. ----------------
MCU    := $(CONFIG_MCU)

ifeq ($(MCU),esp8266)
  ROM_ADDRESS = 0x10000
else
.PHONY: nochip
nochip:
	@echo '=============================================================='
	@echo 'No known target chip specified.'
	@echo
	@echo 'Please edit the Makefile.'
	@exit 1
endif

#---------------- External Memory Options ----------------
#BOOT=none APP=0 SPI_SPEED=40 SPI_MODE=QIO SPI_SIZE_MAP=4 
ESPTOOLS_FLAGS = -fs 4MB
#ESPTOOLS_FLAGS += --baud 921600 --before default_reset --after hard_reset

#---------------- Architecture variables ----------------
ARCH_CFLAGS = -Os -g -Wpointer-arith -Wundef -Wl,-EL -fno-inline-functions \
-nostdlib -mlongcalls -mtext-section-literals -ffunction-sections -fdata-sections \
-fno-builtin-printf -fno-jump-tables -fno-guess-branch-probability -freorder-blocks-and-partition -fno-cse-follow-jumps 
ARCH_CFLAGS += -DICACHE_FLASH=1
ARCH_CFLAGS += -D__ets__
ARCH_CFLAGS += -I $(ESP_SDK)/include/ets -I $(ESP_SDK)/include -I $(ESP_SDK)/driver_lib/include

ARCH_ASFLAGS = -mmcu=$(MCU)
ARCH_LDFLAGS = $(EXTMEMOPTS)
ARCH_LDFLAGS += -L$(ESP_SDK)lib -Tscripts/esp/eagle.app.v6.ld
ARCH_LDFLAGS += -Wl,--no-check-sections -Wl,--gc-sections -u call_user_start -Wl,-static
ARCH_LDFLAGS += -Wl,--start-group -lc -lgcc -lhal -lphy -lpp -lnet80211 -llwip -lwpa -lcrypto -lmain -ldriver -Wl,--end-group

#---------------- Config ----------------
# currently no stack tracking supported
