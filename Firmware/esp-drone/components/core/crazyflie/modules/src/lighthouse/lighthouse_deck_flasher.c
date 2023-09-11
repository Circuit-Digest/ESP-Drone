/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2020 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * lighthouse_deck_flasher.h - handles flashing of FPGA binaries on the
 * Lighthouse deck
 */

#include <string.h>
#include <stdlib.h>

#define DEBUG_MODULE "LHFL"
#include "debug.h"
#include "lh_bootloader.h"
#include "lighthouse_deck_flasher.h"

#ifdef LH_FLASH_BOOTLOADER
#include "lh_flasher.h"
#endif


// Uncomment if you want to force the Crazyflie to reflash the FPGA binary to the deck at each startup
// #define FORCE_FLASH_FPGA true

#ifndef FORCE_FLASH_FPGA
#define FORCE_FLASH_FPGA false
#endif


#ifndef DISABLE_LIGHTHOUSE_DRIVER
  #define DISABLE_LIGHTHOUSE_DRIVER 1
#endif


#if DISABLE_LIGHTHOUSE_DRIVER == 1
void lighthouseDeckFlasherCheckVersionAndBoot() {
    DEBUG_PRINT("No Lighthouse support in FW, deck not initialized\n");
}
#else

#define STR2(x) #x
#define STR(x) STR2(x)

#define INCBIN(name, file) \
    __asm__(".section .rodata\n" \
            ".global incbin_" STR(name) "_start\n" \
            ".align 4\n" \
            "incbin_" STR(name) "_start:\n" \
            ".incbin \"" file "\"\n" \
            \
            ".global incbin_" STR(name) "_end\n" \
            ".align 1\n" \
            "incbin_" STR(name) "_end:\n" \
            ".byte 0\n" \
            ".align 4\n" \
            STR(name) "Size:\n" \
            ".int incbin_" STR(name) "_end - incbin_" STR(name) "_start\n" \
    ); \
    extern const __attribute__((aligned(4))) void* incbin_ ## name ## _start; \
    extern const void* incbin_ ## name ## _end; \
    extern const int name ## Size; \
    static const __attribute__((used)) unsigned char* name = (unsigned char*) & incbin_ ## name ## _start; \

INCBIN(bitstream, BLOBS_LOC"lighthouse.bin");


void lighthouseDeckFlasherCheckVersionAndBoot() {
  lhblInit(I2C1_DEV);

  #ifdef LH_FLASH_BOOTLOADER
  // Flash deck bootloader using SPI (factory and recovery flashing)
  lhflashInit();
  lhflashFlashBootloader();
  #endif

  uint8_t bootloaderVersion = 0;
  lhblGetVersion(&bootloaderVersion);
  DEBUG_PRINT("Lighthouse bootloader version: %d\n", bootloaderVersion);

  // Wakeup mem
  lhblFlashWakeup();
  vTaskDelay(M2T(1));

  // Checking the bitstreams are identical
  // Also decoding bitstream version for console
  static char deckBitstream[65];
  lhblFlashRead(LH_FW_ADDR, 64, (uint8_t*)deckBitstream);
  deckBitstream[64] = 0;
  int deckVersion = strtol(&deckBitstream[2], NULL, 10);
  int embeddedVersion = strtol((char*)&bitstream[2], NULL, 10);

  bool identical = true;
  for (int i=0; i<=bitstreamSize; i+=64) {
    int length = ((i+64)<bitstreamSize)?64:bitstreamSize-i;
    lhblFlashRead(LH_FW_ADDR + i, length, (uint8_t*)deckBitstream);
    if (memcmp(deckBitstream, &bitstream[i], length)) {
      DEBUG_PRINT("Fail comparing firmware\n");
      identical = false;
      break;
    }
  }

  if (identical == false || FORCE_FLASH_FPGA) {
    DEBUG_PRINT("Deck has version %d and we embeed version %d\n", deckVersion, embeddedVersion);
    DEBUG_PRINT("Updating deck with embedded version!\n");

    // Erase LH deck FW
    lhblFlashEraseFirmware();

    // Flash LH deck FW
    if (lhblFlashWriteFW((uint8_t*)bitstream, bitstreamSize)) {
      DEBUG_PRINT("FW updated [OK]\n");
      deckVersion = embeddedVersion;
    } else {
      DEBUG_PRINT("FW updated [FAILED]\n");
    }
  }

  // Launch LH deck FW
  DEBUG_PRINT("Firmware version %d verified, booting deck!\n", deckVersion);
  lhblBootToFW();
}

#endif
