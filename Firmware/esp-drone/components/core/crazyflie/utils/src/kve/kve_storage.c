/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2019 - 2020 Bitcraze AB
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
 * kveStorage.c - Low level storage functions
 *
 */

#include "kve/kve_storage.h"

#include <stdint.h>
#include <string.h>

static size_t min(size_t a, size_t b)
{
    if (a < b) {
        return a;
    } else {
        return b;
    }
}

#define END_TAG (0xffffu)

int kveStorageWriteItem(kveMemory_t *kve, size_t address, const char* key, const void* buffer, size_t length)
{
  kveItemHeader_t header;
  header.key_length = strlen(key);
  header.full_length = 2 + 1 + header.key_length + length;

  // Write full_length, key_length, key and buffer
  kve->write(address, &header, sizeof(header));
  kve->write(address + sizeof(header), key, header.key_length);
  kve->write(address + sizeof(header) + header.key_length, buffer, length);

  kve->flush();

  return header.full_length;
}

uint16_t kveStorageWriteHole(kveMemory_t *kve, size_t address, size_t full_length) {
  kveItemHeader_t header;
  header.full_length = full_length;
  header.key_length = 0;

  kve->write(address, &header, sizeof(header));
  kve->flush();

  return full_length;   
}

uint16_t kveStorageWriteEnd(kveMemory_t *kve, size_t address) {
    uint16_t endTag = END_TAG;

    kve->write(address, &endTag, 2);

    kve->flush();

    return 2;
}

void kveStorageMoveMemory(kveMemory_t *kve, size_t sourceAddress, size_t destinationAddress, size_t length)
{
    static char moveBuffer[32];
    size_t leftToMove = length;

    while (leftToMove > 0) {
        size_t moving = min(leftToMove, (size_t)32);
        kve->read(sourceAddress, moveBuffer, moving);
        kve->write(destinationAddress, moveBuffer, moving);

        sourceAddress += moving;
        destinationAddress += moving;
        leftToMove -= moving;
    }

    kve->flush();
}

size_t kveStorageFindItemByKey(kveMemory_t *kve, size_t address, const char * key) {
    char searchBuffer[255];
    size_t currentAddress = address;
    uint16_t length;
    uint8_t keyLength;
    uint8_t searchedKeyLength = strlen(key);


    while (currentAddress < (kve->memorySize - 3)) {
        kve->read(currentAddress, searchBuffer, 3);
        length = searchBuffer[0] + (searchBuffer[1]<<8);
        keyLength = searchBuffer[2];

        if (length == END_TAG) {
            return SIZE_MAX;
        }

        if (keyLength == searchedKeyLength) {
            kve->read(currentAddress + 3, &searchBuffer, searchedKeyLength);
            if (!memcmp(key, searchBuffer, keyLength)) {
                return currentAddress;
            }
        }

        currentAddress += length;
    }

    return SIZE_MAX;
}

size_t kveStorageFindEnd(kveMemory_t *kve, size_t address) {
    size_t currentAddress = address;
    kveItemHeader_t header;

    while (currentAddress < (kve->memorySize - 2)) {
        kve->read(currentAddress, &header, sizeof(header));
        if (header.full_length == END_TAG) {
            return currentAddress;
        }

        // An item must at least have a key of len>=1
        if (header.full_length < (sizeof(header) + 1)) {
            return KVE_STORAGE_INVALID_ADDRESS;
        }
        currentAddress += header.full_length;
    }

    // This is a corrupted table!
    return KVE_STORAGE_INVALID_ADDRESS;
}

size_t kveStorageFindHole(kveMemory_t *kve, size_t address) {
    size_t currentAddress = address;
    kveItemHeader_t header;

    while (currentAddress < (kve->memorySize - 2)) {
        kve->read(currentAddress, &header, sizeof(header));
        if (header.key_length == 0) {
            return currentAddress;
        }
        currentAddress += header.full_length;
    }

    // This is a corrupted table!
    return KVE_STORAGE_INVALID_ADDRESS;
}


size_t kveStorageFindNextItem(kveMemory_t *kve, size_t address)
{
    size_t currentAddress = address;
    kveItemHeader_t header;

    // Jump over the current item
    kve->read(currentAddress, &header, sizeof(header));
    if (header.full_length == END_TAG) {
        return KVE_STORAGE_INVALID_ADDRESS;
    }
    currentAddress += header.full_length;

    while (currentAddress < (kve->memorySize - 3)) {
        kve->read(currentAddress, &header, sizeof(header));


        if (header.full_length == END_TAG) {
            return KVE_STORAGE_INVALID_ADDRESS;
        }

        if (header.key_length != 0) {
            return currentAddress;
        }
        
        currentAddress += header.full_length;
    }

    return KVE_STORAGE_INVALID_ADDRESS;
}

kveItemHeader_t kveStorageGetItemInfo(kveMemory_t *kve, size_t address)
{
  kveItemHeader_t header;
  
  kve->read(address, &header, sizeof(header));

  return header;
}

size_t kveStorageGetKeyLength(kveItemHeader_t header)
{
  return header.key_length;
}

size_t kveStorageGetBufferLength(kveItemHeader_t header)
{
  return header.full_length - sizeof(header) - header.key_length;
}

size_t kveStorageGetKey(kveMemory_t *kve, size_t address, kveItemHeader_t header, char* key, size_t maxLength)
{
  size_t sizeToRead = min(maxLength, header.key_length);

  kve->read(address + sizeof(header), key, sizeToRead);

  return sizeToRead;
}

size_t kveStorageGetBuffer(kveMemory_t *kve, size_t address, kveItemHeader_t header, void* buffer, size_t maxLength)
{
  size_t sizeToRead = min(kveStorageGetBufferLength(header), maxLength);

  kve->read(address + sizeof(header) + header.key_length, buffer, sizeToRead);

  return sizeToRead;
}