/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
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
 * param.h - Crazy parameter system header file.
 */

#ifndef __PARAM_H__
#define __PARAM_H__

#include <stdbool.h>
#include <stdint.h>

/* Public functions */
void paramInit(void);
bool paramTest(void);

/* Public API to access param variables */

/** Variable identifier.
 * 
 * Should be fetched with paramGetVarId(). This is to be considered as an
 * opaque type, internal structure might change.
 * 
 * Use PARAM_VARID_IS_VALID() to check if the ID is valid.
 */
typedef struct paramVarId_s {
  uint16_t id;
  uint16_t ptr;
} __attribute__((packed)) paramVarId_t;

/** Get the varId from group and name of variable
 * 
 * @param group Group name of the variable
 * @param name Name of the variable
 * @return The variable ID or an invalid ID. Use PARAM_VARID_IS_VALID() to check validity.
 */
paramVarId_t paramGetVarId(char* group, char* name);

/** Check variable ID validity
 * 
 * @param varId variable ID, returned by paramGetVarId()
 * @return true if the variable ID is valid, false otherwise.
 */
#define PARAM_VARID_IS_VALID(varId) (varId.id != 0xffffu)

/** Return the parameter type
 * 
 * @param varId variable ID, returned by paramGetVarId()
 * @return Type of the variable. The value correspond to the defines used when
 *         declaring a param variable.
 */
int paramGetType(paramVarId_t varid);

/** Get group and name strings of a parameter
 * 
 * @param varId variable ID, returned by paramGetVarId()
 * @param group Pointer to a char* that will be filled with the group name
 * @param group Pointer to a char* that will be filled with the variable name
 * 
 * The string buffers must be able to hold at least 32 bytes.
 */
void paramGetGroupAndName(paramVarId_t varid, char** group, char** name);

/** Get parameter variable size in byte
 * 
 * @param type Type returned by paramGetType()
 * @return Size in byte occupied by variable of this type
 */
uint8_t paramVarSize(int type);

/** Return float value of a parameter
 * 
 * @param varId variable ID, returned by paramGetVarId()
 * @return Current value of the variable
 */
float paramGetFloat(paramVarId_t varid);

/** Return int value of a parameter
 * 
 * @param varId variable ID, returned by paramGetVarId()
 * @return Current value of the variable
 */
int paramGetInt(paramVarId_t varid);

/** Return Unsigned int value of a paramter
 * 
 * @param varId variable ID, returned by paramGetVarId()
 * @return Current value of the variable
 */
unsigned int paramGetUint(paramVarId_t varid);

/** Set int value of a parameter
 * 
 * @param varId variable ID, returned by paramGetVarId()
 * @param valuei Value to set in the variable
 */
void paramSetInt(paramVarId_t varid, int valuei);

/** Set float value of a parameter
 * 
 * @param varId variable ID, returned by paramGetVarId()
 * @param valuef Value to set in the variable
 */
void paramSetFloat(paramVarId_t varid, float valuef);


/* Basic parameter structure */
struct param_s {
  uint8_t type;
  char * name;
  void * address;
};

#define PARAM_BYTES_MASK 0x03
#define PARAM_1BYTE  0x00
#define PARAM_2BYTES 0x01
#define PARAM_4BYTES 0x02
#define PARAM_8BYTES 0x03

#define PARAM_TYPE_INT   (0x00<<2)
#define PARAM_TYPE_FLOAT (0x01<<2)

#define PARAM_SIGNED (0x00<<3)
#define PARAM_UNSIGNED (0x01<<3)

#define PARAM_VARIABLE (0x00<<7)
#define PARAM_GROUP    (0x01<<7)

#define PARAM_RONLY (1<<6)

#define PARAM_START 1
#define PARAM_STOP  0

#define PARAM_SYNC 0x02

// User-friendly macros
#define PARAM_UINT8 (PARAM_1BYTE | PARAM_TYPE_INT | PARAM_UNSIGNED)
#define PARAM_INT8  (PARAM_1BYTE | PARAM_TYPE_INT | PARAM_SIGNED)
#define PARAM_UINT16 (PARAM_2BYTES | PARAM_TYPE_INT | PARAM_UNSIGNED)
#define PARAM_INT16  (PARAM_2BYTES | PARAM_TYPE_INT | PARAM_SIGNED)
#define PARAM_UINT32 (PARAM_4BYTES | PARAM_TYPE_INT | PARAM_UNSIGNED)
#define PARAM_INT32  (PARAM_4BYTES | PARAM_TYPE_INT | PARAM_SIGNED)

#define PARAM_FLOAT (PARAM_4BYTES | PARAM_TYPE_FLOAT | PARAM_SIGNED)

/* Macros */
#ifndef UNIT_TEST_MODE

#define PARAM_ADD(TYPE, NAME, ADDRESS) \
   { .type = TYPE, .name = #NAME, .address = (void*)(ADDRESS), },

#define PARAM_ADD_GROUP(TYPE, NAME, ADDRESS) \
   { \
  .type = TYPE, .name = #NAME, .address = (void*)(ADDRESS), },

#define PARAM_GROUP_START(NAME)  \
  static const struct param_s __params_##NAME[] __attribute__((section(".param." #NAME), used)) = { \
  PARAM_ADD_GROUP(PARAM_GROUP | PARAM_START, NAME, 0x0)

//#define PARAM_GROUP_START_SYNC(NAME, LOCK) PARAM_ADD_GROUP(PARAM_GROUP | PARAM_START, NAME, LOCK);

#define PARAM_GROUP_STOP(NAME) \
  PARAM_ADD_GROUP(PARAM_GROUP | PARAM_STOP, stop_##NAME, 0x0) \
  };

#else // UNIT_TEST_MODE

// Empty defines when running unit tests
#define PARAM_ADD(TYPE, NAME, ADDRESS)
#define PARAM_ADD_GROUP(TYPE, NAME, ADDRESS)
#define PARAM_GROUP_START(NAME)
#define PARAM_GROUP_STOP(NAME)

#endif // UNIT_TEST_MODE

#endif /* __PARAM_H__ */
