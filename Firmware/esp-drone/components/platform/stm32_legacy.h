#ifndef ESP32_BRIDGE_H
#define ESP32_BRIDGE_H

#include "esp_err.h"
#include "cfassert.h"

#ifndef __cplusplus
//typedef enum {FALSE = 0, TRUE = !FALSE} bool;
#endif

typedef enum {RESET = 0, SET = !RESET} FlagStatus, ITStatus;

typedef enum {DISABLE = 0, ENABLE = !DISABLE} FunctionalState;
#define IS_FUNCTIONAL_STATE(STATE) (((STATE) == DISABLE) || ((STATE) == ENABLE))

typedef enum {ERROR = 0, SUCCESS = !ERROR} ErrorStatus;

typedef struct {

} GPIO_TypeDef;


#define TASK_LED_ID_NBR         1
#define TASK_RADIO_ID_NBR       2
#define TASK_STABILIZER_ID_NBR  3
#define TASK_ADC_ID_NBR         4
#define TASK_PM_ID_NBR          5
#define TASK_PROXIMITY_ID_NBR   6

typedef enum {
    Bit_RESET = 0,
    Bit_SET
} BitAction;

#define __IO

#ifndef FALSE
# define FALSE 0
#endif

#ifndef TRUE
# define TRUE 1
#endif

#define pdFALSE			( ( BaseType_t ) 0 )
#define pdTRUE			( ( BaseType_t ) 1 )

#define pdPASS			( pdTRUE )
#define pdFAIL			( pdFALSE )
#define errQUEUE_EMPTY	( ( BaseType_t ) 0 )
#define errQUEUE_FULL	( ( BaseType_t ) 0 )

#define M2T(X) ((unsigned int)(X)/ portTICK_PERIOD_MS) //ms to tick
#define F2T(X) ((unsigned int)((configTICK_RATE_HZ/(X))))
#define T2M(X) ((unsigned int)(X)* portTICK_PERIOD_MS)   //tick to ms

// Seconds to OS ticks
#define S2T(X) ((portTickType)((X) * configTICK_RATE_HZ))
#define T2S(X) ((X) / (float)configTICK_RATE_HZ)

#define assert_param(e)  if (e) ; \
    else assertFail( #e, __FILE__, __LINE__ )

uint64_t usecTimestamp(void);

/* GPIO */

#endif