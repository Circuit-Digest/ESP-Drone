

#include "xtensa_math.h"

 /**
 * @addtogroup PID
 * @{
 */

/**
* @brief  Reset function for the floating-point PID Control.
* @param[in] *S	Instance pointer of PID control data structure.
* @return none.
* \par Description:
* The function resets the state buffer to zeros.
*/
void xtensa_pid_reset_f32(
  xtensa_pid_instance_f32 * S)
{

  /* Clear the state buffer.  The size will be always 3 samples */
  memset(S->state, 0, 3U * sizeof(float32_t));
}

/**
 * @} end of PID group
 */
