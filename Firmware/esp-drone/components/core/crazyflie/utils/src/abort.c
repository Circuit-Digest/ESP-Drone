/**
 *
 * abort.c: Simple abort function.
 *
 * Newer versions of newlibc always use unwind functions which do need an
 * implementation of abort(). So in order to link without standard libraries
 * (-nostdlib) we have to provide abort().
 */

void abort(void)
{
  while (1)
    ;
}
