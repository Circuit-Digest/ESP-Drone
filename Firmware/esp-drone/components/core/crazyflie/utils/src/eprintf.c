/**
 *    ||          ____  _ __                           
 * +------+      / __ )(_) /_______________ _____  ___ 
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (c) 2012 Bitcraze AB
 *
 * eprintf.c: Memory-friendly ultra-limited embedded implementation of printf
 *
 * This implementation trades CPU cycle for data memory eficiency
 * (No malloc, no unecessary buffers, etc...)
 *
 * Functionality: Implements %s, %d, %x, %X and %f without print size settings.
 * The float handling is verry limited and without exponential notation (ie.
 * works good for number around 0 and within int32 value range).
 *
 * To use this printf a 'putc' function shall be implemented with the prototype
 * 'int putc(int)'. Then a macro calling eprintf can be created. For example:
 * int consolePutc(int c);
 * #define consolePrintf(FMT, ...) eprintf(consolePutc, FMT, ## __VA_ARGS__)
 */
#include "eprintf.h"

#include <stdarg.h>
#include <stdint.h>
#include <stdbool.h>
#include <ctype.h>

static const char digit[] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 
                             'A', 'B', 'C', 'D', 'E', 'F'};

static int getIntLen (long int value)
{
  int l = 1;
  while(value > 9)
  {
    l++;
    value /= 10;
  }
  return l;
}

int power(int a, int b)
{
  int i;
  int x = a;

  for (i = 1; i < b; i++)
  {
    x *= a;
  }

  return x;
}

static int itoa10Unsigned(putc_t putcf, unsigned long long int num)
{
  int len = 0;

  if (num == 0)
  {
    putcf('0');
    return 1;
  }

  unsigned long long int i = 1;

  while ((num / i) > 9)
  {
    i *= 10L;
  }

  do
  {
    putcf(digit[(num / i) % 10L]);
    len++;
  }
  while (i /= 10L);

  return len;
}

static int itoa10(putc_t putcf, long long int num, int precision)
{
  int len = 0;

  if (num == 0)
  {
    putcf('0');
    return 1;
  }

  long long unsigned int n = num;
  if (num < 0)
  {
    n = -num;
    putcf('-');
    len++;
  }

  int numLenght = getIntLen(num);
  if (numLenght < precision)
  {
    int fillWithZero = precision - numLenght;
    while (fillWithZero > 0)
    {
      putcf('0');
      len++;
      fillWithZero--;
    }
  }

  return itoa10Unsigned(putcf, n) + len;
}

static int itoa16(putc_t putcf, uint64_t num, int width, char padChar)
{
  int len = 0;
  bool foundFirst = false;

  for (int i = 15; i >= 0; i--)
  {
    int shift = i * 4;
    uint64_t mask = (uint64_t)0x0F << shift;
    uint64_t val = (num & mask) >> shift;

    if (val > 0)
    {
      foundFirst = true;
    }

    if (foundFirst || i < width)
    {
      if (foundFirst)
      {
        putcf(digit[val]);
      }
      else
      {
        putcf(padChar);
      }

      len++;
    }
  }

  return len;
}

static int handleLongLong(putc_t putcf, char** fmt, unsigned long long int val, int width, char padChar)
{
  int len = 0;

  switch(*((*fmt)++))
  {
    case 'i':
    case 'd':
      len = itoa10(putcf, (long long int)val, 0);
      break;
    case 'u':
      len = itoa10Unsigned(putcf, val);
      break;
    case 'x':
    case 'X':
      len = itoa16(putcf, val, width, padChar);
      break;
    default:
      // Nothing here
      break;
  }

  return len;
}

static int handleLong(putc_t putcf, char** fmt, unsigned long int val, int width, char padChar)
{
  int len = 0;

  switch(*((*fmt)++))
  {
    case 'i':
    case 'd':
      len = itoa10(putcf, (long int)val, 0);
      break;
    case 'u':
      len = itoa10Unsigned(putcf, val);
      break;
    case 'x':
    case 'X':
      len = itoa16(putcf, val, width, padChar);
      break;
    default:
      // Nothing here
      break;
  }

  return len;
}

int evprintf(putc_t putcf, char * fmt, va_list ap)
{
  int len=0;
  float num;
  char* str;
  int precision;
  int width;
  char padChar;

  while (*fmt)
  {
    if (*fmt == '%')
    {
      precision = 6;
      padChar = ' ';
      width = 0;

      fmt++;
      while ('0' == *fmt)
      {
        padChar = '0';
        fmt++;
      }

			while(isdigit((unsigned)*fmt))
			{
				width *= 10;
				width += *fmt - '0';
				fmt++;
			}

      while (!isalpha((unsigned) *fmt))
      {
        if (*fmt == '.')
        {
          fmt++;
          if (isdigit((unsigned)*fmt))
          {
            precision = *fmt - '0';
            fmt++;
          }
        }
      }
      switch (*fmt++)
      {
        case 'i':
        case 'd':
          len += itoa10(putcf, va_arg(ap, int), 0);
          break;
        case 'u':
          len += itoa10Unsigned(putcf, va_arg(ap, unsigned int));
          break;
        case 'x':
        case 'X':
          len += itoa16(putcf, va_arg(ap, unsigned int), width, padChar);
          break;
        case 'l':
          // Look ahead for ll
          if (*fmt == 'l') {
            fmt++;
            len += handleLongLong(putcf, &fmt, va_arg(ap, unsigned long long int), width, padChar);
          } else {
            len += handleLong(putcf, &fmt, va_arg(ap, unsigned long int), width, padChar);
          }

          break;
        case 'f':
          num = va_arg(ap, double);
          if(num<0)
          {
            putcf('-');
            num = -num;
            len++;
          }
          len += itoa10(putcf, (int)num, 0);
          putcf('.'); len++;
          len += itoa10(putcf, (num - (int)num) * power(10,precision), precision);
          break;
        case 's':
          str = va_arg(ap, char* );
          while(*str)
          {
            putcf(*str++);
            len++;
          }
          break;
        case 'c':
          putcf((char)va_arg(ap, int));
          len++;
          break;
        default:
          break;
      }
    }
    else
    {
      putcf(*fmt++);
      len++;
    }
  }
  
  return len;
}

int eprintf(putc_t putcf, char * fmt, ...)
{
  va_list ap;
  int len;

  va_start(ap, fmt);
  len = evprintf(putcf, fmt, ap);
  va_end(ap);

  return len;
}
