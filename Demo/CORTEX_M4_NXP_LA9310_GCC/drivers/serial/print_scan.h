/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Copyright 2017, 2021 NXP
 */

#ifndef __PRINT_SCAN_H__
#define __PRINT_SCAN_H__

#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/*#define PRINTF_FLOAT_ENABLE   1 */
/*#define PRINT_MAX_COUNT       1 */
/*#define SCANF_FLOAT_ENABLE    1 */

#ifndef HUGE_VAL
    #define HUGE_VAL    ( 99.e99 )/*/wrong value */
#endif

typedef int (* PUTCHAR_FUNC)( int a,
                              void * b );

/*!
 * @brief This function outputs its parameters according to a formatted string.
 *
 * @note I/O is performed by calling given function pointer using following
 * (*func_ptr)(c,farg);
 *
 * @param[in] farg      Argument to func_ptr.
 * @param[in] func_ptr  Function to put character out.
 * @param[in] max_count Maximum character count for snprintf and vsnprintf.
 * Default value is 0 (unlimited size).
 * @param[in] fmt_ptr   Format string for printf.
 * @param[in] args_ptr  Arguments to printf.
 *
 * @return Number of characters
 * @return EOF (End Of File found.)
 */
int _doprint( void * farg,
              PUTCHAR_FUNC func_ptr,
              int max_count,
              char * fmt,
              va_list ap );

/*!
 * @brief Writes the character into the string located by the string pointer and
 * updates the string pointer.
 *
 * @param[in]      c            The character to put into the string.
 * @param[in, out] input_string This is an updated pointer to a string pointer.
 *
 * @return Character written into string.
 */
int _sputc( int c,
            void * input_string );

/*!
 * @brief Converts an input line of ASCII characters based upon a provided
 * string format.
 *
 * @param[in] line_ptr The input line of ASCII data.
 * @param[in] format   Format first points to the format string.
 * @param[in] args_ptr The list of parameters.
 *
 * @return Number of input items converted and assigned.
 * @return IO_EOF - When line_ptr is empty string "".
 */
int scan_prv( const char * line_ptr,
              char * format,
              va_list args_ptr );

#endif /* __PRINT_SCAN_H__*/
