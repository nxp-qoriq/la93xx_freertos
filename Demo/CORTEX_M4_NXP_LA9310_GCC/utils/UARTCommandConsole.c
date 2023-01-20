/*
 * FreeRTOS Kernel V10.2.1
 * Copyright (C) 2017 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

/*
 * NOTE:  This file uses a third party USB CDC driver.
 */

/* Standard includes. */
#include "string.h"
#include "stdio.h"
#include "stdint.h"

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* Example includes. */
#include "FreeRTOS_CLI.h"
#include "UARTCommandConsole.h"

#include "debug_console.h"

/* Dimensions the buffer into which input characters are placed. */
#define cmdMAX_INPUT_SIZE    50

/* The maximum time in ticks to wait for the UART access mutex. */
#define cmdMAX_MUTEX_WAIT    ( 200 / portTICK_PERIOD_MS )

/*-----------------------------------------------------------*/

/*
 * The task that implements the command console processing.
 */
static void prvUARTCommandConsoleTask( void * pvParameters );

/*-----------------------------------------------------------*/

/* Const messages output by the command console. */
static const char * const pcWelcomeMessage = "FreeRTOS command server.\r\nType Help to view a list of registered commands.\r\n\r\n>";
static const char * const pcEndOfOutputMessage = "\r\n[Press ENTER to execute the previous command again]\r\n>";
static const char * const pcNewLine = "\r\n";

/*-----------------------------------------------------------*/

void vUARTCommandConsoleStart( uint16_t usStackSize,
                               unsigned portBASE_TYPE uxPriority )
{
    /* Create that task that handles the console itself. */
    xTaskCreate( prvUARTCommandConsoleTask, /* The task that implements the command console. */
                 "CLI",                     /* Text name assigned to the task.  This is just to assist debugging.  The kernel does not use this name itself. */
                 usStackSize,               /* The size of the stack allocated to the task. */
                 NULL,                      /* The parameter is not used, so NULL is passed. */
                 uxPriority,                /* The priority allocated to the task. */
                 NULL );                    /* A handle is not required, so just pass NULL. */
}
/*-----------------------------------------------------------*/

static void prvUARTCommandConsoleTask( void * pvParameters )
{
    char cRxedChar, * pcOutputString;
    int cInputIndex = 0;
    static char cInputString[ cmdMAX_INPUT_SIZE ], cLastInputString[ cmdMAX_INPUT_SIZE ];
    portBASE_TYPE xReturned;

    ( void ) pvParameters;

    /* Obtain the address of the output buffer.  Note there is no mutual
     * exclusion on this buffer as it is assumed only one command console
     * interface will be used at any one time. */
    pcOutputString = FreeRTOS_CLIGetOutputBuffer();

    /* Send the welcome message. */
    debug_printf( ( const char * ) pcWelcomeMessage );

    for( ; ; )
    {
        /* Only interested in reading one character at a time. */
        while( debug_getchar( ( unsigned char * ) &cRxedChar ) == pdFALSE )
        {
        }

        /* Echo the character back. */
        debug_putchar( cRxedChar );

        /* Was it the end of the line? */
        if( ( cRxedChar == '\n' ) || ( cRxedChar == '\r' ) )
        {
            /* Just to space the output from the input. */
            debug_printf( ( const char * ) pcNewLine );

            /* See if the command is empty, indicating that the last command is
             * to be executed again. */
            if( cInputIndex == 0 )
            {
                /* Copy the last command back into the input string. */
                strcpy( ( char * ) cInputString, ( char * ) cLastInputString );
            }

            /* Pass the received command to the command interpreter.  The
             * command interpreter is called repeatedly until it returns pdFALSE
             * (indicating there is no more output) as it might generate more than
             * one string. */
            do
            {
                /* Get the next output string from the command interpreter. */
                xReturned = FreeRTOS_CLIProcessCommand( cInputString, pcOutputString, configCOMMAND_INT_MAX_OUTPUT_SIZE );

                /* Write the generated string to the UART. */
                debug_printf( ( const char * ) pcOutputString );
            } while( xReturned != pdFALSE );

            /* All the strings generated by the input command have been sent.
             * Clear the input	string ready to receive the next command.  Remember
             * the command that was just processed first in case it is to be
             * processed again. */
            strcpy( ( char * ) cLastInputString, ( char * ) cInputString );
            cInputIndex = 0;
            memset( cInputString, 0x00, cmdMAX_INPUT_SIZE );
            debug_printf( ( const char * ) pcEndOfOutputMessage, strlen( ( char * ) pcEndOfOutputMessage ) );
        }
        else
        {
            if( cRxedChar == '\r' )
            {
                /* Ignore the character. */
            }
            else if( cRxedChar == '\b' )
            {
                /* Backspace was pressed.  Erase the last character in the
                 * string - if any. */
                if( cInputIndex > 0 )
                {
                    cInputIndex--;
                    cInputString[ cInputIndex ] = '\0';
                }
            }
            else
            {
                /* A character was entered.  Add it to the string
                 * entered so far.  When a \n is entered the complete
                 * string will be passed to the command interpreter. */
                if( ( cRxedChar >= ' ' ) && ( cRxedChar <= '~' ) )
                {
                    if( cInputIndex < cmdMAX_INPUT_SIZE )
                    {
                        cInputString[ cInputIndex ] = cRxedChar;
                        cInputIndex++;
                    }
                }
            }
        }
    }
}
/*-----------------------------------------------------------*/
