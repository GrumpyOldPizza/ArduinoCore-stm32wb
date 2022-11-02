/*
 * Copyright (c) 2021-2021 Thomas Roell.  All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal with the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimers.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimers in the
 *     documentation and/or other materials provided with the distribution.
 *  3. Neither the name of Thomas Roell, nor the names of its contributors
 *     may be used to endorse or promote products derived from this Software
 *     without specific prior written permission.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * WITH THE SOFTWARE.
 */

#include "armv7m.h"
#include "rtos_api.h"

#include <stdlib.h>
#include <stdarg.h>

#define ARMV7M_RTT_ID_LENGTH    16

typedef struct _armv7m_rtt_control_t {
    uint8_t           id[ARMV7M_RTT_ID_LENGTH];
    uint32_t          channels[2];              // 1 == tx, 1 == rx
    const uint8_t     *tx_name;
    uint8_t           *tx_data;
    volatile uint32_t tx_size;
    volatile uint32_t tx_write;                 // target
    volatile uint32_t tx_read;                  // host
    volatile uint32_t tx_flags;
    volatile uint32_t tx_write_next;
    volatile uint32_t tx_lock;
} armv7m_rtt_control_t;

static __attribute__((section(".rtt_control"), used)) armv7m_rtt_control_t armv7m_rtt_control;

static __attribute__((section(".rtt_data"), used)) uint8_t armv7m_rtt_tx_data[10*1024];

static const uint32_t armv7m_rtt_tx_data_size = sizeof(armv7m_rtt_tx_data);

static const uint8_t armv7m_rtt_id[ARMV7M_RTT_ID_LENGTH] = { '\0', '\0', '\0', '\0', '\0', '\0', 'T', 'T', 'R', ' ', 'R', 'E', 'G', 'G', 'E', 'S' };

void __armv7m_rtt_initialize(void)
{
    uint32_t index;

    armv7m_rtt_control.id[0] = 0;
    armv7m_rtt_control.tx_size = 0;

    __DMB();

    armv7m_core_udelay(500);
    
    armv7m_rtt_control.channels[0] = 1;
    armv7m_rtt_control.channels[1] = 0;
    armv7m_rtt_control.tx_name = (const uint8_t*)"RTT";
    armv7m_rtt_control.tx_data = (uint8_t*)&armv7m_rtt_tx_data[0];
    armv7m_rtt_control.tx_write = 0;
    armv7m_rtt_control.tx_read = 0;
    armv7m_rtt_control.tx_flags = 0;

    armv7m_rtt_control.tx_write_next = 0;
    armv7m_rtt_control.tx_lock = 0;

    __DMB();

    for (index = 0; index < ARMV7M_RTT_ID_LENGTH; index++)
    {
        armv7m_rtt_control.id[(ARMV7M_RTT_ID_LENGTH -1) - index] = armv7m_rtt_id[index];
    }
    
    __DMB();

    armv7m_rtt_control.tx_size = armv7m_rtt_tx_data_size;

    __DMB();
}

static __attribute__((optimize("O3"))) uint32_t __svc_armv7m_rtt_write(const uint8_t *data, uint32_t size)
{
    uint32_t count, tx_read, tx_write, tx_write_next, tx_lock;

    tx_lock = __armv7m_atomic_swap(&armv7m_rtt_control.tx_lock, 1);
    
    do
    {
        tx_read = armv7m_rtt_control.tx_read;
        tx_write = armv7m_rtt_control.tx_write_next;
        
        if (tx_write >= tx_read)
        {
            count = ((armv7m_rtt_control.tx_size - tx_write) + tx_read) -1;
        }
        else
        {
            count = (tx_read - tx_write) -1;
        }
        
        if (size > count)
        {
            armv7m_rtt_control.tx_lock = tx_lock;
          
            return 0;
        }

        tx_write_next = tx_write + size;

        if (tx_write_next >= armv7m_rtt_control.tx_size)
        {
            tx_write_next -= armv7m_rtt_control.tx_size;
        }
    }
    while (armv7m_atomic_cas(&armv7m_rtt_control.tx_write_next, tx_write, tx_write_next) != tx_write);
    
    count = size;
        
    if (size > (armv7m_rtt_control.tx_size - tx_write))
    {
        size = armv7m_rtt_control.tx_size - tx_write;
    }

    memcpy(&armv7m_rtt_control.tx_data[tx_write], &data[0], size);

    if (size != count)
    {
        memcpy(&armv7m_rtt_control.tx_data[0], &data[size], (count - size));
    }

    if (!tx_lock)
    {
        tx_write = armv7m_rtt_control.tx_write;
        tx_write_next = armv7m_rtt_control.tx_write_next;

        armv7m_rtt_control.tx_lock = 0;    
        
        __DMB();

        __armv7m_atomic_cas(&armv7m_rtt_control.tx_write, tx_write, tx_write_next);

        __DMB();
    }
    
    return count;
}

uint32_t armv7m_rtt_write(const uint8_t *data, uint32_t size)
{
    if (armv7m_core_is_in_interrupt())
    {
        return __svc_armv7m_rtt_write(data, size);
    }

    return armv7m_svcall_2((uint32_t)&__svc_armv7m_rtt_write, (uint32_t)data, (uint32_t)size);
}

#define SCRATCH 64

void armv7m_rtt_printf(const char * format, ...)
{
    uint8_t scratch[SCRATCH], digits[10], *out, c, prefix;
    const uint8_t *s;
    int32_t i;
    uint32_t u, index, count, width, base;
    bool sign, left, zeros;
    const uint8_t *xlate;
    
    static const uint8_t xlate_LC[16] = "0123456789abcdef";
    static const uint8_t xlate_UC[16] = "0123456789ABCDEF";

    va_list ap;
    va_start(ap, format);

    out = &scratch[0];

    while (1)
    {
        c = *format++;

        if (c == '\0')
        {
            break;
        }
        
        if (c != '%')
        {
            goto output_character;
        }

        sign = false;
        left = false;
        zeros = false;
        prefix = '\0';
        width = 0;

        c = *format++;
        
        while (1)
        {
            if      (c == '+') { sign  = true; c = *format++; }
            else if (c == '-') { left  = true; c = *format++; }
            else if (c == '0') { zeros = true; c = *format++; }

            else
            {
                break;
            }
        }

        while (1)
        {
            if ((c >= '0') && (c <= '9')) { width = (width * 10) + (c - '0'); c = *format++; }

            else
            {
                break;
            }
        }
        
        switch (c) {
            
        case '\0':
            goto finish;
                
        case '%':
            goto output_character;

        case 'c':
            c = (uint8_t)va_arg(ap, uint32_t);
            goto output_character;

        case 's':
            s = va_arg(ap, const uint8_t *);
            goto output_string;

        case 'i':
        case 'd':
            base = 10;
            xlate = xlate_LC;
            
            i = va_arg(ap, int32_t);

            if (i < 0)
            {
                prefix = '-';
                
                u = -i;
            }
            else
            {
                if (sign)
                {
                    prefix = '+';
                }
                
                u = i;
            }
            goto output_unsigned;

        case 'u':
            base = 10;
            xlate = xlate_LC;
            
            u = va_arg(ap, uint32_t);
            goto output_unsigned;
            
        case 'x':
            base = 16;
            xlate = xlate_LC;
            
            u = va_arg(ap, uint32_t);
            goto output_unsigned;
            
        case 'X':
            base = 16;
            xlate = xlate_UC;
            
            u = va_arg(ap, uint32_t);
            goto output_unsigned;
            
        default:
            goto output_skip;
        }

    output_skip: {
            continue;
        }
        
    output_character: {
            *out++ = c;
            
            if ((out - &scratch[0]) == SCRATCH)
            {
                armv7m_rtt_write(&scratch[0], SCRATCH);
                
                out = &scratch[0];
            }
            
            continue;
        }
        
    output_string: {
            if (s)
            {
                while (*s)
                {
                    *out++ = *s++;
                    
                    if ((out - &scratch[0]) == SCRATCH)
                    {
                        armv7m_rtt_write(&scratch[0], SCRATCH);
                        
                        out = &scratch[0];
                    }
                }
            }
            
            continue;
        }
        
    output_unsigned: {
            count = 0;

            if (width > 10)
            {
                width = 10;
            }
            
            do
            {
                digits[count++] = xlate[u % base];
                u /= base;
            }
            while (u);

            if (zeros)
            {
                while (count < width)
                {
                    digits[count++] = '0';
                }
            }
            else
            {
                if (left)
                {
                    while (count < width)
                    {
                        *out++ = ' ';
                    
                        if ((out - &scratch[0]) == SCRATCH)
                        {
                            armv7m_rtt_write(&scratch[0], SCRATCH);
                            
                            out = &scratch[0];
                        }

                        width--;
                    }
                }
            }
            
            if (prefix)
            {
                *out++ = prefix;
                
                if ((out - &scratch[0]) == SCRATCH)
                {
                    armv7m_rtt_write(&scratch[0], SCRATCH);
                    
                    out = &scratch[0];
                }
            }

            for (index = 0; index < count; index++)
            {
                *out++ = digits[(count-1) - index];
                
                if ((out - &scratch[0]) == SCRATCH)
                {
                    armv7m_rtt_write(&scratch[0], SCRATCH);
                    
                    out = &scratch[0];
                }
            }

            if (!zeros && !left)
            {
                while (count < width)
                {
                    *out++ = ' ';
                    
                    if ((out - &scratch[0]) == SCRATCH)
                    {
                        armv7m_rtt_write(&scratch[0], SCRATCH);
                        
                        out = &scratch[0];
                    }
                    
                    width--;
                }
            }

            continue;
        }
    }

 finish:
    if (out != &scratch[0])
    {
        armv7m_rtt_write(&scratch[0], (out - &scratch[0]));
    }

    va_end(ap);
}


static void armv7m_rtt_hook_task_create(k_task_t *task, const char *name, uint32_t priority, void *stack_base, uint32_t stack_size)
{
    armv7m_rtt_printf("RTOS_TASK_CREATE(task=%08x, name=\"%s\", priority=%d, stack_base=%08x, stack_size=%d)\n", task, (name ? name : "<<NULL>>"), priority, stack_base, stack_size);
} 

static void armv7m_rtt_hook_task_destroy(k_task_t *task)
{
    armv7m_rtt_printf("RTOS_TASK_DESTROY(task=%08x)\n", task);
}

static void armv7m_rtt_hook_task_exit(void)
{
    armv7m_rtt_printf("RTOS_TASK_EXIT()\n");
}

static void armv7m_rtt_hook_task_terminate(k_task_t *task)
{
    armv7m_rtt_printf("RTOS_TASK_TERMINATE(task=%08x)\n", task);
}

static void armv7m_rtt_hook_task_block(k_task_t *task, uint32_t cause)
{
    armv7m_rtt_printf("RTOS_TASK_BLOCK(task=%08x, cause=%02x)\n", task, cause);
}

static void armv7m_rtt_hook_task_ready(k_task_t *task)
{
    armv7m_rtt_printf("RTOS_TASK_READY(task=%08x)\n", task);
}

static void armv7m_rtt_hook_task_run(k_task_t *task)
{
    armv7m_rtt_printf("RTOS_TASK_RUN(task=%08x)\n", task);
}

const k_hook_table_t armv7m_rtt_hook_table =
{
    .task_create    = armv7m_rtt_hook_task_create,
    .task_destroy   = armv7m_rtt_hook_task_destroy,
    .task_exit      = armv7m_rtt_hook_task_exit,      
    .task_terminate = armv7m_rtt_hook_task_terminate,
    .task_block     = armv7m_rtt_hook_task_block,
    .task_ready     = armv7m_rtt_hook_task_ready,
    .task_run       = armv7m_rtt_hook_task_run,
};

void RTT_SWIHandler(void)
{
    __DMB();

    armv7m_rtt_control.tx_write = armv7m_rtt_control.tx_write_next;

    __DMB();
}
