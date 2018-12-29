/*
 * Copyright (c) 2018 Atmosphère-NX
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * Microlib:
 * simple support library providing simple stdlib equivalents
 * for teensy hypervisor stubs.
 */

#include "microlib.h"


/**
 * Quick (and not particularly performant) implementation of the standard
 * library's memcpy.
 */
void * memcpy(void * dest, const void * src, size_t n)
{
    const char * src_byte = src;
    char * dest_byte = dest;

    size_t i = 0;

    for(i = 0; i < n; ++i)
        dest_byte[i] = src_byte[i];

    return dest;
}

/**
 * Prints a string (synchronously) via serial.
 *
 * @param s The string to be printed; must be null terminated.
 */
extern int puts(const char * s)
{
    // FIXME: implementin terms of UART streams!
    (void)s;
    return 0;
}


/**
 * Determines the length of a string, scanning at most max characters.
 */
size_t strnlen(const char *s, size_t max)
{
    size_t n = 0;

    while(*s) {
        ++n;
        ++s;

        if(n == max)
            return n;
    }

    return n;
}


/**
 * Determines the length of a string.
 */
size_t strlen(const char *s)
{
    return strnlen(s, SIZE_MAX);
}

/**
 * Determines if two memory regiohns are equal, and returns the difference if they are not.
 */
int memcmp(const void *s1, const void *s2, size_t n)
{
    int i;

    const char * c1 = s1;
    const char * c2 = s2;

    for(i = 0; i < n; ++i)
        if(c1[i] != c2[i])
            return c1[i] - c2[i];

    return 0;
}

/**
 * Returns a pointer to the first instance of character 'c' in the given block of memory.
 */
void * memchr(const void *s, int c, size_t n)
{
    const unsigned char *p = s;
    int i;

    for(i = 0; i < n; ++i)
        if(p[i] == (unsigned char)c)
            return (void *)&(p[i]);

    return 0;
}

/**
 * Fills a given block with a byte value.
 */
void * memset(void *b, int c, size_t len)
{
    unsigned char *p = b;
    int i;

    for(i = 0; i < len; ++i)
        p[i] = c;

    return b;
}

