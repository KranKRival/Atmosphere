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
 */

#ifndef __DMESG_H__
#define __DMESG_H__

#include <stdarg.h>
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <errno.h>


/* Initialize debugging. */
void debug_ring_init(void);


/**
 * @return The total used size in the debug ring.
 */
size_t debug_ring_used_space(void);


/**
 * @return The amount of free bytes in the debug ring.
 */
size_t debug_ring_free_space(void);

/**
 * @return True iff the debug ring is full.
 */
bool debug_ring_full(void);


/**
 * @return True iff the debug ring is empty.
 */
bool debug_ring_empty(void);



/**
 * Reads a set of raw bytes from the system's debug ringbuffer.
 *
 * @param buffer The buffer to be populated.
 * @param maximum The maximum length to be populated.
 */
unsigned int debug_ring_read(char *buffer, unsigned int maximum, bool clear);


/**
 * Consumes a single line from the debug ring, freeing space without
 * leaving partial sentences in the ringbuffer.
 */
void debug_ring_reclaim_line(void);


/**
 * Writes a string to the system's debug ringbuffer.
 *
 * @param str The string to write.
 * @param length The length of the string to write.
 */
void debug_ring_write(const char *const str, unsigned int length);


/**
 * Writes a string to the system's debug ringbuffer.
 *
 * @param str The string to write.
 * @param length The length of the string to write.
 */
void debug_ring_write_string(const char *const str);

#endif
