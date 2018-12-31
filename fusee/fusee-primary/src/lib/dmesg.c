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


#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "dmesg.h"


#define MIN(a,b) \
    ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a < _b ? _a : _b; })

// FIXME: Don't assume this from Common -- pull this in from e.g. a configuration header file.
#ifndef CONFIG_DEBUG_BUFFER_SIZE
#define CONFIG_DEBUG_BUFFER_SIZE 4096
#undef CONFIG_DEBUG_INCLUDE_TRACE
#endif

/* Storage for the debug ringbuffer. */
static char debug_ring[CONFIG_DEBUG_BUFFER_SIZE];

unsigned int debug_read_index;
unsigned int debug_write_index;


/**
 * Initializes debugging support.
 */
void debug_ring_init(void)
{
    // TODO: support passing from primary to secondary?
    debug_read_index = 0;
    debug_write_index = 0;
}


/**
 * @return The total used size in the debug ring.
 */
size_t debug_ring_used_space(void)
{
	// Since we let the read and write buffers grow unbounded,
	// we never need to handle wraparound. Our size is simply the difference.
	return debug_write_index - debug_read_index;
}


/**
 * @return The amount of free bytes in the debug ring.
 */
size_t debug_ring_free_space(void)
{
	return sizeof(debug_ring) - debug_ring_used_space();
}


/**
 * @return True iff the debug ring is full.
 */
bool debug_ring_full(void)
{
	return debug_ring_used_space() == sizeof(debug_ring);
}


/**
 * @return True iff the debug ring is empty.
 */
bool debug_ring_empty(void)
{
	return debug_write_index == debug_read_index;
}


/** @return the wrapped version of the debug ring's write index */
static unsigned int debug_ring_write_index()
{
	return debug_write_index % sizeof(debug_ring);
}


/** @return the wrapped version of the debug ring's read index */
static unsigned int debug_ring_read_index()
{
	return debug_read_index % sizeof(debug_ring);
}


/**
 * Reads a set of raw bytes from the system's debug ringbuffer.
 *
 * @param buffer The buffer to be populated.
 * @param maximum The maximum length to be populated.
 */
unsigned int debug_ring_read(char *buffer, unsigned int maximum, bool clear)
{
	unsigned int immediate_length, wrapped_length;
	unsigned int length = MIN(maximum, debug_ring_used_space());

	// Figure out how much data is available following the read buffer, vs
	// available at the beginning of the ring.
	immediate_length = MIN(sizeof(debug_ring) - debug_ring_read_index(), length);
	wrapped_length = length - immediate_length;

	// Copy the immediate and wrapped sections.
	memcpy(buffer, &debug_ring[debug_ring_read_index()], immediate_length);
	memcpy(&buffer[immediate_length], debug_ring, wrapped_length);

	// Update the read pointer, if we're clearing as we read.
	if (clear)
		debug_read_index += length;

	// Return the length actually read.
	return length;
}


/**
 * Consumes a single line from the debug ring, freeing space without
 * leaving partial sentences in the ringbuffer.
 */
void debug_ring_reclaim_line(void)
{
	// Find the next newline after the read pointer.
	unsigned int walk_index;

	// Walk until we find a newline.
	for (walk_index = debug_read_index; walk_index < debug_write_index; ++walk_index) {

		// Figure out where the spot in the ring buffer is that contains our current index.
		unsigned int index_to_read = walk_index % sizeof(debug_ring);

		// If we've found the newline, advance the read pointer to this index,
		// and stop searching.
		if (debug_ring[index_to_read] == '\n') {
			debug_read_index = walk_index + 1;
			return;
		}
	}
}


/**
 * Writes a string to the system's debug ringbuffer.
 *
 * @param str The string to write.
 * @param length The length of the string to write.
 */
void debug_ring_write(const char *const str, unsigned int length)
{
	unsigned int immediate_length, wrapped_length;

	// If this can't fit in our ringbuffer _at all_, truncate it to the
	// size of the ringbuffer.
	if (length > sizeof(debug_ring))
		length = sizeof(debug_ring);

	// If we can't fit the given string, reclaim whole lines until we can.
	while (debug_ring_free_space() < length)
		debug_ring_reclaim_line();

	// Figure out how much of the string should go immediately following
	// the write pointer, vs wrapping around to the beginning of the buffer.
	immediate_length = MIN(sizeof(debug_ring) - debug_ring_write_index(), length);
	wrapped_length = length - immediate_length;

	// Copy the immediate and wrapped sections.
	memcpy(&debug_ring[debug_ring_write_index()], str, immediate_length);
	memcpy(debug_ring, &str[immediate_length], wrapped_length);

	// Update the write pointer.
	debug_write_index += length;
}


/**
 * Writes a string to the system's debug ringbuffer.
 *
 * @param str The string to write.
 * @param length The length of the string to write.
 */
void debug_ring_write_string(const char *const str)
{
	// Get the length of the string to be committed to the ringbuffer.
	// This is the length we'll need to copy.
	unsigned int length = strlen(str);

	// Write the string with the relevant length.
	debug_ring_write(str, length);
}
