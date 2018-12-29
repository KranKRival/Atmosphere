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

#include <stdint.h>

#include "panic.h"
#include "lib/printk.h"


/**
 * Inner panic handler; panics with a given color.
 * Implemented in assembly in entry.S.
 */
void _panic_with_color(uint32_t color);


/**
 * Triggered on an unrecoverable condition; prints an error message
 * and terminates execution.
 *
 * @param message If provided, a message to be printed using printk.
 * @param color_code The color code to be displayed post-reboot using the panic handler.
 */
void panic(const char *message, int color_code)
{
    static const uint32_t codes[0x10] = {COLOR_0, COLOR_1, COLOR_2, COLOR_3, COLOR_4, COLOR_5, COLOR_6, COLOR_7, COLOR_8, COLOR_9, COLOR_A, COLOR_B, COLOR_C, COLOR_D, COLOR_E, COLOR_F};

    // If we have a message, print it.
    if (message) {
        printk("\n\n");
        printk("-----------------------------------------------------------------\n");
        printk("PANIC: %s\n", message);
        printk("-----------------------------------------------------------------\n");
    }

    _panic_with_color(codes[color_code & 0xF]);
}
