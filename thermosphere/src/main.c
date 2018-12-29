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
#include <stddef.h>
#include <string.h>
#include <stdint.h>

#include "regs.h"
#include "panic.h"
#include "lib/display/video_fb.h"
#include "lib/printk.h"


/**
 * Switches to EL1, and then calls main_el1.
 * Implemented in assembly in entry.S.
 */
void launch_horizon(void *entry_point, uint32_t argument);

/**
 * Reference to the EL2 vector table.
 * Note that the type here isn't reprsentative-- we just need the address of the label.
 */
extern uint64_t el2_vector_table;


/**
 * Clear out the system's bss.
 */
void _clear_bss(void)
{
    // These symbols don't actually have a meaningful type-- instead,
    // we care about the locations at which the linker /placed/ these
    // symbols, which happen to be at the start and end of the BSS.
    // We use chars here to make the math easy. :)
    extern uint32_t lds_bss_start, lds_bss_end;
    uint32_t *bss_word = &lds_bss_start;

    // Clear each word of the BSS.
    while (bss_word < &lds_bss_end) {
       *bss_word = 0;
       ++bss_word;
    }
}



/**
 * Core section of the stub-- sets up the hypervisor from up in EL2.
 */
int main(void *horizon_entry_point, uint32_t argument)
{
    // Read the currrent exception level...
    uint32_t el = get_current_el();

    /* Say hello. */
    printk("Welcome to Atmosph\xe8re Thermosph\xe8" "re!\n");
    printk("Running at EL%d.\n", el);

    // ... and ensure we're in EL2.
    if (el != 2) {
        panic("Thermosph\xe8" "re must be launched from EL2!", 0x1);
    }

    // Set up the vector table for EL2, so that the HVC instruction can be used
    // from EL1. This allows us to return to EL2 after starting the EL1 guest.
    set_vbar_el2(&el2_vector_table);

    // TODO:
    // Insert any setup you want done in EL2, here. For now, EL2 is set up
    // to do almost nothing-- it doesn't take control of any hardware,
    // and it hasn't set up any trap-to-hypervisor features.
    printk("\nSwitching to EL1...\n");
    launch_horizon(horizon_entry_point, argument);
}
