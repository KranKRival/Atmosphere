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
#include <atmosphere/version.h>

#include "thermosphere.h"

static uintptr_t g_tms_ep = 0;

void thermosphere_detect(void *section, size_t sec_size) {
    if (thermosphere_is_present()) {
        panic_predefined(0xA);
    }
    
    thermosphere_header_t *header = (thermosphere_header_t *)section;
    if (header->magic != MAGIC_TMS0) {
        return;
    }
    
    /* Check the physical base matches the section. */
    if (header->phys_base != (uintptr_t)section) {
        panic_predefined(0xB);
    }
    
    /* Check the physical base is allowed. */
    if (!(THERMOSPHERE_ADDR_START <= header->phys_base && header->phys_base < THERMOSPHERE_ADDR_END)) {
        panic_predefined(0xC);
    }
    
    /* Check that thermosphere isn't too big. */
    if (sec_size >= THERMOSPHERE_SIZE_MAX - (header->phys_base - THERMOSPHERE_ADDR_START)) {
        panic_predefined(0xD);
    }
    
    g_tms_ep = (uintptr_t)header->phys_base + header->rel_ep;    
    /* Sanity check the relative entrypoint. */
    if (g_tms_ep < (uintptr_t)section || g_tms_ep >= (uintptr_t)section + sec_size) {
        panic_predefined(0xE);
    }
    
    /* Thermosphere present means we should deprivilege to EL2. */
    {
        uint64_t temp_reg;
        SET_SYSREG(spsr_el3, 0b1111 << 6 | 0b1001); /* DAIF + EL2h */
    }
}

bool thermosphere_is_present(void) {
    return g_tms_ep != 0;
}

uintptr_t thermosphere_get_entrypoint(void) {
    return g_tms_ep;
}
