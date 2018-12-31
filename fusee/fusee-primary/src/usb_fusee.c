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

#include <errno.h>
#include "lib/usb.h"
#include "lib/dmesg.h"

enum {
    REQUEST_READ_LOGS = 0xaa,
};

char output_buffer[4096];
extern usb_controller_t g_usb;

/**
 * Handle READ_LOGS requests, which read from the current ring buffer.
 */
static int fusee_handle_read_logs_request(usb_controller_t *controller, usb_transfer_stage_t stage)
{
    // If we've just completed the setup stage, schedule the output transaction for the data and ACK stages.
    if (stage == USB_TRANSFER_STAGE_SETUP) {

        bool clear = !!controller->last_setup.index;
        uint16_t length = sizeof(output_buffer);

        // Truncate our length to the maximum we have, 
        // or the maximum requested, whicherver is lesser.
        if (controller->last_setup.length < length) {
           length = controller->last_setup.length; 
        }

        // Get the data to be sent back.
        length = debug_ring_read(output_buffer, length, clear);

        // Respond during the data stage, and then ACK during the handshake stage.
        usb_transfer_schedule_in(controller, USB_ENDPOINT_CONTROL, output_buffer, length);
        usb_transfer_schedule_out(controller, USB_ENDPOINT_CONTROL, NULL, 0);
    }

    return 0;
}


/**
 * Handle the vendor requests supported by fusée.
 */
int fusee_handle_vendor_request(usb_controller_t *controller, usb_transfer_stage_t stage)
{
    // Delegate to the relevant function.
    switch (controller->last_setup.request) {
        case REQUEST_READ_LOGS:
            return fusee_handle_read_logs_request(controller, stage);

        default:
            usb_print(controller, "stalling unhandled vendor request (%x)", controller->last_setup.request);
            return EPIPE;    
    }
}
