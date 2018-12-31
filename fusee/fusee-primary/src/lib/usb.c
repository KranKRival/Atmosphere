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

#include <stdarg.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include "log.h"
#include "usb.h"
#include "usb_standard.h"
#include "../car.h"

#define USB1_REGISTER_BASE  (0x7d000000)
#define USB_LOG_LEVEL SCREEN_LOG_LEVEL_MANDATORY

/**
 * Masks for various registers.
 */
enum {
    USB_PAGE_MASK                    = 0xfffff000,
    USB_PAGE_SIZE                    = 0x1000,

    USB_QH_OFFSET                    = 0x1000,
    USB_QH_IN_EP_OFFSET              = 0x40,

    // USBCMD register
    USB_USBCMD_RUN                   = (1 << 0),

    // USBSTS register
    USB_USBSTS_TRANSFER              = (1 << 0),
    USB_USBSTS_USB_ERROR             = (1 << 1),
    USB_USBSTS_PORT_CHANGE           = (1 << 2),
    USB_USBSTS_FRAME_LIST_ROLLOVER   = (1 << 3),
    USB_USBSTS_SYSTEM_ERROR          = (1 << 4),
    USB_USBSTS_ASYNCH_ADVANCE        = (1 << 5),
    USB_USBSTS_BUS_RESET_REQUEST     = (1 << 6),
    USB_USBSTS_SOF_RECEIVED          = (1 << 7),
    USB_USBSTS_DC_SUSPEND            = (1 << 8),
    USB_USBSTS_ULPI                  = (1 << 10),
    USB_USBSTS_ULPI_ALTERNATE        = (1 << 11),
    USB_USBSTS_HC_HALTED             = (1 << 12),
    USB_USBSTS_RECLAMATION           = (1 << 13),
    USB_USBSTS_PERIODIC_STATUS       = (1 << 14),
    USB_USBSTS_ASYNCH_STATUS         = (1 << 15),

    // USBINTR register
    USB_USBINTR_TRANSFER             = (1 << 0),
    USB_USBINTR_USB_ERROR            = (1 << 1),
    USB_USBINTR_PORT_CHANGE          = (1 << 2),
    USB_USBINTR_FRAME_LIST_ROLLOVER  = (1 << 3),
    USB_USBINTR_SYSTEM_ERROR         = (1 << 4),
    USB_USBINTR_ASYNCH_ADVANCE       = (1 << 5),
    USB_USBINTR_BUS_RESET_REQUEST    = (1 << 6),
    USB_USBINTR_SOF_RECEIVED         = (1 << 7),
    USB_USBINTR_DC_SUSPEND           = (1 << 8),
    USB_USBINTR_ULPI                 = (1 << 10),
    USB_USBINTR_ULPI_ALTERNATE       = (1 << 11),
    USB_USBINTR_HC_HALTED            = (1 << 12),
    USB_USBINTR_RECLAMATION          = (1 << 13),
    USB_USBINTR_PERIODIC_STATUS      = (1 << 14),
    USB_USBINTR_ASYNCH_STATUS        = (1 << 15),

    // PERIODICLISTBASE register
    USB_ADDR_SHIFT                   = 25,
    USB_SET_ADDR_DEFERRED            = (1 << 25),

    // ENDPTSETUPSTAT register
    USB_SETUP_PENDING                = (1 << 0),

    // ENDPTCONTROL registers
    USB_ENDPTCTRL_OUT_STALL                    = (1 << 0),
    USB_ENDPTCTRL_IN_STALL                     = (1 << 16),
};

/**
 * Debug print that's specialized for 
 */
void usb_print(usb_controller_t *controller, char *fmt, ...)
{
    va_list list;

    va_start(list, fmt);
    print(USB_LOG_LEVEL, "%s: ", controller->name);
    vprint(USB_LOG_LEVEL | SCREEN_LOG_LEVEL_NO_PREFIX, fmt, list);
    print(USB_LOG_LEVEL | SCREEN_LOG_LEVEL_NO_PREFIX, "\n");
    va_end(list);
}


/*
 * @return the index of the endpoints QH in the controller's data structure
 * given the endpoint's number and diretion.
 */
static inline int _endpoint_index_for_endpoint_number(uint8_t ep_number, bool is_in)
{
    return (ep_number << 1) | (is_in ? 1 : 0);
}


/**
 * @return the index of the endpoints QH in the controller's data structure
 * given the endpoint address.
 */
static inline int _endpoint_index_for_address(uint8_t ep_address) 
{
    return _endpoint_index_for_endpoint_number(ep_address & 0x7f, ep_address & 0x80);
}


/**
 * @return a pointer to the Queue Head for the relevant object
 */
static inline usb_queue_head_t *usb_get_queue_head(usb_controller_t *controller, uint8_t ep_number, bool is_in)
{
    uintptr_t address = (uintptr_t)controller->reg + USB_QH_OFFSET;

    // Find the endpoint pair for the given QH.
    address += (ep_number * (sizeof(usb_queue_head_t) * 2));

    // If this is an in endpoint, move to the second queue head in the pair.
    if (is_in) {
        address += sizeof(usb_queue_head_t);
    }

    // Convert the address to a pointer, and return it.
    return (usb_queue_head_t *)address;
}


/**
 * @returns A reference to the USB2 controller bank for the primary USB port on the Tegra X1.
 * TODO: support more than one USB controller?
 */
static usb_registers_t *usb_get_registers(void)
{
    return (usb_registers_t *)USB1_REGISTER_BASE; 
}


/**
 * Determine the maximum packet size for our endpoints, which depends on the
 * speed of the connected host.
 */
static int usb_determine_max_packet_size(usb_controller_t *controller, uint8_t ep_number)
{
    // FIXME: Figure this out from the port status register.
    // For now, assume high speed.
    if (ep_number) {
        return 512;
    } else {
        return 64;
    }
}


/**
 * Sets up the QH object for a given endpoint and direction.
 */
static void tegra_usb_set_up_endpoint_qh(usb_controller_t *controller, uint8_t ep_number, bool is_in)
{
    // Get a reference to the QH object for the relevant EP.
    usb_queue_head_t *qh = usb_get_queue_head(controller, ep_number, is_in); 

    // Clear the relevant QH structure.
    memset(qh, 0, sizeof(*qh));

    // If this QH belongs to the control endpoint, request notifications every time
    // we receive a setup packet. If interrupts are enabled, this will issue an interrupt;
    // otherwise, it will set a flag for us to detect on polling.
    if (ep_number == 0) {
        qh->interrupt_on_setup = 1;
    }

    // Ensure there's no transactions queued, currently.
    qh->overlay.next_dtd_pointer = DTD_TERMINATING;

    // Configure the maximum packet size for this endpoint, which should be
    // matched to the comms speed.
    qh->maximum_packet_length = usb_determine_max_packet_size(controller, ep_number);

    // FIXME:
    // For the previously configured endpoints, the endpt_controlN register is already
    // configured, so for now we'll skip setting those up. In the future, we should set this up,
    // so we can use more than EP0/1.
}

/**
 * Sets up the Queue Head for both 
 */
static void tegra_usb_set_up_endpoint_qhs(usb_controller_t *controller, uint8_t ep_number)
{
    // Set up both the OUT and in directions for the given endpoint.
    tegra_usb_set_up_endpoint_qh(controller, ep_number, false);
    tegra_usb_set_up_endpoint_qh(controller, ep_number, true);
}


/**
 * Allocates a new transfer descriptor from the freelist.
 *
 * @return the allocated TD, or NULL if none could be allocated.
 */
static volatile usb_transfer_descriptor_t *usb_allocate_td(usb_controller_t *controller)
{
    volatile usb_transfer_descriptor_t *td;

    // If the the freelist is empty, return NULL.
    if (controller->freelist_head == DTD_TERMINATING) {
        return NULL;
    }

    // Otherwise, advance the freelist by one.
    td = controller->freelist_head;
    controller->freelist_head = td->next_dtd_pointer;

    // ... and return the TD we extracted, zeroed-out.
    memset((void *)td, 0, sizeof(*td));
    return td;
}

/**
 *  Deallocates a transfer descriptor, returning it to the freelist.
 */
static void usb_free_td(volatile usb_controller_t *controller, usb_transfer_descriptor_t *td)
{
    // Stitch the TD into the start of the freelist.
    td->next_dtd_pointer = controller->freelist_head;
    controller->freelist_head = td;
}


/**
 * Initialize the allocation pool for USB transfer descriptor.
 */
static void usb_set_up_td_pool(usb_controller_t *controller)
{
    // First, set the freelist-head to indicate a termating DTD,
    // starting us with an empty freelist.
    controller->freelist_head = DTD_TERMINATING;

    // And add each of our TDs to the freelist.
    for (int i = 0; i < MAX_ALLOCATABLE_TDS; ++i) {
        usb_free_td(controller, &controller->td_pool[i]);
    }
}


/**
 *  Initializes the USB controller's asynchronous queue.
 */
static void usb_set_up_endpoints(usb_controller_t *controller)
{
    // Set up the QHs for each endpoint...
    for (int i = 0; i < MAX_SUPPORTED_ENDPOINTS; ++i) {
        tegra_usb_set_up_endpoint_qhs(controller, i);
    }

    // ... point the controller to that list...
    controller->reg->ep_list_base = (uint32_t)usb_get_queue_head(controller, 0, false);

    // ... and clear any pending setup events.
    controller->reg->endptsetupstat = controller->reg->endptsetupstat;
}

/**
 * Deatches the USB controller from the host, 
 * effectively stopping USB communications.
 */
void usb_detach(usb_controller_t *controller)
{
    controller->reg->usbcmd &= ~USB_USBCMD_RUN;
}


/**
 * Attaches the USB controller from the host, 
 * prompting it to issue a bus reset and request enumeration.
 */
void usb_attach(usb_controller_t *controller)
{
    controller->reg->usbcmd &= USB_USBCMD_RUN;
}


/**
 * Enables the standard set of events that we'll monitor.
 *
 * This method effectively enables interrupts -- but if interrupts
 * are masked, or not delivered to the BPMP, these can be polled with
 * usb_handle_events.
 */
void usb_enable_events(usb_controller_t *controller)
{
    controller->reg->usbintr =
        USB_USBSTS_TRANSFER           |
        USB_USBSTS_USB_ERROR          |
        USB_USBSTS_PORT_CHANGE        |
        USB_USBSTS_BUS_RESET_REQUEST  |
        USB_USBSTS_DC_SUSPEND;
}


/**
 * Initialize a new USB controller for use.
 * For now, we only support using the primary USB port, as it's the only exposed port on the Switch.
 *
 * @param controller The controller object to be initialized.
 */
int usb_controller_init(usb_controller_t *controller)
{
    // For now, always use the USB1 controller, as that's the only exposed port on the Switch.
    memset(controller, 0, sizeof(*controller));
    controller->name = "USB1"; 
    controller->reg  = usb_get_registers();
    controller->standard_request_handler = usb_handle_standard_request;

    // Ensure that the USB controller is fully clocked.
    clk_enable(CARDEVICE_USB);

    // Intialize our TD pool.
    usb_set_up_td_pool(controller);

    // Once we-'re handed the USB controller, it's already been configured for use by the bootROM.
    // This isn't quite right -- it has a bunch of references to structures in IRAM that are no longer
    // valid. We'll need to replace these with our own references before continuing.

    // First, we'll set up the "asynchronous queue" -- this essentially is a structure that points
    // to all of the USB data structures for transfers handled asynchronously -- i.e. the structures used
    // for control and bulk endpoints.
    usb_set_up_endpoints(controller);

    // Configure USB events.
    usb_enable_events(controller);

    // In the future, we may want to also set up the "periodic list", which is a structure that
    // points to all of the data structures for repeating ("periodic") transfers -- e.g. those used for
    // interrupt and isochronous endpoints.
    return 0;
}


/**
 * Returns an array of any pending interrupts, while marking the 
 * pending interrupts as serviced.
 *
 * @return A bitfield containing the pending interrupts.
 */
static uint32_t usb_get_status(usb_controller_t *controller)
{
    // Read the active USB interrupts.
    uint32_t status = controller->reg->usbsts & controller->reg->usbintr;

    // Mark the interrupt flags as serviced.
    controller->reg->usbsts = status;
    return status;
}


/**
 * Sets the address that the USB device will respnod to queries on; usually
 * as a result of a reset or a SET_ADDRESS request.
 *
 * @param address The address to apply.
 * @param deferred If set, the relevant set_address will not be applied until the
 *      successful completion of the active control request. See TRM 22.16.1.22.
 */
static void usb_set_address(usb_controller_t *controller, uint8_t address, bool deferred)
{
    uint32_t to_write = (address << USB_ADDR_SHIFT);

    if (deferred) {
        to_write |= USB_SET_ADDR_DEFERRED;
    }

    controller->reg->device_addr = to_write;
}


/**
 * Sets the device's active configuration.
 */
static void usb_set_configuration(usb_controller_t *controller, uint8_t index)
{
    // FIXME: TODO    
}



/**
 *  Handles a bus reset; usually in response to a host bus reset request.
 */
static void usb_bus_reset(usb_controller_t *controller)
{
    // FIXME: reset all endpoints; clearing them of TDs
    
    // Restore the device to its fresh, unenumerated state.
    usb_set_address(controller, 0, false);
    usb_set_configuration(controller, 0);
}


static void usb_handle_transfer_events(usb_controller_t *controller)
{

}


/**
 * Retreives a reference to the last setup packet recieved on the given endpoint.
 *
 * @param endpoint_number The endpoint number (not address) for the endpoint we're looking
 *      for setup packets on.
 * @return A copy of the setup data stored within the endpoint.
 */
static usb_setup_packet_t usb_retreive_setup_packet(usb_controller_t *controller)
{
    usb_queue_head_t *qh = usb_get_queue_head(controller, USB_ENDPOINT_CONTROL, false);
    usb_setup_packet_t setup = qh->setup_data;

    // Clear the relevant flag from the endpointsetupstatus.
    controller->reg->endptsetupstat |= USB_SETUP_PENDING;

    // ... and return the read setup data.
    return setup;
}


/**
 * @returns The handler function for the active control request on 
 * the given controller, or NULL if no handler exists.
 */
static usb_request_handler_t usb_handler_for_request(usb_controller_t *controller)
{
    switch (controller->last_setup.type) {
        case USB_REQUEST_TYPE_STANDARD: 
            return controller->standard_request_handler;
        case USB_REQUEST_TYPE_CLASS: 
            return controller->class_request_handler;
        case USB_REQUEST_TYPE_VENDOR: 
            return controller->vendor_request_handler;
        default:
            return NULL;
    }
}


/**
 * General top-level handler for USB control requests. 
 * Used to delegate control requests to subordinate functions.
 *
 * @param setup Reference to the setup packet to be handled.
 * @param stage Indicates the stage of the USB transaction that has most recently completed.
 */
void usb_handle_control_request(usb_controller_t *controller, usb_transfer_stage_t stage)
{
    int rc = EPIPE;

    // If possible, identify the handler for the given request type and execute it.
    usb_request_handler_t handler = usb_handler_for_request(controller);
    if (handler) {
        rc = handler(controller, stage);
    } else {
        usb_print(controller, "ignoring unhandled request (type %d)", controller->last_setup.type);
    }

    // If we weren't able to handle the given command,
    // stall the control endpoint.
    if (rc) {
        usb_endpoint_stall(controller, USB_ENDPOINT_CONTROL);
    }
}


/**
 * Checks for any outstanding setup events, and handles them.
 */
static void usb_handle_setup_events(usb_controller_t *controller)
{
    uint32_t setup_status = controller->reg->endptsetupstat;

    // If we have an outstanding setup packet, handle it.
    // Strictly, we may want to check all endpoints, but non-ep0 control endpoints 
    // are so rare (and so not useful) that we'll not support them.
    if (setup_status & USB_SETUP_PENDING) {

        // Copy the setup packet, and call our general control request handler.
        controller->last_setup = usb_retreive_setup_packet(controller);
        usb_handle_control_request(controller, USB_TRANSFER_STAGE_SETUP);
    }
}


/**
 * Main service routine for USB events.
 * Used to allow polled handling of USB events in environments where interrupts aren't supported.
 */
void usb_handle_events(usb_controller_t *controller)
{
    const uint32_t status = usb_get_status(controller);

    // Optimization: if there's no interrupts pending,
    // bail out.
    if (status == 0) {
       return; 
    }

    // USB transfer complete / short packet detected / setup
    if (status & USB_USBSTS_TRANSFER) {
        usb_handle_setup_events(controller);
        usb_handle_transfer_events(controller);
    }

    // USB error detected
    if (status & USB_USBSTS_USB_ERROR) {
        usb_print(controller, "Protocol error detected.");
    }

    // Port change detected
    if (status & USB_USBSTS_PORT_CHANGE) {
        usb_print(controller, "Host connected.");
    }

    // USB port reset requested
    if (status & USB_USBSTS_BUS_RESET_REQUEST) {
        usb_print(controller, "Host is requesting a bus reset.");
        usb_bus_reset(controller);
    }

    // Host disconnect or suspension
    if (status & USB_USBSTS_DC_SUSPEND) {
        usb_print(controller, "Connection suspended/disconnected.");
    }
}


/**
 * Issues a stall on the relevant endpoint.
 */
void usb_endpoint_stall(usb_controller_t *controller, uint32_t endpoint_number)
{
    // Stall both the IN and OUT endpoint; as per the spec endpoints should be
    // stalled in pairs.
    controller->reg->endptctrl[endpoint_number] |= (USB_ENDPTCTRL_OUT_STALL | USB_ENDPTCTRL_IN_STALL);

    // FIXME: if this is a protocol stall (i.e. if EP0 is stalled), flush EP0
}


/**
 * Adds a TD to the USB controller's active transaction queue.
 */
void usb_enqueue_td(usb_controller_t *controller, usb_queue_head_t *qh, volatile usb_transfer_descriptor_t *td)
{
    // Start off assuming the overlay QH is the only transfer in existence.
    volatile usb_transfer_descriptor_t *tail = &qh->overlay;

    // Iterate until we find the tail.
    while(tail->next_dtd_pointer != DTD_TERMINATING) {
        tail = tail->next_dtd_pointer;
    }

    // Link the transfer into the list.
    td->next_dtd_pointer = DTD_TERMINATING;
    tail->next_dtd_pointer = td;
}

/**
 * Primes a given endpoint, readying it for transfer.
 */
void usb_endpoint_prime(usb_controller_t *controller, 
        uint8_t endpoint_number, bool is_in)
{
    // Determine the bit in endptprime that should be set to affect the given endpoint.
    uint32_t prime_mask = (1 << endpoint_number);

    // If this is an IN transaction, shift the mask over to affect
    // the bits that affect IN transactions, rather than the default OUT.
    if (is_in) {
        prime_mask <<= 16;
    }

    controller->reg->endptprime |= prime_mask;
}


/**
 * Schedules a USB transaction, which sets up the hardware to perform an automatic transmit/recieve
 * when the host issues the appropriate token. Non-blocking, but retains a reference to the
 * relevant memory until the transaction is complete.
 *
 * To simulate a blocking transaction, call usb_wait_for_transfer_completion on the returned TD. 
 *
 * @param endpoint_number The endpoint number (not address) on which this transaction should be
 *  scheduled.
 * @param is_in True iff the given USB transcation is an IN transaction, and thus a _transmission_.
 * @param buffer The buffer that contains the data to be transmitted (IN transaction) or the the
 *      buffer to be populated with received data (OUT transaction).
 * @param length The maximum size of the buffer; either the desired amount of data to be transmitted
 *      (IN transaction) or the maximum amount we can recieve (OUT transaction).
 */
volatile usb_transfer_descriptor_t *usb_transfer_schedule(usb_controller_t *controller, 
        uint8_t endpoint_number, bool is_in, void *const buffer, uint32_t length)
{
    usb_queue_head_t *qh = usb_get_queue_head(controller, endpoint_number, is_in); 
    uint32_t data_pointer = (uint32_t)buffer;

    // Allocate a transfer descriptor.
    volatile usb_transfer_descriptor_t *td = usb_allocate_td(controller); 
    if (!td) {
        usb_print(controller, "out of transfer descriptors; could not schedule transaction");
        return NULL;
    }

    // Populate the transfer descriptor with the parameters of the transactions.
    td->next_dtd_pointer = DTD_TERMINATING;
    td->total_bytes = length;

    // Set the first page directly to our data pointer; this sets both the page address
    // and the register that stores the offset into the given page.
    td->buffer_pointer_page[0] = data_pointer;

    // And populate the page addresses for the remainder of the buffer.
    for (int i = 1; i < 5; ++i) {
        data_pointer += USB_PAGE_SIZE;
        td->buffer_pointer_page[i] = data_pointer & USB_PAGE_MASK;
    }

    // Ensure the USB controller generates an event when this TD is complete.
    td->int_on_complete = 1;

    // Mark the transaction as active.
    td->active = 1;

    // FIXME: determine if we need to prime the given endpoint before priming, below?
    // FIXME: general priming logic may be wrong

    // TODO: genericize me to disable interrupts, here
    usb_enqueue_td(controller, qh, td); 
    usb_endpoint_prime(controller, endpoint_number, is_in);
    // TODO: genericize me to restore interrupts, here

    return td;
}


/**
 * Schedules a USB IN transaction, which sets up the hardware to perform an automatic transmit
 * when the host issues the appropriate token. Non-blocking, but retains a reference to the
 * relevant memory until the transaction is complete.
 *
 * To simulate a blocking transaction, call usb_wait_for_transfer_completion on the returned TD. 
 *
 * @param endpoint_number The endpoint number (not address) on which this transaction should be
 *  scheduled.
 * @param buffer The buffer that contains the data to be transmitted.
 * @param length The desired amount to be transmitted.
 */
volatile usb_transfer_descriptor_t *usb_transfer_schedule_in(usb_controller_t *controller, 
        uint8_t endpoint_number, void *const buffer, uint32_t length)
{
    return usb_transfer_schedule(controller, endpoint_number, true, buffer, length); 
}



/**
 * Schedules a USB OUT transaction, which sets up the hardware to perform an automatic recieve
 * when the host issues the appropriate token. Non-blocking, but retains a reference to the
 * relevant memory until the transaction is complete.
 *
 * To simulate a blocking transaction, call usb_wait_for_transfer_completion on the returned TD. 
 *
 * @param endpoint_number The endpoint number (not address) on which this transaction should be
 *  scheduled.
 * @param buffer The buffer that contains the buffer to be populated with received data.
 * @param length The maximum amount of data that we should allow reciept of; usually the size of
 *      the buffer (and never larger).
 */
volatile usb_transfer_descriptor_t *usb_transfer_schedule_out(usb_controller_t *controller, 
        uint8_t endpoint_number, void *const buffer, uint32_t length)
{
    return usb_transfer_schedule(controller, endpoint_number, false, buffer, length); 
}


