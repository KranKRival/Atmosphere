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
 
#include "../utils.h"

#ifndef __FUSEE_USB_H__
#define __FUSEE_USB_H__

// Maximum number of endpoints supported by _this_ driver.
// For now, we'll support one, as the bootloader does; though we can theoretically up this
// to support up to 16.
#define MAX_SUPPORTED_ENDPOINTS (1)

// Maximum number of supported QHs.
// We have two QHs per endpoint, as by convention we separate each endpoint's queues into IN
// and OUT queues.
#define MAX_SUPPORTED_QHS (MAX_SUPPORTED_ENDPOINTS * 2)

// Magic pointer that indicates there are no additional dTDs after the current link.
#define DTD_TERMINATING ((void *)0x1)

// Total number of TDs that can be active at a given time; used for preallocation.
#define MAX_ALLOCATABLE_TDS (8)

enum {
    USB_ENDPOINT_CONTROL = 0,    
};


/**
 * Enumeration that describes the multiple stages of a USB control request.
 */
typedef enum {
    USB_TRANSFER_STAGE_SETUP,
    USB_TRANSFER_STAGE_DATA,
    USB_TRANSFER_STAGE_STATUS,
} usb_transfer_stage_t;


/**
 * Constants for the various USB transfer types.
 */
enum {
    USB_REQUEST_TYPE_STANDARD = 0,
    USB_REQUEST_TYPE_CLASS    = 1,
    USB_REQUEST_TYPE_VENDOR   = 2,
};


/** 
 * Constants for the various USB transfer recipient "contexts".
 */
enum {
    USB_RECIPIENT_DEVICE = 0,
    USB_RECIPIENT_INTERFACE = 1,
    USB_RECIPIENT_ENDPOINT = 2,
    USB_RECIPIENT_OTHER = 3,

};



/**
 *  Register layout for the Tegra USB2 controller.
 */ 
typedef struct PACKED {
    volatile uint32_t id_register;
    volatile uint32_t hw_host_reg;
    volatile uint32_t hw_dev_reg;
    volatile uint32_t tx_buf_ctl;
    volatile uint32_t rx_buf_ctl;
    volatile uint8_t undefined0[108];
    volatile uint32_t gptimer0_ctl;
    volatile uint32_t gptimer1_ld;
    volatile uint32_t gptimer1_ctl;
    volatile uint8_t undefined1[116];
    volatile uint16_t cap_length;
    volatile uint16_t hci_version;
    volatile uint32_t hci_structural;
    volatile uint32_t hci_caps;
    volatile uint8_t undefined2[20];
    volatile uint32_t udc_version;
    volatile uint32_t udc_caps;
    volatile uint32_t extsts;
    volatile uint32_t extintr;
    volatile uint32_t usbcmd;
    volatile uint32_t usbsts;
    volatile uint32_t usbintr;
    volatile uint32_t frame_index;
    volatile uint32_t undefined3;

    union {
        volatile uint32_t periodic_list_base;
        volatile uint32_t device_addr;
    };

    union {
        volatile uint32_t async_list_base;
        volatile uint32_t ep_list_base;
    };

    volatile uint32_t async_buf_status;

    volatile uint32_t burst_size;
    volatile uint32_t xmit_fill_tuning;
    volatile uint8_t undefined4[8];
    volatile uint32_t ulpi_viewport;
    volatile uint8_t undefined5[16];
    volatile uint32_t portsc1;
    volatile uint8_t undefined6[60];
    volatile uint32_t lpm_behavior_ctl;
    volatile uint8_t undefined7[60];
    volatile uint32_t otgsc;
    volatile uint32_t usbmode;
    volatile uint32_t undefined8;
    volatile uint32_t endptnak;
    volatile uint32_t endptnak_enable;
    volatile uint32_t endptsetupstat;
    volatile uint32_t endptprime;
    volatile uint32_t endptflush;
    volatile uint32_t endptstatus;
    volatile uint32_t endptcomplete;
    volatile uint32_t endptctrl[16];
    volatile uint8_t undefined9[420];
    volatile uint32_t suspend_ctl;
    volatile uint32_t vbus_sensors;
    volatile uint32_t vbus_wakeup_and_id;
    volatile uint32_t alt_vbus_status_id;
    volatile uint8_t gap410[2];
    volatile uint32_t field_410;
} usb_registers_t;


/**
 *  USB Setup packet.
 *  Adapted from the USB specification -- always 8 bytes in length.
 */
typedef struct PACKED {
    union {
        uint8_t request_type;
        struct {
            uint8_t recipient : 5;    
            uint8_t type : 2;    
            uint8_t transfer_direction : 1;    
        };
    };

	uint8_t request;
	union {
		struct {
			uint8_t value_l;
			uint8_t value_h;
		};
		uint16_t value;
	};
	union {
		struct {
			uint8_t index_l;
			uint8_t index_h;
		};
		uint16_t index;
	};
	union {
		struct {
			uint8_t length_l;
			uint8_t length_h;
		};
		uint16_t length;
	};
} usb_setup_packet_t;

/**
 * USB transfer descriptor.
 * Adapted from the EHCI specification, section 3.5.
 *
 * Describes a single USB data transfer.
 */
typedef struct usb_transfer_descriptor usb_transfer_descriptor_t;
struct PACKED usb_transfer_descriptor {

    // Pointer to the next transfer in the queue.
	volatile usb_transfer_descriptor_t *next_dtd_pointer;

    // Note that, unlike the standard EHCI transfer descriptors,
    // device transfer descriptors lack an alternate dtd pointer.
    // (The host can't send short packets, so this field would be meaningless.)

    // Status flags and the total bytes.
	struct {
		uint32_t ping_state_err : 1;
		uint32_t split_transaction_state : 1;
		uint32_t missed_uframe : 1;
		uint32_t transaction_error : 1;
		uint32_t babble : 1;
		uint32_t buffer_error : 1;
		uint32_t halted : 1;
		uint32_t active : 1;

		uint32_t pid_code : 2;
		uint32_t error_counter : 2;
		uint32_t current_page : 3;
		uint32_t int_on_complete : 1;
		uint32_t total_bytes : 15;
		uint32_t data_toggle : 1;
	};

    // Pointers to each page that contains either data to be sent (for an IN transaction)
    // or buffer for data to be recieved (for an OUT transaction).
	volatile uint32_t buffer_pointer_page[5];
	volatile uint32_t _reserved;
};



/**
 * Tegra Endpoint Queue Head (QH)
 * Structure that points to all active transactions for a given endpoint.
 *
 * Adapted From the ECHI specification, section 3.6; per the Tegra TRM.
 */
typedef struct PACKED {

	// DWord 1
	struct {
		uint32_t                              : 15;
		uint32_t interrupt_on_setup           : 1;
		uint32_t maximum_packet_length        : 11;
		uint32_t                              : 2;
		uint32_t auto_zlp                     : 1;
		uint32_t multiplier                   : 2;
	};

	// Dword 3
	uint32_t current_dtd;

    // This field is a "shadow" copy of the active transfer descriptor.
    // This is populated and accessed by hardware, and not something
    // we'll typically touch. It provides an active "working area" for
    // the hardware during transaction execution.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpragmas"
#pragma GCC diagnostic ignored "-Wpacked-not-aligned"
	usb_transfer_descriptor_t overlay;

    // Storage area for the most recently recieved setup packet.
    // In lieu of using the standard transfer mechanism, the hardware
    // automatically handles setup packets for us, and plops their data here
    // once they're complete.
    usb_setup_packet_t setup_data;

    // Unused.
	volatile uint32_t _reserved[4];

} __attribute__((packed, aligned(64))) usb_queue_head_t;
#pragma GCC diagnostic pop


typedef struct usb_controller usb_controller_t;

/**
 * Callback signature for a USB request handler.
 *
 * @param controller The USB controller associated with the relevant handler.
 * @param stage An indication of the USB control request stage that has just completed execution.
 *
 * @return 0 for success, or an error code to stall the relevant transaction
 */
typedef int (*usb_request_handler_t)(usb_controller_t *controller, usb_transfer_stage_t stage);


/**
 * Object representing a Tegra USB2 controller.
 */
struct usb_controller {

    // The name of the provided controller; used for debug output.
    char *name;

    // A reference to the register bank associated with the given controller.
    usb_registers_t *reg;

    // A holding buffer for the last setup packet recieved.
    usb_setup_packet_t last_setup;

    // Callback functions that allow us to handle control requests.
    usb_request_handler_t standard_request_handler;
    usb_request_handler_t class_request_handler;
    usb_request_handler_t vendor_request_handler;

    // A free-list used to track TDs while not in use.
    volatile usb_transfer_descriptor_t *freelist_head;

    // Pool of transfer descriptors to be allocated, aligned to 32-byte boundaries.
    struct {} __attribute__((aligned(32)));
    usb_transfer_descriptor_t td_pool[MAX_ALLOCATABLE_TDS];
};


/**
 * Debugging print for the USB driver.
 */
void usb_print(usb_controller_t *controller, char *fmt, ...);

/**
 * Initialize a new USB controller for use.
 * For now, we only support using the primary USB port, as it's the only exposed port on the Switch.
 *
 * @param controller The controller object to be initialized.
 */
int usb_controller_init(usb_controller_t *controller);

/**
 * Deatches the USB controller from the host, 
 * effectively stopping USB communications.
 */
void usb_detach(usb_controller_t *controller);

/**
 * Attaches the USB controller from the host, 
 * prompting it to issue a bus reset and request enumeration.
 */
void usb_attach(usb_controller_t *controller);

/**
 * Main service routine for USB events.
 * Used to allow polled handling of USB events in environments where interrupts aren't supported.
 */
void usb_handle_events(usb_controller_t *controller);

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
        uint8_t endpoint_number, bool is_in, void *const buffer, uint32_t length);


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
        uint8_t endpoint_number, void *const buffer, uint32_t length);

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
        uint8_t endpoint_number, void *const buffer, uint32_t length);



/**
 * Issues a stall on the relevant endpoint.
 */
void usb_endpoint_stall(usb_controller_t *controller, uint32_t endpoint);


/**
 * Register a function to handle a given type of USB control requests.
 */
static inline void usb_register_vendor_request_handler(usb_controller_t *controller, usb_request_handler_t handler)
{
    controller->vendor_request_handler = handler;
}
static inline void usb_register_class_request_handler(usb_controller_t *controller, usb_request_handler_t handler)
{
    controller->class_request_handler = handler;
}

/**
 * Override the standard request handler.
 */
static inline void usb_override_standard_request_handler(usb_controller_t *controller, usb_request_handler_t handler)
{
    controller->standard_request_handler = handler;
}


#endif
