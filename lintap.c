#include <linux/kernel.h>
#include <linux/kernel.h>
#include <linux/moduleparam.h>
#include <linux/parport.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/delay.h>

// TODO - this has been designed with multiple parallel ports in mind, which will probably
// never happen.  If it does however, the timer is shared and needs to be changed so that
// it is suspended when ever the list of lintap devices is modified.  Otherwise, it could
// interrupt when the list is being altered and cause some nasty error.

// TODO - replace timer active flag with kernel function to check for pending timer event

/* Module Information */

MODULE_AUTHOR("JS");
MODULE_DESCRIPTION("Reengineered Megatap device for 4 PSX pads, no shock, kernel 3.0+");
MODULE_LICENSE("GPL");

/* Constants Definitions */

#define LINTAP_VERSION 15032015

#define PSX_COMMAND             0x01		//00000001b	//bit 0 of data register
#define PSX_SELECT_ALL          0x02		//00000010b	//bit 2 of data register
#define PSX_CLOCK               0x04		//00000100b	//bit 2 of data register

#define PSX_DATA_0              0x08		//00001000b	//bit 3 of status register
#define PSX_DATA_1              0x10		//00010000b	//bit 4 of status register
#define PSX_DATA_2              0x20		//00100000b	//bit 5 of status register
#define PSX_DATA_3              0x40		//01000000b	//bit 6 of status register

#define PSX_ACKNOWLEDGE			0x80		//10000000b	//bit 7 of status register

#define PSX_COMMAND_START		0x01		//init start state command
#define PSX_COMMAND_TRANSFER	0x42		//request pad status command
#define PSX_NORMAL_PAD_ID		0x41		//what pad should return in response to start command
#define PSX_NORMAL_STATUS		0x5a		//what pad should return in response to status request

#define PSX_BIT_DELAY				5			//number of useconds
#define PSX_CMD_DELAY				10

#define MAX_PADS				4			//maximum number of pads that can be connected
#define MAX_BUTTONS				10			//number of buttons on pad

#define PSX_PAD_ID				7

#define GC_REFRESH_TIME	HZ/100

/* End Constants */

// Debug Macro
#ifdef DEBUG
#define debugk(...) printk(__VA_ARGS__)
#else
#define debugk(...) {;}
#endif // DEBUG

/* Parameter Configuration */

static unsigned short bit_delay = PSX_BIT_DELAY;
static unsigned short cmd_delay = PSX_CMD_DELAY;
// Set up two parameters which are world readable in sysfs
module_param(bit_delay, ushort, 0444);
MODULE_PARM_DESC(bit_delay, "Delay between bits when receiving reading pad status (usecs).  Default 5");
module_param(cmd_delay, ushort, 0444);
MODULE_PARM_DESC(cmd_delay, "Delay after sending command to PSX pad (usecs).  Default 10");

/* User defined types */
typedef enum PSX_Status_Mask {PSX_LEFT = 0x0080, PSX_DOWN = 0x0040, PSX_RIGHT = 0x0020,
	PSX_UP = 0x0010, PSX_START = 0x0008, PSX_SELECT = 0x0001, PSX_SQUARE = 0x8000, PSX_CROSS = 0x4000,
	PSX_CIRCLE = 0x2000, PSX_TRIANGLE = 0x1000, PSX_RIGHT1 = 0x0800, PSX_LEFT1 = 0x0400,
	PSX_RIGHT2 = 0x0200, PSX_LEFT2 = 0x0100} psx_status_mask;
/* End types */

/* Data Structures */

struct psx_pad {
    int pad_num;							//number of pad 0-4
	uint8_t pad_id;					//bits 7-4 = controller type, 3-0 = transfer byte(normal pad = 1)
	uint8_t pad_status;				//should normally be 0x5a ('Z')
    // 16bit button_status maps to the two bytes for button status bytes 1 & 2
    uint8_t button_status[2]; //| L  | DW | R  | UP | ST |  1 |  1 |SEL | [] |  X |  O | <| | R1 | L1 | R2 | L2 |
	struct lintap_device* lintap;			//lintap this pad is attached to
	struct input_dev* dev;					//kernel device pad is mapped to
	int use_count;								//counts the number of times the device is opened/closed.  A positive count means in use
};


struct lintap_device {
	bool port_claimed;						//TRUE if port has been claimed for use.  Should only be claimed if devices in use
	struct pardevice* port_dev;				//pointer pardevice data structure
	struct lintap_device* next; 			//next registered with driver
	struct psx_pad pads[MAX_PADS];			//array of structures for available pads.  If NULL then no pad connected to respective slot
};


/* Function Prototypes */

static void attach_to_parport(struct parport* port);
static void detach_from_parport(struct parport* port);

static int __init lintap_module_init(void);
static void __exit lintap_module_exit(void);

/* End Prototypes */

/* Global Variables */
static struct parport_driver lintap_driver = {
	"LinTap",	//Name of parallel port driver
	attach_to_parport, //function pointer for attach free port
	detach_from_parport, //function pointer for detach current port
	{ NULL, NULL } //list_head structure with NULL pointer for next and prev driver
};

static bool registered_with_parport = false;  // Indicates that driver has been registered with parralel port manager
static struct lintap_device* lintap_list = NULL; //list of all device registrations
static const uint16_t psxpad_button_events[MAX_BUTTONS] = { BTN_TL2, BTN_TR2, BTN_TL, BTN_TR, BTN_Y, BTN_X, BTN_B, BTN_A , BTN_START, BTN_SELECT };
static const uint8_t psxpad_data_masks[MAX_PADS] = { PSX_DATA_0, PSX_DATA_1, PSX_DATA_2, PSX_DATA_3 };
static const char pad_name[] = "PSX Controller";
static struct timer_list timer;	// timer function info.  Shared by all instances of lintap.  Initialised when module is loaded
static bool timer_active = false;  // Indicates timer is in use

/* End Global Variables */

/* Functions */

// returns true if there are still pads in use attached to respective port
static bool check_port_required(const struct lintap_device* lintap)
{
	int pad_count = 0;
	bool required = false;

	while (pad_count < MAX_PADS && !required)
    {
		debugk("Use count for pad %d is %d\n", lintap->pads[pad_count].pad_num, lintap->pads[pad_count].use_count);
		if (lintap->pads[pad_count].use_count > 0) { required = true; }
		pad_count++;
	}

	return required;
}

// returns true if any lintap device has claim on any of the parallel ports
static bool check_timer_required(const struct lintap_device* lintap)
{
	bool required = false;

	while (lintap != NULL && !required)
    {
		if (check_port_required(lintap)) { required = true; } // check if port is required - then timer is required
		lintap = lintap->next;  //loop through next lintap port device driver
	}

	return required;
}

// Reschedule timer event
static void enable_lintap_timer(void)
{
    // Set enabled flag first before adding timer, so that flag is set before timer interrupts anything
    timer_active = true;
    // Set enabled flag first before adding timer, so that flag is set before timer interrupts anything
    mod_timer(&timer, jiffies + GC_REFRESH_TIME);
    debugk("Activating timer function\n");
}

static void disable_lintap_timer(void)
{
    del_timer_sync(&timer);
    // Set disabled flag first after deleting timer, so that flag truly reflects timer state
    timer_active = false;
    // Set disabled flag first after deleting timer, so that flag truly reflects timer state
    debugk("Timer deactivated\n");
}

static void psxpads_select(const struct lintap_device* lintap) {
	struct parport* port = lintap->port_dev->port;

	// send select high to all pads, then lower select edge
	parport_write_data(port, PSX_CLOCK|PSX_SELECT_ALL);
	udelay(bit_delay);	// wait some time for parallel port
	// set selected pad low and clock high and command high
	parport_write_data(port, PSX_CLOCK);
	udelay(bit_delay);	//wait some time for parallel port
}

// sends select high to all pads, sets clock high
static void psxpads_deselect(const struct lintap_device* lintap) {

	parport_write_data(lintap->port_dev->port, PSX_CLOCK|PSX_SELECT_ALL);
	//udelay(PSX_DELAY); // DOESN'T seemt to be required	//wait some time for parallel port
}

// Claim the parallel port that the lintap device is registererd against
// Enable the timer if it isn't already
static bool lintap_claim_port(struct lintap_device* lintap)
{
    if (parport_claim(lintap->port_dev) == 0)
    {
        lintap->port_claimed = true;
        debugk("Parport %s claimed\n", lintap->port_dev->port->name);
        if (!timer_active) { enable_lintap_timer(); }
        return true;
    }
    else { return false; }
}

// Release the parallel port that the lintap device is registererd against
static void lintap_release_port(struct lintap_device* lintap)
{
    debugk("Releasing parport %s\n", lintap->port_dev->port->name);
    parport_release(lintap->port_dev); //release the port
    lintap->port_claimed = false; //make note of this with status flag
}

// Claim parallel port if not already claimed
static int psxpad_open(struct input_dev* dev)
{
	struct psx_pad* pad = (struct psx_pad*)input_get_drvdata(dev); //get handle to respective pad structure for device
	debugk("Call to pad open for pad %d\n", pad->pad_num);

	pad->use_count++;
    // if use count was zero and is now one, port needs to be claimed to use pad if it hasn't already been claimed
	if (pad->use_count == 1 && !pad->lintap->port_claimed)
    {
        if (!lintap_claim_port(pad->lintap))
        {
            // If claiming port failed, set pad use count to zero and return busy
            pad->use_count = 0;
            return -EBUSY;
        }
    }
	return 0;
}

// Close pad device.  Decrement use count for the pad.  If use count reaches zero, then check
// if any other pads on same lintap device are in use.  If not, then release the parallel port
// and check if any other lintaps still need timer.  If not, disable timer function
static void psxpad_close(struct input_dev* dev) {
	struct psx_pad* pad = (struct psx_pad*)input_get_drvdata(dev); //dev->private;  //get handle to respective pad structure for device
	struct lintap_device* lintap = pad->lintap;
	debugk("Call to pad close for pad %d\n", pad->pad_num);
	pad->use_count--;
	if (pad->use_count == 0 && lintap->port_claimed && !check_port_required(lintap))
    {
        // Stop timer first before releasing parallel port
        if (timer_active && !check_timer_required(lintap_list)) { disable_lintap_timer(); }
        lintap_release_port(lintap);
    }
}

static void psxpads_send_command(const struct lintap_device* lintap, uint8_t command, uint8_t store[MAX_PADS]) {
	//takes a byte command as an argument, sends bit by bit on command pin, and returns resultant 4 data bytes for all 4 pads
	int bit_count;
	struct parport* port = lintap->port_dev->port;

	memset(store, 0, MAX_PADS);
    debugk("Sending command %x\n", command);

	for (bit_count = 0; bit_count < 8; bit_count++)
    {
		uint8_t data = 0;
		uint8_t commbyte = command & 0x01;
		int pad_count = 0;
		parport_write_data(port, commbyte); //transmit least significant bit of command, on data pin 0clock is low
		udelay(bit_delay);	//wait per usual

		data = parport_read_status(port); //read next stream of bits coming from all pads

		for (pad_count = 0; pad_count < MAX_PADS; pad_count++) {
			store[pad_count] |= ( data & psxpad_data_masks[pad_count]) ? (1 << bit_count) : 0;
		}

		commbyte |= PSX_CLOCK;	//command bit must be sent again? but with clock high again
		parport_write_data(port, commbyte);  //set clock high
		udelay(bit_delay);
		command >>= 1; //shift command once right
	}

	udelay(cmd_delay);

}

// Read the ID and working status of the pad, and the status of all axes and buttons
// from the device into the psx_pad structure
static void psxpads_read_status(struct lintap_device* lintap) {
	uint8_t id[MAX_PADS],status[MAX_PADS],buttons_1[MAX_PADS],buttons_2[MAX_PADS],temp[MAX_PADS];
	int pad_count;

	psxpads_select(lintap);

	debugk("Sending start command\n");

	psxpads_send_command(lintap, PSX_COMMAND_START, temp);			//get pads' attentions

	psxpads_send_command(lintap, PSX_COMMAND_TRANSFER, id);  //request status transfer from all pads
	psxpads_send_command(lintap, 0, status);			//get pad status
	psxpads_send_command(lintap, 0, buttons_1);		//get first lot of buttons
	psxpads_send_command(lintap, 0, buttons_2);		//get second load of buttons

	for (pad_count = 0;pad_count < MAX_PADS; pad_count++)
    {
        struct psx_pad* pad = &lintap->pads[pad_count];

        pad->pad_id = id[pad_count];
        pad->pad_status = status[pad_count];
        // check if we have the right pad type and it is present, and read status from pad
        if (pad->pad_id == PSX_NORMAL_PAD_ID && pad->pad_status == PSX_NORMAL_STATUS)
        {
			pad->button_status[0] = buttons_1[pad_count];
			pad->button_status[1] = buttons_2[pad_count];
		}
		else
        {
            pad->button_status[0] = pad->button_status[1] = 0xFF; // Set all bits high (pretend there is no pad)
        }
    }

	psxpads_deselect(lintap);
}

static struct input_dev* input_device_new(const char *name, unsigned bus, unsigned vendor, unsigned prod, unsigned ver,
		void *private, void *open_func, void *close_func) {
	//Creates a new kernel input device and returns pointer
	struct input_dev *new_dev = input_allocate_device();

	if (new_dev!=NULL)
    {
		input_set_drvdata(new_dev, private); //give the input device structure a handle to the parent pad structure for callback access
		new_dev->open = open_func;
		new_dev->close = close_func;

		new_dev->name = name;
		new_dev->id.bustype = (__u16)bus;
		new_dev->id.vendor = (__u16)vendor;
    	new_dev->id.product = (__u16)prod;
    	new_dev->id.version = (__u16)ver;
	}

	return new_dev;
}


static bool register_psxpad_device(struct psx_pad* pad) {  //returns TRUE if successfull, else false
	//Create the input device structure
	pad->dev = input_device_new(pad_name, BUS_PARPORT, 0x0001, PSX_PAD_ID, LINTAP_VERSION, pad, psxpad_open, psxpad_close);

	if (pad->dev != NULL)
    {
		int event_count;
		//Assign event information
		pad->dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);  //set device capable of key(button) and absolute movement events

		for (event_count=0;event_count<MAX_BUTTONS;event_count++) {
			__set_bit(psxpad_button_events[event_count], pad->dev->keybit);  //set possible key events for this input device
		}

        // New ABS info for kernel 3
        input_set_abs_params(pad->dev, ABS_X, -255, 255, 0, 0);
        input_set_abs_params(pad->dev, ABS_Y, -255, 255, 0, 0);

        input_register_device(pad->dev);
		debugk("Registered pad input device\n");

		return true;
	}
	else { return false; }
}

static void lintap_timer_func(unsigned long private) {
	struct lintap_device *lintap = *((struct lintap_device**)private);

    while (lintap != NULL)
    {
   		debugk("Timer running\n");
		if (lintap->port_claimed)
        { //only do input checking if the parallel port has been claimed for use
            int pad_count = 0;

			psxpads_read_status(lintap); //get status from pad
			for (pad_count = 0; pad_count < MAX_PADS; pad_count++)
            {
				int button_count = 0;
				const struct psx_pad* pad = &lintap->pads[pad_count];   //get handle to current pad
				struct input_dev* dev = pad->dev;				//get pointer to device structure
				const uint16_t button_status = *((uint16_t*)pad->button_status);
                const int abs_x = 0 + (button_status & PSX_RIGHT ? 0 : 255) - (button_status & PSX_LEFT ? 0 : 255);
                const int abs_y = 0 + (button_status & PSX_DOWN ? 0 : 255) - (button_status & PSX_UP ? 0 : 255);

                debugk("lintap pad num: %d, axis x: %d, axis y: %d\n", pad_count, abs_x, abs_y);

                input_report_abs(dev, ABS_X, abs_x);
				input_report_abs(dev, ABS_Y, abs_y);

				for (button_count = 0; button_count < MAX_BUTTONS - 2; button_count++) {
					input_report_key(dev, psxpad_button_events[button_count], ~button_status & (0x0100 << button_count));
				}

				input_report_key(dev, psxpad_button_events[MAX_BUTTONS - 2],  ~button_status & PSX_START);
				input_report_key(dev, psxpad_button_events[MAX_BUTTONS - 1], ~button_status & PSX_SELECT);

				input_sync(dev);
   			}
		}

		lintap = lintap->next;

	} //End while loop for ports

	enable_lintap_timer(); //reactivate timer function
}

static void init_psxpads(struct lintap_device *lintap) {
	int pad_count;

	for (pad_count = 0; pad_count<MAX_PADS; pad_count++) {
		struct psx_pad *new_pad = &lintap->pads[pad_count];

		if (new_pad) {
			memset(new_pad, 0, sizeof(struct psx_pad));
			new_pad->lintap = lintap;	//make the pad belong to the current lintap
			new_pad->pad_num = pad_count;
			register_psxpad_device(new_pad);
        }
	}
}

// Create timer with address of timer_list (pointer to pointer to list object)
// THIS FUNCTION MARKED __init so that it will be dropped from memory after module initialisation finished
static void __init init_lintap_timer(struct timer_list* ptr_timer, struct lintap_device** lintap_list_address)
{
    memset(ptr_timer, 0, sizeof(struct timer_list));
    init_timer_on_stack(ptr_timer);
    ptr_timer->data = (long)lintap_list_address;
    ptr_timer->function = lintap_timer_func;
}


static void attach_to_parport(struct parport* port) {
	struct lintap_device* new_lintap = (struct lintap_device*)kmalloc(sizeof(struct lintap_device), GFP_KERNEL);
#ifdef DEBUG
	printk("New parallel port is available for register!\n"); printk(" - Name: %s\n",port->name); printk(" - Number: %d\n",port->number);
	printk("Available modes :");if (port->modes&PARPORT_MODE_PCSPP)	{ printk("SPP "); } if (port->modes&PARPORT_MODE_TRISTATE) { printk("TRISTATE "); }
	if (port->modes&PARPORT_MODE_COMPAT) { printk("COMPAT "); } if (port->modes&PARPORT_MODE_EPP) { printk("EPP "); }
	if (port->modes&PARPORT_MODE_ECP) { printk("ECP "); } if (port->modes&PARPORT_MODE_DMA) { printk("DMA "); }	printk("\n");
#endif //DEBUG

	if (new_lintap != NULL) { //kmalloc for new structure succeeded
		memset(new_lintap, 0, sizeof(struct lintap_device));  //Zero out new allocated structure
		new_lintap->port_dev = parport_register_device(port, "Lintap", NULL, NULL, NULL, 0, new_lintap); //attempt to register device
		if (new_lintap->port_dev != NULL) { //successful registration of driver with port
			debugk("Successful registration of device\n");
            init_psxpads(new_lintap);
			new_lintap->next = lintap_list;
			lintap_list = new_lintap;	//simply prepend new lintap record to existing list
		} else {
			debugk("Failed to register device: \n");
			kfree(new_lintap); //free memory
		}

	} else {
		debugk("Error - could not allocate memory for lintap device\n");
	}
}

static void detach_from_parport(struct parport *port) {  //handle detachment from parport due to no devices being registered */
	// TODO - This needs to be implemented for multiple parallel ports or if weird stuff happens.
	// Should unregister the lintap device attached with the port
	debugk("detach_from_parport() - Lintap is being detached.  No problem.\n");
}

// Set up the module.  Register it as a parallel port driver with handler functions to attach
// and detach to and from parallel ports.  Initialise timer strucure with address
// of lintap list which will be later used to set the call back timer
static int __init lintap_module_init(void) {
    debugk("!!!!!!!!LINTAP INIT!!!!!\n");
    // INITALISE TIMER FIRST BEFORE REGISTERING WITH PARPORT SO THAT IT IS READY AS SOON AS REGISTRATION IS COMPLETE
	init_lintap_timer(&timer, &lintap_list);
	// INITALISE TIMER FIRST BEFORE REGISTERING WITH PARPORT SO THAT IT IS READY AS SOON AS REGISTRATION IS COMPLETE
	if (parport_register_driver(&lintap_driver)) {
		debugk("Error registering Lintap parport driver.\n");
	} else {
		debugk("Lintap parport driver registered successfully.\n");
		registered_with_parport = true;
	}
	return 0;
}

static void __exit lintap_module_exit(void)
{
    // deactive timer before doing anything else!
    disable_lintap_timer();
    // deactive timer before doing anything else!

    if (registered_with_parport) {
		struct lintap_device* ptr_lintap = lintap_list; //get handle to start of list
        debugk("Unregistering parport driver.\n");

        // Loop through each lintap structure.  Release the parallel port it has claimed
        // and unregister all the pad devices it was responsible for.
        // Finally, release the kernel memory for the Lintap structure.
		while (ptr_lintap != NULL)
        {
            int pad_count = 0;
			struct lintap_device* temp_lintap = ptr_lintap->next;
			if (ptr_lintap->port_claimed) {	lintap_release_port(ptr_lintap); }
            // Unregister each pad device
   			for (pad_count = 0; pad_count < MAX_PADS; pad_count++)
            {
                struct input_dev* dev = ptr_lintap->pads[pad_count].dev;
				if (dev != NULL) { input_unregister_device(dev); }
            }
			debugk("Unregistering device: %s\n", ptr_lintap->port_dev->name);
			parport_unregister_device(ptr_lintap->port_dev);
            kfree(ptr_lintap);  //free lintap and pads memory
			ptr_lintap = temp_lintap; //On to the next pointer on the list
		}

        // unregister driver from kernel
		parport_unregister_driver(&lintap_driver);
		registered_with_parport = false;
		debugk("Destroying Lintap Timer\n");
		destroy_timer_on_stack(&timer);
	} else {
		debugk("Driver is not registered, module exiting.\n");
	}
}

module_init(lintap_module_init);  //tell kernel to use lintap_module_init routine
module_exit(lintap_module_exit);  //tell kernel to use lintap_module_exit routine
