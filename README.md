# Lintap
This is a Linux kernel module to control a reengineered version of the 'Megatap' DIY adapter that allows up to 4 Playstation game controllers to be connected to a PC via a parallel port.  The original Megatap circuit supported force feedback, but could only poll each controller one at a time.  When used with 4 controllers at once, this resulted in significant slow down on a 1Ghz CPU PC, due to the slow nature of Parallel Port I/O.

This kernel module has been written to work with an alternative version of the Megatap adapter.  By rewiring the Megatap to match an alternative circuit, it is possible to poll all 4 controllers connected to the PC at once.  This reduces the processing penalty to a quarter of what it would be, in exchange for losing force feedback.

Some parallel ports are faster than others, so there are two module parameters that adjust the delay between reading bits from the port and the delay after sending commands to the PSX pads before attempting to read from them.
