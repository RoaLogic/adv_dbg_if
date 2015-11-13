# adv_dbg_if
Universal Advanced JTAG Debug Interface

This is a fork of the work the ETH-Zurich did on the Advanced Debug Interface from Opencores.

Changes compared to the Opencores version:
- Broke out the bus-interface. This allows easy adaptions to support multiple bus interfaces
- Bug fixes and coding fixes (mostly blocking vs. non-blocking)
- Migrated the code to SystemVerilog
- Removed dependency on define statements. Moved to parameters and SystemVerilog packages.
- Added support for non-32bit CPUs (e.g. 16bit, 64bit, ...).
- Added support for non-32bit bus interfaces

ToDo:
- More cleaning up to do
- More work to do on the JTAG Serial Port
	- Move to 8bit interface, instead of current 32bit interface
	- Add support for alternative bus interfaces
