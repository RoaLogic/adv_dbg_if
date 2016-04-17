# adv_dbg_if
Universal Advanced JTAG Debug Interface

This is a fork of the work the ETH-Zurich did on the Advanced Debug Interface from Opencores.

Changes compared to the Opencores version:
- Broke out the bus-interface. This allows easy adaptions to support multiple bus interfaces
- Supported Bus interfaces:
-- Wishbone
-- AHB3
- Bug fixes and coding fixes (mostly blocking vs. non-blocking)
- Migrated the code to SystemVerilog
- Removed dependency on define statements. Moved to parameters and SystemVerilog packages.
- Added support for non-32bit CPUs (e.g. 16bit, 64bit, ...).
- Added support for non-32bit bus interfaces
- Moved JSP to 8bit data interface

Current status:
- Core is up and running, happily debugging RISC-V. Tested in HW on an Altera development board (using the Altera virtual JTAG interface), and in simulations using VPI and FileIO JTAG
