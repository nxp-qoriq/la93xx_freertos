1 - Introduction:
================
LX9310 SoC supports below boot sources:

DCFG_ccsr: PORSR1 : BOOT_SRC [15:14]
	00b - Reserved
	01b - Reserved
	10b - I2C EEPROM
	11b - PCIe Host Memory


In case of I2C Boot, the target software (Bootloader/OS) is generally
kept on I2C compatible non-volatile storage like an EEPROM which is
pre-programmed with boot firmware using tools like debuggers/flash
programmers. Boot software initializes the I2C subsystem to read and
execute firmware programmed on EEPROM.


2 - Boot Overview:
=================

2.1 - BOOTROM:
--------------
When the Cortex M4 core is released from reset it attempts to
execute instructions from a Boot ROM located at address 0x0000_0000.
The Boot ROM occupies 8KB of memory space.  During execution the Boot
ROM code uses the highest 4KB region of TCMU (Tightly Coupled Data Memory)
as its stack space – 0x2000_F000-0x2000_FFFF.

2.2 - Boot Plugin Routine
-------------------------
The Boot Plugin Routine is an executable software routine that runs
on the M4 core to do pre-boot initialization of the device or system.
It is intended to do any initialization that is required before the
Reset State Machine completes the reset process.

2.3 - Boot Header
-----------------
The Boot Header consists of 8 32-bit words of information used by the
Boot ROM. The structure is 32-bit aligned and all elements are padded
to fill 32-bit total size. It includes the following

struct header { 
uint32_t preamble; 
uint32_t plugin_size; 
uint32_t plugin_offset; 
uint32_t bl_size; 
uint32_t bl_src_offset; 
uint32_t bl_dest; 
uint32_t bl_entry; 
uint32_t flags; 
};

2.4 - Boot Header Flags	
-----------------------
The bits of the flags field are used to convey information to the
BootROM regarding the processing as described below.

31                 16             0
|-----------------|-|------------|-|
|-----------------|-|------------|-|

BIT[16] : Early_Exit
          Earty Exit to RTOS.
          When set, this flag causes the BootROM to exit and begin
          executing the next level of Boot Loader or RTOS immediately
          after downloading it. It becomes the responsibility of the
          downloaded code to complete the reset sequence as the
          BootROM would have done. The expectation is that the “boot loader”
          downloaded is actually the full RTOS.

BIT[0] : Memcopy
         Memcopy based code download
         When set, this flag causes BootROM to use a memcopy routine run on
         the M4 core to copy the plugin routine and/or the bootloader from host
         memory When left clear, the BootROM uses the eDMA engine to copy code
         from the host memory



3 - I2C Boot on LA9310RDB
=========================

3.1 - Configure I2C Boot Source
-------------------------------

By Default, the LA9310RDB board is enabled with PCIe as the boot source.
To change the Boot Source to I2C, the board need to be reworked to connect
CFG_BOOT_SRC0 to ground.

3.2 - Enable write operation on EEPROM
--------------------------------------
To Enable write operation to EEPROM CAT24C512 connected to LA9310RDB,
Jumper (J6) needs to be installed. By default jumper (J6) is installed
on LA9310RDB board.
If a different EEPROM is used, make sure to enable write operation
to the EEPROM.

3.3 - EEPROM Size and Layout
----------------------------
A minimum of a 512KB EEPROM is recommended to hold the Boot header, Boot
firmware and VSPA firmware. VSPA firmware should be copied from the EEPROM
to the VSPA IP block.

3.4 - I2C boot sequence
-----------------------
The target containing the Boot Plugin and Boot Loader is indicated by the
Power-On Reset Configuration input field CFG_BOOT_SRC, which can be read
from the PORSR register in the Device Configuration block (DCFG).

If the Boot Source is I2C the offset is relative to address 0x0 of the
I2C target. On the LA9310RDB board, the I2C target (EEPROM of size 64KB)
is connected at address 0x50.

- M4 begins execution in ROM
- M4 Reads CFG_BOOT_SRC from POR Configuration settings in the Device
  Configuration block (DCFG) block
- For CFG_BOOT_SRC = 2’b10 (I2C EEPROM - slave address 7’h50) ROM initializes
  both TCM memory arrays with writes of 0x0000 to initialize ECC memory to
  valid values.
- ROM reads I2C offset 0x0 for a valid Boot Preamble
  Note: supported devices are extended address EEPROM with calling address
        7’b101_0aaa. BootROM only supports ‘aaa’=3’b000.
- ROM reads Boot Header which should be preprogrammed at offset 0x0 on EEPROM
  using debugger/flash programmer
- If the Plugin Routine size is nonzero:
    - ROM code downloads the Plugin Routine from the I2C EEPROM and writes
      it to the base of TCM Code memory at local address 0x1F80_0000.
    - When the copy is complete, ROM code makes a subroutine call to TCM Code
      memory at 0x1F80_0000
    - The Plugin routine executes (e.g. programming the RFIC and configuring a
      valid clock to support completion of the reset sequence) then returns to
      the ROM code
- If header.flags[16]=0 (no “Early_Exit”):
    - ROM code handshakes with the Reset FSM allowing hardware to complete the
      reset sequence all the way to the SYSREADY state
    - ROM code waits for reset sequence to complete (either through polling the
      Reset FSM state or by configuring EPU to generate RXEV event upon Reset
      FSM reaching SYSREADY then entering a WFE loop - based on CFG_SYSRDY_EVT
      input).
- ROM code then downloads the Boot Loader from the I2C EEPROM according to the
  size and offset previously found in the Boot Header and writes it to the TCM
  Code or Data memory as specified by the BL_DEST value.
- When the copy is complete, code jumps from ROM to the Boot Loader at the
  BL_ENTRY location.

3.5 - I2C Boot PoC on LA9310RDB
-------------------------------
Existing LA9310RDB boards are equipped with 64KB EEPROM CAT24C512.
To test I2C boot, we need to reduce size of FreeRTOS image to fit into
64KB EEPROM space.
We have disabled RFIC with below changes to reduce image size.

--- a/Demo/CORTEX_M4_NXP_LA9310_GCC/CMakeLists.txt
+++ b/Demo/CORTEX_M4_NXP_LA9310_GCC/CMakeLists.txt
@@ -41,7 +41,7 @@ SET(CMAKE_ASM_FLAGS_DEBUG "${CMAKE_ASM_FLAGS_DEBUG}  -D__DEBUG")
 SET(LA9310_TURN_ON_COMMAND_LINE ON)

 # Enable/Disable RFIC driver
-SET(RFIC ON)
+SET(RFIC OFF)

--- a/Demo/CORTEX_M4_NXP_LA9310_GCC/platform/ARM_CM4/la9310.ld
+++ b/Demo/CORTEX_M4_NXP_LA9310_GCC/platform/ARM_CM4/la9310.ld
@@ -18,8 +18,8 @@ MEMORY
 {
   m_startup            (RX)  : ORIGIN = 0x1F800000, LENGTH = 0x00000100
   m_interrupts         (RX)  : ORIGIN = 0x1F800100, LENGTH = 0x00000240
-  m_text               (RX)  : ORIGIN = 0x1F800340, LENGTH = 0x00010000
-  m_data               (RW)  : ORIGIN = 0x1F810340, LENGTH = 0x00008000
+  m_text               (RX)  : ORIGIN = 0x1F800340, LENGTH = 0x0000BF00
+  m_data               (RW)  : ORIGIN = 0x1F80C240, LENGTH = 0x00008000
   m_dtcm               (RW)  : ORIGIN = 0x20000000, LENGTH = 0x0000F000
   br_data              (R)   : ORIGIN = 0x2000F000, LENGTH = 0x00001000
 }
