/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief This file is not a part of the header files referenced by the source code.
 *		  However it defines the only a part of the doxygen documentation.
 *
 * \par Application note:
 *      AVR2001: AT86RF230 Software Programmer's Guide
 *
 * \par Documentation
 *      For comprehensive code documentation, supported compilers, compiler 
 *      settings and supported devices see readme.html
 *
 * \author
 *      Atmel Corporation: http://www.atmel.com \n
 *      Support email: avr@atmel.com
 * 
 * $Name$
 * $Revision: 613 $
 * $RCSfile$
 * $Date: 2006-04-07 14:40:07 +0200 (fr, 07 apr 2006) $  \n
 *
 * Copyright (c) 2006, Atmel Corporation All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. The name of ATMEL may not be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY AND
 * SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

/*=== Introduction ===================================================*/
/**
@mainpage AVR2001: AT86RF230 Software Programmer's Guide - Examples

@section secCont Contents
 - @ref secIntro
 - @ref avrSetup
 - @ref pgCompiler
 - @ref runApplication

@section secIntro Introduction
Two examples are currently supplied with the AVR2001 application note. Both these 
examples focus on how to use the TAT. The examples themselves are two different
implementations of a wireless communication link. This link can convey data between
two nodes. The idea is that these nodes are connected to a PC, either through USB 
or RS232, and data is sent wirelessly between them. Find more about the two examples
by clicking on the liks below:

 - @ref wireless_uart
 - @ref wireless_uart_extended

<h1>The focus for this documentation is the code written in the C programming language for the AVR.</h1>
*/

/*=== Functions ======================================================*/

/*=== Setup ==========================================================*/
/**
@page avrSetup Requirements to run the Example Applications


Steps:
 -# @ref hwRZ502
 -# @ref hwSTK541
 -# @ref secProgramming


@section hwRZ502 Setup of the ATAVRRZ502 Accessory Kit
The following steps walk the reader through how to assemble and prepare the 
STK500 board and the STK501 top module to be used with the RZ502.
Then finally how to mount the RZ502 board and configure it to run the evaluation application.

@note The following steps must be completed successfully to ensure correct operation of the hardware.

@subsection subsecA Step A. Assemble and configure the STK500 board

The jumpers and operating voltage of the STK500 board must be set up 
correct in advance of mounting the STK501 and radio board.
 -# Carefully remove any AVRs from the target sockets on the STK500.
 -# Place jumpers on the following headers:
  - VTARGET
  - AREF
  - RESET
  - BSEL2
 -# Connect a serial cable to the connector marked RS232CTRL on the STK500 to a COM port on the PC.
 -# Apply power to the STK500 by moving the power switch toward the edge of the board.
 -# Start AVR Studio and press "Cancel" in the Welcome dialog window.
 -# From the Tools menu, select "Program AVR" and "Connect...".
 -# Select "STK500 or AVRISP" as platform and then press "Connect". Verify that a window named "STK500" appears.
 -# Open the Board tab. Adjust the VTarget voltage and ARef voltage to 3 Volts. Press the Write Voltages button. 
 Close the STK500 window.


@subsection subsecB Step B. Assemble and configure the STK500 board
The jumpers are now set correctly and the target voltage is set to 3 volts. This ensures that 
the AT86RF230 radio transceiver is not damaged and the ATmega1281 will run at 8MHz. The next 
steps describes how to mount the STK501 top module.
 -# Insert an ATmega1281 into the Zero Insertion Force (ZIF) socket on the STK501. Press the 
 spring-loaded top frame of the ZIF down and gently aligning the device underneath the 
 fine-pitched wires. Verify that pin 1 on the TQFP package (dot in one of its corners) points 
 towards the notched corner of the ZIF.
 -# Ensure that power is turned off on the STK500 board.
 -# Connect the STK501 to the STK500 by using expansion header 0 and 1. Ensure that the 
 EXPAND0 written on the STK501 top module aligns with the EXPAND0 written beside the expansion 
 header on the STK500 board.
 -# Connect PINE1 and PINE2 to the RXD and TXD pins respectively. This can be done using 
 one of the 2-wire cables included in the STK500 starter kit. The RXD and TXD pins are 
 found close to the 9-pins RS-232 connector on the STK501.
 -# To enable the In-System Programmer (ISP) interface, connect a 6-wire cable between the 
 ISP6PIN connector on the STK500 board and the SPROG connector on the STK501 top module. 

@subsection subsecC Step C. Mount the Radio Board
The RZ502 radio board is added to the STK501 by using its EXPAND1 header.
 -# Find pin 1 on the radio board (Only square pad and with the digit 1 printed to its right). 
 This is pin must align with pin 1 on STK501's EXPAND1 header.
 -# Insert the radio board.

@section hwSTK541 Setup of the STK541 Kit
The STK541 needs no special guidelines for hardware assembley. Just mount a RCB without 
batteries to the STK541 board.

@section secProgramming Programming the ATmega1281
The precompiled firmware used in the following subsection can be found in the "examples\bin" folder of the .zip.

\note This particular example assumes that the RZ502 HW is used and that the firmware is the simple Wireless UART.

 -# Connect a serial cable to the connector marked RS232CTRL on the STK500 to a COM port on the PC.
 -# Apply power to the STK500 by moving the power switch toward the edge of the board.
 -# Start AVR Studio and press the "Cancel" button in the Welcome dialog window.
 -# From the Tools menu, select "Program AVR" and "Connect...".
 -# Select STK500 or AVRISP as platform and then press "Connect". Verify that a dialog named "STK500 with 
 STK501 top module" appears.
 -# Open the "Program" tab. Select ATmega1281 from the Device pull down menu.
 -# Select the "Fuses" tab and ensure that the following Fuses are checked/not checked:
  - <b>Checked:</b> Brown-out detection disabled
  - <b>Checked:</b> Boot Flash section size=4096 words
  - <b>Checked:</b> Int. RC Osc.;Start-up time: 6CK + 65 ms
  - <b>Not checked:</b> Divide clock by 8 internally
 -# Go back to the "Program" tab. Program the flash of the ATmega1281 with the 
 "WirelessUart_stk541/rz502.hex" or "WirelessUartExtended_stk541/rz502_node1/node2.hex" file found in the "examples\bin" folder.
 -# Close the STK500 window.

*/

/*=== Compilers ======================================================*/
/**
@page pgCompiler Compiler information.

@section secCOverview Overview

 - @ref secIAR
 - @ref secGCC
 - @ref secCRef

The first step to compile the TAT into your application with either the IAR or GCC compiler is to make
a new project folder and copy the lib and inc folders here. Pre configured project files can
be found in the examples folder for reference.

@note In the reminder of this chapter it is assumed that you have AVR Studio and a valid copy
of one of the two compilers installed.

@section secIAR Setup and Compile with IAR
 - Open IAR Embedded Workbench. Select New Workspace.
 - Then select the Project menu and Create New Project...
   - The Create New Project will be visible. Select AVR as Tool chain.
   - Choose the Empty project template and press OK.
   - Save the Workspace and Project file to your project directory (Same level as inc and lib).
 - Select the Project menu again and Options. The Options window will be visible:
   - General Options:
     - Target tab: Processor configurations: Choose Processor configuration -cpu=m1281.
     - Library Configuration tab: Library to Normal DLIB.
     - System tab: Check the Enable bit definition... option.
   - C/C++ Compiler:
     - Preprocessor tab:
       - Additional include directories: write "$PROJ_DIR$\inc".
       - Defined symbols: Add "AVR" and "F_CPU=8000000UL".
   - Linker:
     - Output tab:
	   - Format: select other and Output format to "ubrof 8 (forced)". Do not alter anything else.
     - Extra Options tab: 
	   - Check Use command line options and add "-C $PROJ_DIR$\lib\avr\iar\tat_lib.r90".
 - Add the files in the src/avr to the project.
 - Press F7 and verify that the code compiles without errors.

@section secGCC Setup and Compile with GCC
 - Open AVRStudio, the Welcome to AVR Studio 4 screen will appear. Select New Project.
   - In the Create new Project screen:
     - Select AVR GCC as project type and type in a project name.
	 - Browse into your project folder where you already have two folders (inc and lib).
	 - Press next.
   - In the Select debug platform and device screen:
	 - Select your Debug platform and ATmega1281 from the Device list.
	 - Press Finish.
 - In AVR Studio:
   - Select Project and Project Options from the drop down list.
   - General Tab:
	 - Verify that Device is ATmega1281.
	 - Set Frequency to 8000000 Hz.
	 - Set Optimization to -Os.
	 - Check Unsigned Chars, Unsigned Bitfields, PackStructure Members and Short Enums.
   - Include Directories Tab:
	 - Add "inc" to the Include Search Path.
   - Libraries Tab:
	 - Add "lib\avr\gcc" to the File Search Path.
	 - Verify that the Available Link Objects list have a member called "lib_tat.a".
	 - Select "libc.a" and "tat_lib.a" using the Add Library button.
	 - Press OK.
 - Select Project and Project Options again (Must be done to update the Custom Options List).	
   - Custom Options Tab:
     - Add the following three options to your Custom Compilation Options window:
       - "-DAVR" press add.
   - Press OK.
 - Add the files in the src to the project.
 - Press F7 and verify that the code compiles.

@section secCRef Compiler References
   - http://www.iar.com
   - http://winavr.sourceforge.net
*/

/*=== RUNNING THE APPLICATION ========================================*/
/**
@page runApplication Quick Start Guide.

@section secCOverview Overview

 - @ref secAssembly
 - @ref secProgram
 - @ref secPCSetup

The reminder of this page is a quick start guide to get two PCs sending
messages to eachother in the shortest time possible.

@section secAssembly Hardware Assembly
 Assemble the hardware (RZ502 or STK541 boards) as described in the @ref hwRZ502 or @ref hwSTK541.

@section secConfig Example Configuration
Both examples have a "config_uart.h" and "config_uart_extended.h" file respectively 
associated with them. These files should be altered so that the different options
are to the user's sattisfaction. It is especially important that the extended example
is compiled with different options for node1 and node2, otherwise the address filter
inside the radio transceiver will not work.

@section secProgram Firmware Programming
Program the device firmware as described in the @ref secProgramming section. Remember to program your nodes with
the correct hardware options (RZ502 or STK541).

@section secPCSetup PC Set Up
With the steps described above successfully completed, it is time to connect the nodes to two PCs, and send
data between them.
A convenient way to do this is through Microsoft (r) HyperTerminal. The STK541 will be 
connected as a virtual com port. You can find the com port number from "start->controlpanel->
Administartive Tools->Computer Management->Device Manager->Ports". The RZ502 kit will connect as a traditional
COM port.

Start a new Terminal session, in ex. open HyperTerminal. Choose the correct COM port and set the 
communication parameters (Default firmware use: 38400 8N1).
When the STK541 or RZ502 node is powered up it will send a welcome message indicating that the serial
interface is working. 
Repeat the steps above on both PCs. 
Now with both nodes up and running, type any message (Ensure that "\r\n") is 
appended and verify that the message you just sent was received by the other terminal program.
*/


