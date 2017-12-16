EHAL (Embedded Hardware Abstraction Library)
====
 
This is a multi-archtecture multi-platform Hardware Abstraction Library. The main purpose is to create a layer of generic peripherals and devices interface (drivers) that is common across MCU architectures and platforms. This will allow creation of more reusable and portable applications (firmware for most part) that can be moved form on MCU to an other with minimal changes.      
 
See http://embeddedsoftdev.blogspot.ca/p/h.html for more detail installation and setup of development environment.


The way the EHAL folder is structure is simple.  The deeper you go inside the more it is specific the the architecture or platform.  The parent folder contains all that is commonly available to the child fonder.  Which means, source file from child folder can access any source in the upper parent folder but not the other way around.  This is the way to keep the abstraction separated from implementation and easier to keep track of things.


/your_root     - Development root directory
 |-- external    - Contains downloaded SDKs from silicon vendors
 |   |-- CMSIS            - ARM CMSIS SDK for all ARM platform (download from ARM)
 |   |-- nRF5_SDK     - Latest Nordic SDK (download from nordicsemi.com)
 |   |-- KSDK             - Kinetis SDK
 |   |......
 |-- EHAL      - Put the EHAL here
 |   |-- include     - Generic include common to all platform
 |   |   |-- bluetooth   - Generic definition for Bluetooth
 |   |   |-- converters  - Generic definition for ADV, DAC, etc...
 |   |   |-- miscdev     - Generic definition for other non categorized devices
 |   |   |-- sensors      - Generic definition for al sort of sensors (environmental, motion, etc...)
 |   |-- src            - Generic implementation source common to all platform
 |   |
 |   |-- ARM - Cortex-M series based MCU
 |   |   |-- include    - Common include for all ARM platform
 |   |   |-- src           - Common source for all ARM platform
 |   |   |
 |   |   |-- NXP        - NXP ARM platform
 |   |   |   |-- LPC11xx - LPC11xx processor workspace
 |   |   |   |   |-- CMSIS
 |   |   |   |   |-- EHAL     - Embedded Hardware Abstraction Library project for NXP
 |   |   |   |   |   |-- include
 |   |   |   |   |   |-- src
 |   |   |   |   |-- exemples - Example code
 |   |   |   |-- LPC17xx - LPC17xx processor workspace
 |   |   |   |   |-- CMSIS
 |   |   |   |   |-- EHAL     - Embedded Hardware Abstraction Library project for NXP
 |   |   |   |   |   |-- include
 |   |   |   |   |   |-- src
 |   |   |   |   |-- exemples - Example code
 |   |   |
 |   |   |-- Nordic
 |   |   |   |-- nRF51 - nRF51 processor workspace
 |   |   |   |   |-- CMSIS   - static library of CMSIS system functions for nRF51
 |   |   |   |   |-- EHAL    - Embedded Hardware Abstraction Library project for Nordic
 |   |   |   |   |   |-- include
 |   |   |   |   |   |-- src
 |   |   |   |   |-- exemples - exemple projects
 |   |   |   |-- nRF52 - nRF52 processor workspace
 |   |   |   |   |-- CMSIS   - static library of CMSIS system functions for nRF52
 |   |   |   |   |-- EHAL    - Embedded Hardware Abstraction Library project for Nordic
 |   |   |   |   |   |-- include
 |   |   |   |   |   |-- src
 |   |   |   |   |-- exemples - exemple projects
 |   |   |
 |   |   |-- ST
 |   |   |
 |   |   |-- TI
 |   |   |   |-- CC3200
 |   |   |   |   |-- CMSIS
 |   |   |   |   |-- EHAL
 |   |   |   |   |   |-- include
 |   |   |   |   |   |-- src
 |   |   |   |   |-- exemples - Example code
 |   |   |
 |   |   |-- Freescale
 |   |   |   |-- MKL
 |   |   |   |   |-- CMSIS
 |   |   |   |   |-- EHAL
 |   |   |   |   |   |-- include
 |   |   |   |   |   |-- src
 |   |   |   |   |-- exemples - Example code
 |   |...
 |   |-- Linux
 |   |   |...
 |   |-- OSX
 |   |   |...
 |   |-- Win
 |   |   |...
 | ...
