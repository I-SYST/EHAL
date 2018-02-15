EHAL (Embedded Hardware Abstraction Library)
====
 
This is a multi-archtecture multi-platform Hardware Abstraction Library. The main purpose is to create a layer of generic peripherals and devices interface (drivers) that is common across MCU architectures and platforms. This will allow creation of more reusable and portable applications (firmware for most part) that can be moved form on MCU to an other with minimal changes.      
 
See https://embeddedsoftdev.blogspot.ca/p/h.html for more detail installation and setup of development environment.

Library online documentation located here https://i-syst.github.io/docs/EHAL/ 
 
Library source code on GitHub at https://github.com/I-SYST/EHAL 

external vendors' SDK and library needed :
--- 
 
CMSIS     : [ARM CMSIS SDK for all ARM platform](https://github.com/ARM-software/CMSIS_5)

nRF5_SDK  : [Latest Nordic SDK](https://developer.nordicsemi.com)

Micro-ECC : [Micro-ECC](download from https://github.com/kmackay/micro-ecc)

KSDK      : Kinetis SDK

BSEC      : [Bosch Sensortec Environmental Cluster (BSEC) Software](https://www.bosch-sensortec.com/bst/products/all_products/bsec) for #BME680 environmental sensor  

![IBK-BME680](docs/IBK-BME680_PinOut.png) 
[Available @Tindie](https://www.tindie.com/products/hnhoan/bme680-temperature-pressure-humidity-gas-sensor/) 

EHAL folder structure
---
 
The way the EHAL folder is structure is simple.  The deeper you go inside the more it is specific the the architecture or platform.  The parent folder contains all that is commonly available to the child folder.  Which means, source file from child folder can access any source in the upper parent folder but not the other way around.  This is the way to keep the abstraction separated from implementation and easier to keep track of things.

There are 2 main libraries in the EHAL for each ARM based MCU.  
- CMSIS : This contains the startup code, main interrupt vector and RTOS if available.  This lib is required for all firmware
- EHAL : This is the main hardware abstraction library. It contains all peripheral drivers, SPI, I2C, UART, Timer, BLE,...

```
/your_root     - Development root directory
 |-- external        - Contains downloaded SDKs from silicon vendors
 |   |-- CMSIS           - ARM CMSIS SDK for all ARM platform (https://github.com/ARM-software/CMSIS_5)
 |   |-- nRF5_SDK        - Latest Nordic SDK (https://developer.nordicsemi.com)
 |   |-- Micro-ECC       - Micro-ECC (download from https://github.com/kmackay/micro-ecc)
 |   |-- KSDK            - Kinetis SDK
 |   |-- BSEC            - Bosch Sensortec Environmental Cluster (BSEC) Software (https://www.bosch-sensortec.com/bst/products/all_products/bsec) for #BME680		
 |   |...
 |   |
 |-- EHAL      - Put the EHAL here
 |   |-- docs        - Contains EHAL Doxygen documentations. (https://i-syst.github.io/docs/EHAL/)
 |   |-- include     - Generic include common to all platform
 |   |   |-- bluetooth   - Generic definition for Bluetooth
 |   |   |-- converters  - Generic definition for ADV, DAC, etc...
 |   |   |-- miscdev     - Generic definition for other non categorized devices
 |   |   |-- sensors     - Generic definition for al sort of sensors (environmental, motion, etc...)
 |   |   |-- usb         - Generic definition for USB
 |   |-- src         - Generic implementation source common to all platform
 |   |
 |   |-- ARM         - Cortex-M series based MCU
 |   |   |-- include     - Common include for all ARM platform
 |   |   |-- src         - Common source for all ARM platform
 |   |   |
 |   |   |-- NXP         - NXP based MCU
 |   |   |   |-- LPC11xx      - LPC11xx processor workspace
 |   |   |   |   |-- CMSIS
 |   |   |   |   |-- EHAL     - Embedded Hardware Abstraction Library project for NXP
 |   |   |   |   |   |-- include
 |   |   |   |   |   |-- src
 |   |   |   |   |-- exemples - Example code
 |   |   |   |   |
 |   |   |   |-- LPC17xx      - LPC17xx processor workspace
 |   |   |   |   |-- CMSIS
 |   |   |   |   |-- EHAL     - Embedded Hardware Abstraction Library project for NXP
 |   |   |   |   |   |-- include
 |   |   |   |   |   |-- src
 |   |   |   |   |-- exemples - Example code
 |   |   |
 |   |   |-- Nordic      - Nordic Semiconductor based  MCU
 |   |   |   |-- nRF51        - nRF51 processor workspace
 |   |   |   |   |-- CMSIS        - static library of CMSIS system functions for nRF51
 |   |   |   |   |-- EHAL         - Embedded Hardware Abstraction Library project for Nordic
 |   |   |   |   |   |-- include
 |   |   |   |   |   |-- src
 |   |   |   |   |-- exemples     - exemple projects
 |   |   |   |-- nRF52        - nRF52 processor workspace
 |   |   |   |   |-- CMSIS        - static library of CMSIS system functions for nRF52
 |   |   |   |   |-- EHAL         - Embedded Hardware Abstraction Library project for Nordic
 |   |   |   |   |   |-- include
 |   |   |   |   |   |-- src
 |   |   |   |   |-- exemples     - exemple projects
 |   |   |
 |   |   |-- ST          - ST based MCU
 |   |   |
 |   |   |-- TI          - Texas Instruments based MCU
 |   |   |   |-- CC3200
 |   |   |   |   |-- CMSIS
 |   |   |   |   |-- EHAL
 |   |   |   |   |   |-- include
 |   |   |   |   |   |-- src
 |   |   |   |   |-- exemples - Example code
 |   |   |
 |   |   |-- Freescale   - Freescale based MCU
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
```
 
All EHAL projects are Eclipse native project with GCC compiler.  For Eclipse & Gcc installations, follow this blog page http://embeddedsoftdev.blogspot.ca/p/eclipse.html. 
 
