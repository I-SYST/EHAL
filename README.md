![I-SYST inc.](https://www.i-syst.com/images/I-SYST_Logo180.png) 
  

![nRF5x product line](https://www.i-syst.com/images/nRF5_Products_1024.png) 
 
  
<p align="center"> 
 
[Nordic nRF51 & nRF52 product line](https://www.tindie.com/stores/hnhoan/)
 
</p> 
 

EHAL (Embedded Hardware Abstraction Library)
====
 
This is a multi-archtecture multi-platform Hardware Abstraction Library. The main purpose is to create a layer of generic peripherals and devices interface (drivers) that is common across MCU architectures and platforms. This will allow creation of more reusable and portable applications (firmware for most part) that can be moved form on MCU to an other with minimal changes.      
 
See https://embeddedsoftdev.blogspot.ca/p/h.html for more detail installation and setup of development environment.

Library online documentation located here https://www.i-syst.com/docs/EHAL/ 
 
Library source code on GitHub at https://github.com/I-SYST/EHAL 

* Need to port a specific driver or MCU ? Put your request in issues. It will get done eventually. If you want to have it faster, a fee based port is available.   
 
* EHAL is being outgrown. We've moved to the new [IOsonata](https://github.com/IOsonata/IOsonata) in order to support more archtecture & platforms.  We are re-organizing the new library for easier project integrations in order to response to more demands for other environnements.  Current status : progressing well. New features will be added to the IOsanata.  We will keep updating EHAL for bug fixes. If you are new user or starting a new project, it would be best to start with IOsonata.  Thank you for your supports. 
---
 
external vendors' SDK and library required :
--- 
 
[CMSIS](https://github.com/ARM-software/CMSIS_5) : ARM CMSIS SDK for all ARM platform

[nRF5_SDK](https://developer.nordicsemi.com)  : Nordic nRF5x Bluetooth Low Energy

[nrf5_SDK_Mesh](https://www.nordicsemi.com/Software-and-Tools/Software/nRF5-SDK-for-Mesh/Download#infotabs) : Nordic nRF5 SDK for Bluetoth Mesh

[Micro-ECC](https://github.com/kmackay/micro-ecc) : Encryption library

[ICM-20948 Motion_Driver](https://www.invensense.com/developers) : Create a user at https://www.invensense.com/developers. Under "Downloads" download "DK-20948 eMD-SmartMotion ...". Unzip the downloaded file and navigate to EMD-Core/sources. Copy the folder Invn to external/Invn as indected in the folder tree bellow.

KSDK      : Kinetis SDK

[BSEC](https://www.bosch-sensortec.com/bst/products/all_products/bsec) : Bosch Sensortec Environmental Cluster (BSEC) Software for #BME680 environmental sensor.  BSEC is needed for calculating Air Quality Index.  Go to https://www.bosch-sensortec.com/bst/products/all_products/bsec at the end of the page.  Select checkbox to accept license terms to download.  Unzip the the downloaded file. Rename the extracted folder BSEC and copy the whole folder to external as indicated in the folder tree bellow.  

![BLUEIO-TAG-EVIM](https://www.i-syst.com/images/BLUEIO-TAG-EVIM_page.png) 
 
<p align="center"> 
  
[BLYST Nano sensor board](https://www.crowdsupply.com/i-syst/blyst-nano)
 
</p> 
 
 
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
 |   |-- nrf5_SDK_Mesh   - Latest Nordic SDK for Mesh (https://www.nordicsemi.com/eng/nordic/Products/nRF5-SDK-for-Mesh/nRF5-SDK-for-Mesh/62377)
 |   |---nRF5_SDK_12     - Last version of Nordick SDK12 for nRF51 series
 |   |-- Micro-ECC       - Micro-ECC (download from https://github.com/kmackay/micro-ecc)
 |   |-- KSDK            - Kinetis SDK
 |   |-- BSEC            - Bosch Sensortec Environmental Cluster (BSEC) Software (https://www.bosch-sensortec.com/bst/products/all_products/bsec) for #BME680
 |   |-- MPL             - Invensense Motion Driver (download https://www.invensense.com/developers)
 |   |   |-- core        - Copy core from motion_driver_6.12/arm/STM32F4_MD6/Projects/eMD6 here
 |   |   |-- lib
 |   |   |   |-- m0      - Unzip the liblibmplmpu_m0.zip, copy liblibmplmpu.a here
 |   |   |   |-- m3      - Unzip the liblibmplmpu_m3.zip, copy liblibmplmpu.a here
 |   |   |   |-- m4hfp   - Unzip the liblibmplmpu_m4_hardfp.zip, copy liblibmplmpu.a here
 |   |   |   |-- m4nfp   - Unzip the liblibmplmpu_m4_npfp.zip, copy liblibmplmpu.a here
 |   |   |   |-- m4sfp   - Unzip the liblibmplmpu_m4_softfp.zip, copy liblibmplmpu.a here
 |   |-- Invn            - Invensense SmartMotion Driver (download https://www.invensense.com/developers) 
 |   |   |-- Devices
 |   |   |...
 |   |...
 |   |
 |-- EHAL      - Put the EHAL here
 |   |-- docs        - Contains EHAL Doxygen documentations. (https://i-syst.github.io/docs/EHAL/)
 |   |-- include     - Generic include common to all platform
 |   |   |-- bluetooth   - Generic definition for Bluetooth
 |   |   |-- converters  - Generic definition for ADV, DAC, etc...
 |   |   |-- coredev     - Generic core processor builtin devices such as i2c, uart, spi, timer, etc...
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
 |   |   |   |   |-- CMSIS    - Static library containing startup code, interrupt vector, etc...
 |   |   |   |   |-- EHAL     - Embedded Hardware Abstraction Library project for NXP
 |   |   |   |   |   |-- include
 |   |   |   |   |   |-- src
 |   |   |   |   |-- exemples - Example code
 |   |   |   |   |
 |   |   |   |-- LPC17xx      - LPC17xx processor workspace
 |   |   |   |   |-- CMSIS    - Static library containing startup code, interrupt vector, etc...
 |   |   |   |   |-- EHAL     - Embedded Hardware Abstraction Library project for NXP
 |   |   |   |   |   |-- include
 |   |   |   |   |   |-- src
 |   |   |   |   |-- exemples - Example code
 |   |   |
 |   |   |-- Nordic      - Nordic Semiconductor based  MCU
 |   |   |   |-- nRF51        - nRF51 processor workspace
 |   |   |   |   |-- CMSIS        - Static library containing startup code, interrupt vector, etc...
 |   |   |   |   |-- EHAL         - Embedded Hardware Abstraction Library project for Nordic
 |   |   |   |   |   |-- include
 |   |   |   |   |   |-- src
 |   |   |   |   |-- exemples     - exemple projects
 |   |   |   |-- nRF52        - nRF52 processor workspace
 |   |   |   |   |-- CMSIS        - Static library containing startup code, interrupt vector, etc...
 |   |   |   |   |-- EHAL         - Embedded Hardware Abstraction Library project for Nordic
 |   |   |   |   |   |-- include
 |   |   |   |   |   |-- src
 |   |   |   |   |-- exemples     - exemple projects
 |   |   |
 |   |   |-- ST          - ST based MCU
 |   |   |   |-- STM32F0xx
 |   |   |   |-- STM32F4xx
 |   |   |   |-- STM32L0xx
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
 
