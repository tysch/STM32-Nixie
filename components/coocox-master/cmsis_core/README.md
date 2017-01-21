
@if (DOXYGEN_PROJECT)
@defgroup cmsis_core
@endif

# cmsis_core 

[TOC] 



## CMSIS DSP Software Library 
 


### Introduction

This user manual describes the CMSIS DSP software library, a suite of common signal processing functions for use on Cortex-M processor based devices.

The library is divided into a number of functions each covering a specific category:
- Basic math functions
- Fast math functions
- Complex math functions
- Filters
- Matrix functions
- Transforms
- Motor control functions
- Statistical functions
- Support functions
- Interpolation functions

The library has separate functions for operating on 8-bit integers, 16-bit integers, 32-bit integer and 32-bit floating-point values.

### Using the Library

The library installer contains prebuilt versions of the libraries in the Lib folder.
- arm_cortexM4lf_math.lib (Little endian and Floating Point Unit on Cortex-M4)
- arm_cortexM4bf_math.lib (Big endian and Floating Point Unit on Cortex-M4)
- arm_cortexM4l_math.lib (Little endian on Cortex-M4)
- arm_cortexM4b_math.lib (Big endian on Cortex-M4)
- arm_cortexM3l_math.lib (Little endian on Cortex-M3)
- arm_cortexM3b_math.lib (Big endian on Cortex-M3)
- arm_cortexM0l_math.lib (Little endian on Cortex-M0)
- arm_cortexM0b_math.lib (Big endian on Cortex-M3)

The library functions are declared in the public file arm_math.h which is placed in the Include folder. Simply include this file and link the appropriate library in the application and begin calling the library functions. The Library supports single public header file arm_math.h for Cortex-M4/M3/M0 with little endian and big endian. Same header file will be used for floating point unit(FPU) variants. Define the appropriate pre processor MACRO ARM_MATH_CM4 or ARM_MATH_CM3 or ARM_MATH_CM0 or ARM_MATH_CM0PLUS depending on the target processor in the application.

### Examples

The library ships with a number of examples which demonstrate how to use the library functions.

### Toolchain Support

The library has been developed and tested with MDK-ARM version 4.60. The library is being tested in GCC and IAR toolchains and updates on this activity will be made available shortly.

### Building the Library

The library installer contains project files to re build libraries on MDK Tool chain in the CMSIS\DSP_Lib\Source\ARM folder.
- arm_cortexM0b_math.uvproj
- arm_cortexM0l_math.uvproj
- arm_cortexM3b_math.uvproj
- arm_cortexM3l_math.uvproj
- arm_cortexM4b_math.uvproj
- arm_cortexM4l_math.uvproj
- arm_cortexM4bf_math.uvproj
- arm_cortexM4lf_math.uvproj

The project can be built by opening the appropriate project in MDK-ARM 4.60 chain and defining the optional pre processor MACROs detailed above.

### Pre-processor Macros

Each library project have differant pre-processor macros.
- UNALIGNED_SUPPORT_DISABLE:

Define macro UNALIGNED_SUPPORT_DISABLE, If the silicon does not support unaligned memory access
- ARM_MATH_BIG_ENDIAN:

Define macro ARM_MATH_BIG_ENDIAN to build the library for big endian targets. By default library builds for little endian targets.
- ARM_MATH_MATRIX_CHECK:

Define macro ARM_MATH_MATRIX_CHECK for checking on the input and output sizes of matrices
- ARM_MATH_ROUNDING:

Define macro ARM_MATH_ROUNDING for rounding on support functions
- ARM_MATH_CMx:

Define macro ARM_MATH_CM4 for building the library on Cortex-M4 target, ARM_MATH_CM3 for building library on Cortex-M3 target and ARM_MATH_CM0 for building library on cortex-M0 target, ARM_MATH_CM0PLUS for building library on cortex-M0+ target.
- __FPU_PRESENT:

Initialize macro __FPU_PRESENT = 1 when building on FPU supported Targets. Enable this macro for M4bf and M4lf libraries

### Copyright Notice

Copyright (C) 2010-2013 ARM Limited. All rights reserved. 

