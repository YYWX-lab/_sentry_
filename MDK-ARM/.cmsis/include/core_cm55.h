/**************************************************************************//**
 * @file     core_cm55.h
 * @brief    CMSIS Cortex-M55 Core Peripheral Access Layer Header File
 * @version  V1.2.4
 * @date     21. April 2022
 ******************************************************************************/
/*
 * Copyright (c) 2018-2022 Arm Limited. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#if   defined ( __ICCARM__ )
  #pragma system_include                        /* treat file as system include file for MISRA check */
#elif defined (__clang__)
  #pragma clang system_header                   /* treat file as system include file */
#elif defined ( __GNUC__ )
  #pragma GCC diagnostic ignored "-Wpedantic"   /* disable pedantic warning due to unnamed structs/unions */
#endif

#ifndef __CORE_CM55_H_GENERIC
#define __CORE_CM55_H_GENERIC

#include <stdint.h>

#ifdef __cplusplus
 extern "C" {
#endif

/**
  \page CMSIS_MISRA_Exceptions  MISRA-C:2004 Compliance Exceptions
  CMSIS violates the following MISRA-C:2004 rules:

   \li Required Rule 8.5, object/function definition in header file.<br>
     Function definitions in header files are used to allow 'inlining'.

   \li Required Rule 18.4, declaration of union type or object of union type: '{...}'.<br>
     Unions are used for effective representation of core registers.

   \li Advisory Rule 19.7, Function-like macro defined.<br>
     Function-like macros are used to allow more efficient code.
 */


/*******************************************************************************
 *                 CMSIS definitions
 ******************************************************************************/
/**
  \ingroup Cortex_M55
  @{
 */

#include "cmsis_version.h"

/*  CMSIS CM55 definitions */
#define __CM55_CMSIS_VERSION_MAIN  (__CM_CMSIS_VERSION_MAIN)                  /*!< \deprecated [31:16] CMSIS HAL main version */
#define __CM55_CMSIS_VERSION_SUB   (__CM_CMSIS_VERSION_SUB)                   /*!< \deprecated [15:0]  CMSIS HAL sub version */
#define __CM55_CMSIS_VERSION       ((__CM55_CMSIS_VERSION_MAIN << 16U) | \
                                     __CM55_CMSIS_VERSION_SUB           )     /*!< \deprecated CMSIS HAL version number */

#define __CORTEX_M                      (55U)                                 /*!< Cortex-M Core */

#if defined ( __CC_ARM )
  #error Legacy Arm Compiler does not support Armv8.1-M target architecture.
#elif defined (__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
  #if defined __ARM_FP
    #if defined (__FPU_PRESENT) && (__FPU_PRESENT == 1U)
      #define __FPU_USED       1U
    #else
      #error "Compiler generates FPU instructions for a device without an FPU (check __FPU_PRESENT)"
      #define __FPU_USED       0U
    #endif
  #else
    #define __FPU_USED         0U
  #endif

  #if defined(__ARM_FEATURE_DSP)
    #if defined(__DSP_PRESENT) && (__DSP_PRESENT == 1U)
      #define __DSP_USED       1U
    #else
      #error "Compiler generates DSP (SIMD) instructions for a devices without DSP extensions (check __DSP_PRESENT)"
      #define __DSP_USED       0U
    #endif
  #else
    #define __DSP_USED         0U
  #endif

#elif defined ( __GNUC__ )
  #if defined (__VFP_FP__) && !defined(__SOFTFP__)
    #if defined (__FPU_PRESENT) && (__FPU_PRESENT == 1U)
      #define __FPU_USED       1U
    #else
      #error "Compiler generates FPU instructions for a device without an FPU (check __FPU_PRESENT)"
      #define __FPU_USED       0U
    #endif
  #else
    #define __FPU_USED         0U
  #endif

  #if defined(__ARM_FEATURE_DSP)
    #if defined(__DSP_PRESENT) && (__DSP_PRESENT == 1U)
      #define __DSP_USED       1U
    #else
      #error "Compiler generates DSP (SIMD) instructions for a devices without DSP extensions (check __DSP_PRESENT)"
      #define __DSP_USED         0U
    #endif
  #else
    #define __DSP_USED         0U
  #endif

#elif defined ( __ICCARM__ )
  #if defined __ARMVFP__
    #if defined (__FPU_PRESENT) && (__FPU_PRESENT == 1U)
      #define __FPU_USED       1U
    #else
      #error "Compiler generates FPU instructions for a device without an FPU (check __FPU_PRESENT)"
      #define __FPU_USED       0U
    #endif
  #else
    #define __FPU_USED         0U
  #endif

  #if defined(__ARM_FEATURE_DSP)
    #if defined(__DSP_PRESENT) && (__DSP_PRESENT == 1U)
      #define __DSP_USED       1U
    #else
      #error "Compiler generates DSP (SIMD) instructions for a devices without DSP extensions (check __DSP_PRESENT)"
      #define __DSP_USED         0U
    #endif
  #else
    #define __DSP_USED         0U
  #endif

#elif defined ( __TI_ARM__ )
  #if defined __TI_VFP_SUPPORT__
    #if defined (__FPU_PRESENT) && (__FPU_PRESENT == 1U)
      #define __FPU_USED       1U
    #else
      #error "Compiler generates FPU instructions for a device without an FPU (check __FPU_PRESENT)"
      #define __FPU_USED       0U
    #endif
  #else
    #define __FPU_USED         0U
  #endif

#elif defined ( __TASKING__ )
  #if defined __FPU_VFP__
    #if defined (__FPU_PRESENT) && (__FPU_PRESENT == 1U)
      #define __FPU_USED       1U
    #else
      #error "Compiler generates FPU instructions for a device without an FPU (check __FPU_PRESENT)"
      #define __FPU_USED       0U
    #endif
  #else
    #define __FPU_USED         0U
  #endif

#elif defined ( __CSMC__ )
  #if ( __CSMC__ & 0x400U)
    #if defined (__FPU_PRESENT) && (__FPU_PRESENT == 1U)
      #define __FPU_USED       1U
    #else
      #error "Compiler generates FPU instructions for a device without an FPU (check __FPU_PRESENT)"
      #define __FPU_USED       0U
    #endif
  #else
    #define __FPU_USED         0U
  #endif

#endif

#include "cmsis_compiler.h"               /* CMSIS compiler specific defines */


#ifdef __cplusplus
}
#endif

#endif /* __CORE_CM55_H_GENERIC */

#ifndef __CMSIS_GENERIC

#ifndef __CORE_CM55_H_DEPENDANT
#define __CORE_CM55_H_DEPENDANT

#ifdef __cplusplus
 extern "C" {
#endif

/* check device defines and use defaults */
#if defined __CHECK_DEVICE_DEFINES
  #ifndef __CM55_REV
    #define __CM55_REV               0x0000U
    #warning "__CM55_REV not defined in device header file; using default!"
  #endif

  #ifndef __FPU_PRESENT
    #define __FPU_PRESENT             0U
    #warning "__FPU_PRESENT not defined in device header file; using default!"
  #endif

  #if __FPU_PRESENT != 0U
    #ifndef __FPU_DP
      #define __FPU_DP             0U
      #warning "__FPU_DP not defined in device header file; using default!"
    #endif
  #endif

  #ifndef __MPU_PRESENT
    #define __MPU_PRESENT             0U
    #warning "__MPU_PRESENT not defined in device header file; using default!"
  #endif

  #ifndef __ICACHE_PRESENT
    #define __ICACHE_PRESENT          0U
    #warning "__ICACHE_PRESENT not defined in device header file; using default!"
  #endif

  #ifndef __DCACHE_PRESENT
    #define __DCACHE_PRESENT          0U
    #warning "__DCACHE_PRESENT not defined in device header file; using default!"
  #endif

  #ifndef __VTOR_PRESENT
    #define __VTOR_PRESENT             1U
    #warning "__VTOR_PRESENT not defined in device header file; using default!"
  #endif

  #ifndef __PMU_PRESENT
    #define __PMU_PRESENT             0U
    #warning "__PMU_PRESENT not defined in device header file; using default!"
  #endif

  #if __PMU_PRESENT != 0U
    #ifndef __PMU_NUM_EVENTCNT
      #define __PMU_NUM_EVENTCNT      8U
      #warning "__PMU_NUM_EVENTCNT not defined in device header file; using default!"
    #elif (__PMU_NUM_EVENTCNT > 8 || __PMU_NUM_EVENTCNT < 2)
    #error "__PMU_NUM_EVENTCNT is out of range in device header file!" */
    #endif
  #endif

  #ifndef __SAUREGION_PRESENT
    #define __SAUREGION_PRESENT       0U
    #warning "__SAUREGION_PRESENT not defined in device header file; using default!"
  #endif

  #ifndef __DSP_PRESENT
    #define __DSP_PRESENT             0U
    #warning "__DSP_PRESENT not defined in device header file; using default!"
  #endif

  #ifndef __NVIC_PRIO_BITS
    #define __NVIC_PRIO_BITS          3U
    #warning "__NVIC_PRIO_BITS not defined in device header file; using default!"
  #endif

  #ifndef __Vendor_SysTickConfig
    #define __Vendor_SysTickConfig    0U
    #warning "__Vendor_SysTickConfig not defined in device header file; using default!"
  #endif
#endif

/* IO definitions (access restrictions to peripheral registers) */
/**
    \defgroup CMSIS_glob_defs CMSIS Global Defines

    <strong>IO Type Qualifiers</strong> are used
    \li to specify the access to peripheral variables.
    \li for automatic generation of peripheral register debug information.
*/
#ifdef __cplusplus
  #define   __I     volatile             /*!< Defines 'read only' permissions */
#else
  #define   __I     volatile const       /*!< Defines 'read only' permissions */
#endif
#define     __O     volatile             /*!< Defines 'write only' permissions */
#define     __IO    volatile             /*!< Defines 'read / write' permissions */

/* following defines should be used for structure members */
#define     __IM     volatile const      /*! Defines 'read only' structure member permissions */
#define     __OM     volatile            /*! Defines 'write only' structure member permissions */
#define     __IOM    volatile            /*! Defines 'read / write' structure member permissions */

/*@} end of group Cortex_M55 */



/*******************************************************************************
 *                 Register Abstraction
  Core Register contain:
  - Core Register
  - Core NVIC Register
  - Core EWIC Register
  - Core SCB Register
  - Core SysTick Register
  - Core Debug Register
  - Core PMU Register
  - Core MPU Register
  - Core SAU Register
  - Core FPU Register
 ******************************************************************************/
/**
  \defgroup CMSIS_core_register Defines and Type Definitions
  \brief Type definitions and defines for Cortex-M processor based devices.
*/

/**
  \ingroup    CMSIS_core_register
  \defgroup   CMSIS_CORE  Status and Control Registers
  \brief      Core Register type definitions.
  @{
 */

/**
  \brief  Union type to access the Application Program Status Register (APSR).
 */
typedef union
{
  struct
  {
    uint32_t _reserved0:16;              /*!< bit:  0..15  Reserved */
    uint32_t GE:4;                       /*!< bit: 16..19  Greater than or Equal flags */
    uint32_t _reserved1:7;               /*!< bit: 20..26  Reserved */
    uint32_t Q:1;                        /*!< bit:     27  Saturation condition flag */
    uint32_t V:1;                        /*!< bit:     28  Overflow condition code flag */
    uint32_t C:1;                        /*!< bit:     29  Carry condition code flag */
    uint32_t Z:1;                        /*!< bit:     30  Zero condition code flag */
    uint32_t N:1;                        /*!< bit:     31  Negative condition code flag */
  } b;                                   /*!< Structure used for bit  access */
  uint32_t w;                            /*!< Type      used for word access */
} APSR_Type;

/* APSR Register Definitions */
#define APSR_N_Pos                         31U                                            /*!< APSR: N Position */
#define APSR_N_Msk                         (1UL << APSR_N_Pos)                            /*!< APSR: N Mask */

#define APSR_Z_Pos                         30U                                            /*!< APSR: Z Position */
#define APSR_Z_Msk                         (1UL << APSR_Z_Pos)                            /*!< APSR: Z Mask */

#define APSR_C_Pos                         29U                                            /*!< APSR: C Position */
#define APSR_C_Msk                         (1UL << APSR_C_Pos)                            /*!< APSR: C Mask */

#define APSR_V_Pos                         28U                                            /*!< APSR: V Position */
#define APSR_V_Msk                         (1UL << APSR_V_Pos)                            /*!< APSR: V Mask */

#define APSR_Q_Pos                         27U                                            /*!< APSR: Q Position */
#define APSR_Q_Msk                         (1UL << APSR_Q_Pos)                            /*!< APSR: Q Mask */

#define APSR_GE_Pos                        16U                                            /*!< APSR: GE Position */
#define APSR_GE_Msk                        (0xFUL << APSR_GE_Pos)                         /*!< APSR: GE Mask */


/**
  \brief  Union type to access the Interrupt Program Status Register (IPSR).
 */
typedef union
{
  struct
  {
    uint32_t ISR:9;                      /*!< bit:  0.. 8  Exception number */
    uint32_t _reserved0:23;              /*!< bit:  9..31  Reserved */
  } b;                                   /*!< Structure used for bit  access */
  uint32_t w;                            /*!< Type      used for word access */
} IPSR_Type;

/* IPSR Register Definitions */
#define IPSR_ISR_Pos                        0U                                            /*!< IPSR: ISR Position */
#define IPSR_ISR_Msk                       (0x1FFUL /*<< IPSR_ISR_Pos*/)                  /*!< IPSR: ISR Mask */


/**
  \brief  Union type to access the Special-Purpose Program Status Registers (xPSR).
 */
typedef union
{
  struct
  {
    uint32_t ISR:9;                      /*!< bit:  0.. 8  Exception number */
    uint32_t _reserved0:7;               /*!< bit:  9..15  Reserved */
    uint32_t GE:4;                       /*!< bit: 16..19  Greater than or Equal flags */
    uint32_t _reserved1:4;               /*!< bit: 20..23  Reserved */
    uint32_t T:1;                        /*!< bit:     24  Thumb bit        (read 0) */
    uint32_t IT:2;                       /*!< bit: 25..26  saved IT state   (read 0) */
    uint32_t Q:1;                        /*!< bit:     27  Saturation condition flag */
    uint32_t V:1;                        /*!< bit:     28  Overflow condition code flag */
    uint32_t C:1;                        /*!< bit:     29  Carry condition code flag */
    uint32_t Z:1;                        /*!< bit:     30  Zero condition code flag */
    uint32_t N:1;                        /*!< bit:     31  Negative condition code flag */
  } b;                                   /*!< Structure used for bit  access */
  uint32_t w;                            /*!< Type      used for word access */
} xPSR_Type;

/* xPSR Register Definitions */
#define xPSR_N_Pos                         31U                                            /*!< xPSR: N Position */
#define xPSR_N_Msk                         (1UL << xPSR_N_Pos)                            /*!< xPSR: N Mask */

#define xPSR_Z_Pos                         30U                                            /*!< xPSR: Z Position */
#define xPSR_Z_Msk                         (1UL << xPSR_Z_Pos)                            /*!< xPSR: Z Mask */

#define xPSR_C_Pos                         29U                                            /*!< xPSR: C Position */
#define xPSR_C_Msk                         (1UL << xPSR_C_Pos)                            /*!< xPSR: C Mask */

#define xPSR_V_Pos                         28U                                            /*!< xPSR: V Position */
#define xPSR_V_Msk                         (1UL << xPSR_V_Pos)                            /*!< xPSR: V Mask */

#define xPSR_Q_Pos                         27U                                            /*!< xPSR: Q Position */
#define xPSR_Q_Msk                         (1UL << xPSR_Q_Pos)                            /*!< xPSR: Q Mask */

#define xPSR_IT_Pos                        25U                                            /*!< xPSR: IT Position */
#define xPSR_IT_Msk                        (3UL << xPSR_IT_Pos)                           /*!< xPSR: IT Mask */

#define xPSR_T_Pos                         24U                                            /*!< xPSR: T Position */
#define xPSR_T_Msk                         (1UL << xPSR_T_Pos)                            /*!< xPSR: T Mask */

#define xPSR_GE_Pos                        16U                                            /*!< xPSR: GE Position */
#define xPSR_GE_Msk                        (0xFUL << xPSR_GE_Pos)                         /*!< xPSR: GE Mask */

#define xPSR_ISR_Pos                        0U                                            /*!< xPSR: ISR Position */
#define xPSR_ISR_Msk                       (0x1FFUL /*<< xPSR_ISR_Pos*/)                  /*!< xPSR: ISR Mask */


/**
  \brief  Union type to access the Control Registers (CONTROL).
 */
typedef union
{
  struct
  {
    uint32_t nPRIV:1;                    /*!< bit:      0  Execution privilege in Thread mode */
    uint32_t SPSEL:1;                    /*!< bit:      1  Stack-pointer select */
    uint32_t FPCA:1;                     /*!< bit:      2  Floating-point context active */
    uint32_t SFPA:1;                     /*!< bit:      3  Secure floating-point active */
    uint32_t _reserved1:28;              /*!< bit:  4..31  Reserved */
  } b;                                   /*!< Structure used for bit  access */
  uint32_t w;                            /*!< Type      used for word access */
} CONTROL_Type;

/* CONTROL Register Definitions */
#define CONTROL_SFPA_Pos                    3U                                            /*!< CONTROL: SFPA Position */
#define CONTROL_SFPA_Msk                   (1UL << CONTROL_SFPA_Pos)                      /*!< CONTROL: SFPA Mask */

#define CONTROL_FPCA_Pos                    2U                                            /*!< CONTROL: FPCA Position */
#define CONTROL_FPCA_Msk                   (1UL << CONTROL_FPCA_Pos)                      /*!< CONTROL: FPCA Mask */

#define CONTROL_SPSEL_Pos                   1U                                            /*!< CONTROL: SPSEL Position */
#define CONTROL_SPSEL_Msk                  (1UL << CONTROL_SPSEL_Pos)                     /*!< CONTROL: SPSEL Mask */

#define CONTROL_nPRIV_Pos                   0U                                            /*!< CONTROL: nPRIV Position */
#define CONTROL_nPRIV_Msk                  (1UL /*<< CONTROL_nPRIV_Pos*/)                 /*!< CONTROL: nPRIV Mask */

/*@} end of group CMSIS_CORE */


/**
  \ingroup    CMSIS_core_register
  \defgroup   CMSIS_NVIC  Nested Vectored Interrupt Controller (NVIC)
  \brief      Type definitions for the NVIC Registers
  @{
 */

/**
  \brief  Structure type to access the Nested Vectored Interrupt Controller (NVIC).
 */
typedef struct
{
  __IOM uint32_t ISER[16U];              /*!< Offset: 0x000 (R/W)  Interrupt Set Enable Register */
        uint32_t RESERVED0[16U];
  __IOM uint32_t ICER[16U];              /*!< Offset: 0x080 (R/W)  Interrupt Clear Enable Register */
        uint32_t RSERVED1[16U];
  __IOM uint32_t ISPR[16U];              /*!< Offset: 0x100 (R/W)  Interrupt Set Pending Register */
        uint32_t RESERVED2[16U];
  __IOM uint32_t ICPR[16U];              /*!< Offset: 0x180 (R/W)  Interrupt Clear Pending Register */
        uint32_t RESERVED3[16U];
  __IOM uint32_t IABR[16U];              /*!< Offset: 0x200 (R/W)  Interrupt Active bit Register */
        uint32_t RESERVED4[16U];
  __IOM uint32_t ITNS[16U];              /*!< Offset: 0x280 (R/W)  Interrupt Non-Secure State Register */
        uint32_t RESERVED5[16U];
  __IOM uint8_t  IPR[496U];              /*!< Offset: 0x300 (R/W)  Interrupt Priority Register (8Bit wide) */
        uint32_t RESERVED6[580U];
  __OM  uint32_t STIR;                   /*!< Offset: 0xE00 ( /W)  Software Trigger Interrupt Register */
}  NVIC_Type;

/* Software Triggered Interrupt Register Definitions */
#define NVIC_STIR_INTID_Pos                 0U                                         /*!< STIR: INTLINESNUM Position */
#define NVIC_STIR_INTID_Msk                (0x1FFUL /*<< NVIC_STIR_INTID_Pos*/)        /*!< STIR: INTLINESNUM Mask */

/*@} end of group CMSIS_NVIC */


/**
  \ingroup  CMSIS_core_register
  \defgroup CMSIS_SCB     System Control Block (SCB)
  \brief    Type definitions for the System Control Block Registers
  @{
 */

/**
  \brief  Structure type to access the System Control Block (SCB).
 */
typedef struct
{
  __IM  uint32_t CPUID;                  /*!< Offset: 0x000 (R/ )  CPUID Base Register */
  __IOM uint32_t ICSR;                   /*!< Offset: 0x004 (R/W)  Interrupt Control and State Register */
  __IOM uint32_t VTOR;                   /*!< Offset: 0x008 (R/W)  Vector Table Offset Register */
  __IOM uint32_t AIRCR;                  /*!< Offset: 0x00C (R/W)  Application Interrupt and Reset Control Register */
  __IOM uint32_t SCR;                    /*!< Offset: 0x010 (R/W)  System Control Register */
  __IOM uint32_t CCR;                    /*!< Offset: 0x014 (R/W)  Configuration Control Register */
  __IOM uint8_t  SHPR[12U];              /*!< Offset: 0x018 (R/W)  System Handlers Priority Registers (4-7, 8-11, 12-15) */
  __IOM uint32_t SHCSR;                  /*!< Offset: 0x024 (R/W)  System Handler Control and State Register */
  __IOM uint32_t CFSR;                   /*!< Offset: 0x028 (R/W)  Configurable Fault Status Register */
  __IOM uint32_t HFSR;                   /*!< Offset: 0x02C (R/W)  HardFault Status Register */
  __IOM uint32_t DFSR;                   /*!< Offset: 0x030 (R/W)  Debug Fault Status Register */
  __IOM uint32_t MMFAR;                  /*!< Offset: 0x034 (R/W)  MemManage Fault Address Register */
  __IOM uint32_t BFAR;                   /*!< Offset: 0x038 (R/W)  BusFault Address Register */
  __IOM uint32_t AFSR;                   /*!< Offset: 0x03C (R/W)  Auxiliary Fault Status Register */
  __IM  uint32_t ID_PFR[2U];             /*!< Offset: 0x040 (R/ )  Processor Feature Register */
  __IM  uint32_t ID_DFR;                 /*!< Offset: 0x048 (R/ )  Debug Feature Register */
  __IM  uint32_t ID_AFR;                 /*!< Offset: 0x04C (R/ )  Auxiliary Feature Register */
  __IM  uint32_t ID_MMFR[4U];            /*!< Offset: 0x050 (R/ )  Memory Model Feature Register */
  __IM  uint32_t ID_ISAR[6U];            /*!< Offset: 0x060 (R/ )  Instruction Set Attributes Register */
  __IM  uint32_t CLIDR;                  /*!< Offset: 0x078 (R/ )  Cache Level ID register */
  __IM  uint32_t CTR;                    /*!< Offset: 0x07C (R/ )  Cache Type register */
  __IM  uint32_t CCSIDR;                 /*!< Offset: 0x080 (R/ )  Cache Size ID Register */
  __IOM uint32_t CSSELR;                 /*!< Offset: 0x084 (R/W)  Cache Size Selection Register */
  __IOM uint32_t CPACR;                  /*!< Offset: 0x088 (R/W)  Coprocessor Access Control Register */
  __IOM uint32_t NSACR;                  /*!< Offset: 0x08C (R/W)  Non-Secure Access Control Register */
        uint32_t RESERVED7[21U];
  __IOM uint32_t SFSR;                   /*!< Offset: 0x0E4 (R/W)  Secure Fault Status Register */
  __IOM uint32_t SFAR;                   /*!< Offset: 0x0E8 (R/W)  Secure Fault Address Register */
        uint32_t RESERVED3[69U];
  __OM  uint32_t STIR;                   /*!< Offset: 0x200 ( /W)  Software Triggered Interrupt Register */
  __IOM uint32_t RFSR;                   /*!< Offset: 0x204 (R/W)  RAS Fault Status Register */
        uint32_t RESERVED4[14U];
  __IM  uint32_t MVFR0;                  /*!< Offset: 0x240 (R/ )  Media and VFP Feature Register 0 */
  __IM  uint32_t MVFR1;                  /*!< Offset: 0x244 (R/ )  Media and VFP Feature Register 1 */
  __IM  uint32_t MVFR2;                  /*!< Offset: 0x248 (R/ )  Media and VFP Feature Register 2 */
        uint32_t RESERVED5[1U];
  __OM  uint32_t ICIALLU;                /*!< Offset: 0x250 ( /W)  I-Cache Invalidate All to PoU */
        uint32_t RESERVED6[1U];
  __OM  uint32_t ICIMVAU;                /*!< Offset: 0x258 ( /W)  I-Cache Invalidate by MVA to PoU */
  __OM  uint32_t DCIMVAC;                /*!< Offset: 0x25C ( /W)  D-Cache Invalidate by MVA to PoC */
  __OM  uint32_t DCISW;                  /*!< Offset: 0x260 ( /W)  D-Cache Invalidate by Set-way */
  __OM  uint32_t DCCMVAU;                /*!< Offset: 0x264 ( /W)  D-Cache Clean by MVA to PoU */
  __OM  uint32_t DCCMVAC;                /*!< Offset: 0x268 ( /W)  D-Cache Clean by MVA to PoC */
  __OM  uint32_t DCCSW;                  /*!< Offset: 0x26C ( /W)  D-Cache Clean by Set-way */
  __OM  uint32_t DCCIMVAC;               /*!< Offset: 0x270 ( /W)  D-Cache Clean and Invalidate by MVA to PoC */
  __OM  uint32_t DCCISW;                 /*!< Offset: 0x274 ( /W)  D-Cache Clean and Invalidate by Set-way */
  __OM  uint32_t BPIALL;                 /*!< Offset: 0x278 ( /W)  Branch Predictor Invalidate All */
} SCB_Type;

/* SCB CPUID Register Definitions */
#define SCB_CPUID_IMPLEMENTER_Pos          24U                                            /*!< SCB CPUID: IMPLEMENTER Position */
#define SCB_CPUID_IMPLEMENTER_Msk          (0xFFUL << SCB_CPUID_IMPLEMENTER_Pos)          /*!< SCB CPUID: IMPLEMENTER Mask */

#define SCB_CPUID_VARIANT_Pos              20U                                            /*!< SCB CPUID: VARIANT Position */
#define SCB_CPUID_VARIANT_Msk              (0xFUL << SCB_CPUID_VARIANT_Pos)               /*!< SCB CPUID: VARIANT Mask */

#define SCB_CPUID_ARCHITECTURE_Pos         16U                                            /*!< SCB CPUID: ARCHITECTURE Position */
#define SCB_CPUID_ARCHITECTURE_Msk         (0xFUL << SCB_CPUID_ARCHITECTURE_Pos)          /*!< SCB CPUID: ARCHITECTURE Mask */

#define SCB_CPUID_PARTNO_Pos                4U                                            /*!< SCB CPUID: PARTNO Position */
#define SCB_CPUID_PARTNO_Msk               (0xFFFUL << SCB_CPUID_PARTNO_Pos)              /*!< SCB CPUID: PARTNO Mask */

#define SCB_CPUID_REVISION_Pos              0U                                            /*!< SCB CPUID: REVISION Position */
#define SCB_CPUID_REVISION_Msk             (0xFUL /*<< SCB_CPUID_REVISION_Pos*/)          /*!< SCB CPUID: REVISION Mask */

/* SCB Interrupt Control State Register Definitions */
#define SCB_ICSR_PENDNMISET_Pos            31U                                            /*!< SCB ICSR: PENDNMISET Position */
#define SCB_ICSR_PENDNMISET_Msk            (1UL << SCB_ICSR_PENDNMISET_Pos)               /*!< SCB ICSR: PENDNMISET Mask */

#define SCB_ICSR_NMIPENDSET_Pos            SCB_ICSR_PENDNMISET_Pos                        /*!< SCB ICSR: NMIPENDSET Position, backward compatibility */
#define SCB_ICSR_NMIPENDSET_Msk            SCB_ICSR_PENDNMISET_Msk                        /*!< SCB ICSR: NMIPENDSET Mask, backward compatibility */

#define SCB_ICSR_PENDNMICLR_Pos            30U                                            /*!< SCB ICSR: PENDNMICLR Position */
#define SCB_ICSR_PENDNMICLR_Msk            (1UL << SCB_ICSR_PENDNMICLR_Pos)               /*!< SCB ICSR: PENDNMICLR Mask */

#define SCB_ICSR_PENDSVSET_Pos             28U                                            /*!< SCB ICSR: PENDSVSET Position */
#define SCB_ICSR_PENDSVSET_Msk             (1UL << SCB_ICSR_PENDSVSET_Pos)                /*!< SCB ICSR: PENDSVSET Mask */

#define SCB_ICSR_PENDSVCLR_Pos             27U                                            /*!< SCB ICSR: PENDSVCLR Position */
#define SCB_ICSR_PENDSVCLR_Msk             (1UL << SCB_ICSR_PENDSVCLR_Pos)                /*!< SCB ICSR: PENDSVCLR Mask */

#define SCB_ICSR_PENDSTSET_Pos             26U                                            /*!< SCB ICSR: PENDSTSET Position */
#define SCB_ICSR_PENDSTSET_Msk             (1UL << SCB_ICSR_PENDSTSET_Pos)                /*!< SCB ICSR: PENDSTSET Mask */

#define SCB_ICSR_PENDSTCLR_Pos             25U                                            /*!< SCB ICSR: PENDSTCLR Position */
#define SCB_ICSR_PENDSTCLR_Msk             (1UL << SCB_ICSR_PENDSTCLR_Pos)                /*!< SCB ICSR: PENDSTCLR Mask */

#define SCB_ICSR_STTNS_Pos                 24U                                            /*!< SCB ICSR: STTNS Position (Security Extension) */
#define SCB_ICSR_STTNS_Msk                 (1UL << SCB_ICSR_STTNS_Pos)                    /*!< SCB ICSR: STTNS Mask (Security Extension) */

#define SCB_ICSR_ISRPREEMPT_Pos            23U                                            /*!< SCB ICSR: ISRPREEMPT Position */
#define SCB_ICSR_ISRPREEMPT_Msk            (1UL << SCB_ICSR_ISRPREEMPT_Pos)               /*!< SCB ICSR: ISRPREEMPT Mask */

#define SCB_ICSR_ISRPENDING_Pos            22U                                            /*!< SCB ICSR: ISRPENDING Position */
#define SCB_ICSR_ISRPENDING_Msk            (1UL << SCB_ICSR_ISRPENDING_Pos)               /*!< SCB ICSR: ISRPENDING Mask */

#define SCB_ICSR_VECTPENDING_Pos           12U                                            /*!< SCB ICSR: VECTPENDING Position */
#define SCB_ICSR_VECTPENDING_Msk           (0x1FFUL << SCB_ICSR_VECTPENDING_Pos)          /*!< SCB ICSR: VECTPENDING Mask */

#define SCB_ICSR_RETTOBASE_Pos             11U                                            /*!< SCB ICSR: RETTOBASE Position */
#define SCB_ICSR_RETTOBASE_Msk             (1UL << SCB_ICSR_RETTOBASE_Pos)                /*!< SCB ICSR: RETTOBASE Mask */

#define SCB_ICSR_VECTACTIVE_Pos             0U                                            /*!< SCB ICSR: VECTACTIVE Position */
#define SCB_ICSR_VECTACTIVE_Msk            (0x1FFUL /*<< SCB_ICSR_VECTACTIVE_Pos*/)       /*!< SCB ICSR: VECTACTIVE Mask */

/* SCB Vector Table Offset Register Definitions */
#define SCB_VTOR_TBLOFF_Pos                 7U                                            /*!< SCB VTOR: TBLOFF Position */
#define SCB_VTOR_TBLOFF_Msk                (0x1FFFFFFUL << SCB_VTOR_TBLOFF_Pos)           /*!< SCB VTOR: TBLOFF Mask */

/* SCB Application Interrupt and Reset Control Register Definitions */
#define SCB_AIRCR_VECTKEY_Pos              16U                                            /*!< SCB AIRCR: VECTKEY Position */
#define SCB_AIRCR_VECTKEY_Msk              (0xFFFFUL << SCB_AIRCR_VECTKEY_Pos)            /*!< SCB AIRCR: VECTKEY Mask */

#define SCB_AIRCR_VECTKEYSTAT_Pos          16U                                            /*!< SCB AIRCR: VECTKEYSTAT Position */
#define SCB_AIRCR_VECTKEYSTAT_Msk          (0xFFFFUL << SCB_AIRCR_VECTKEYSTAT_Pos)        /*!< SCB AIRCR: VECTKEYSTAT Mask */

#define SCB_AIRCR_ENDIANESS_Pos            15U                                            /*!< SCB AIRCR: ENDIANESS Position */
#define SCB_AIRCR_ENDIANESS_Msk            (1UL << SCB_AIRCR_ENDIANESS_Pos)               /*!< SCB AIRCR: ENDIANESS Mask */

#define SCB_AIRCR_PRIS_Pos                 14U                                            /*!< SCB AIRCR: PRIS Position */
#define SCB_AIRCR_PRIS_Msk                 (1UL << SCB_AIRCR_PRIS_Pos)                    /*!< SCB AIRCR: PRIS Mask */

#define SCB_AIRCR_BFHFNMINS_Pos            13U                                            /*!< SCB AIRCR: BFHFNMINS Position */
#define SCB_AIRCR_BFHFNMINS_Msk            (1UL << SCB_AIRCR_BFHFNMINS_Pos)               /*!< SCB AIRCR: BFHFNMINS Mask */

#define SCB_AIRCR_PRIGROUP_Pos              8U                                            /*!< SCB AIRCR: PRIGROUP Position */
#define SCB_AIRCR_PRIGROUP_Msk             (7UL << SCB_AIRCR_PRIGROUP_Pos)                /*!< SCB AIRCR: PRIGROUP Mask */

#define SCB_AIRCR_IESB_Pos                  5U                                            /*!< SCB AIRCR: Implicit ESB Enable Position */
#define SCB_AIRCR_IESB_Msk                 (1UL << SCB_AIRCR_IESB_Pos)                    /*!< SCB AIRCR: Implicit ESB Enable Mask */

#define SCB_AIRCR_DIT_Pos                   4U                                            /*!< SCB AIRCR: Data Independent Timing Position */
#define SCB_AIRCR_DIT_Msk                  (1UL << SCB_AIRCR_DIT_Pos)                     /*!< SCB AIRCR: Data Independent Timing Mask */

#define SCB_AIRCR_SYSRESETREQS_Pos          3U                                            /*!< SCB AIRCR: SYSRESETREQS Position */
#define SCB_AIRCR_SYSRESETREQS_Msk         (1UL << SCB_AIRCR_SYSRESETREQS_Pos)            /*!< SCB AIRCR: SYSRESETREQS Mask */

#define SCB_AIRCR_SYSRESETREQ_Pos           2U                                            /*!< SCB AIRCR: SYSRESETREQ Position */
#define SCB_AIRCR_SYSRESETREQ_Msk          (1UL << SCB_AIRCR_SYSRESETREQ_Pos)             /*!< SCB AIRCR: SYSRESETREQ Mask */

#define SCB_AIRCR_VECTCLRACTIVE_Pos         1U                                            /*!< SCB AIRCR: VECTCLRACTIVE Position */
#define SCB_AIRCR_VECTCLRACTIVE_Msk        (1UL << SCB_AIRCR_VECTCLRACTIVE_Pos)           /*!< SCB AIRCR: VECTCLRACTIVE Mask */

/* SCB System Control Register Definitions */
#define SCB_SCR_SEVONPEND_Pos               4U                                            /*!< SCB SCR: SEVONPEND Position */
#define SCB_SCR_SEVONPEND_Msk              (1UL << SCB_SCR_SEVONPEND_Pos)                 /*!< SCB SCR: SEVONPEND Mask */

#define SCB_SCR_SLEEPDEEPS_Pos              3U                                            /*!< SCB SCR: SLEEPDEEPS Position */
#define SCB_SCR_SLEEPDEEPS_Msk             (1UL << SCB_SCR_SLEEPDEEPS_Pos)                /*!< SCB SCR: SLEEPDEEPS Mask */

#define SCB_SCR_SLEEPDEEP_Pos               2U                                            /*!< SCB SCR: SLEEPDEEP Position */
#define SCB_SCR_SLEEPDEEP_Msk              (1UL << SCB_SCR_SLEEPDEEP_Pos)                 /*!< SCB SCR: SLEEPDEEP Mask */

#define SCB_SCR_SLEEPONEXIT_Pos             1U                                            /*!< SCB SCR: SLEEPONEXIT Position */
#define SCB_SCR_SLEEPONEXIT_Msk            (1UL << SCB_SCR_SLEEPONEXIT_Pos)               /*!< SCB SCR: SLEEPONEXIT Mask */

/* SCB Configuration Control Register Definitions */
#define SCB_CCR_TRD_Pos                    20U                                            /*!< SCB CCR: TRD Position */
#define SCB_CCR_TRD_Msk                    (1UL << SCB_CCR_TRD_Pos)                       /*!< SCB CCR: TRD Mask */

#define SCB_CCR_LOB_Pos                    19U                                            /*!< SCB CCR: LOB Position */
#define SCB_CCR_LOB_Msk                    (1UL << SCB_CCR_LOB_Pos)                       /*!< SCB CCR: LOB Mask */

#define SCB_CCR_BP_Pos                     18U                                            /*!< SCB CCR: BP Position */
#define SCB_CCR_BP_Msk                     (1UL << SCB_CCR_BP_Pos)                        /*!< SCB CCR: BP Mask */

#define SCB_CCR_IC_Pos                     17U                                            /*!< SCB CCR: IC Position */
#define SCB_CCR_IC_Msk                     (1UL << SCB_CCR_IC_Pos)                        /*!< SCB CCR: IC Mask */

#define SCB_CCR_DC_Pos                     16U                                            /*!< SCB CCR: DC Position */
#define SCB_CCR_DC_Msk                     (1UL << SCB_CCR_DC_Pos)                        /*!< SCB CCR: DC Mask */

#define SCB_CCR_STKOFHFNMIGN_Pos           10U                                            /*!< SCB CCR: STKOFHFNMIGN Position */
#define SCB_CCR_STKOFHFNMIGN_Msk           (1UL << SCB_CCR_STKOFHFNMIGN_Pos)              /*!< SCB CCR: STKOFHFNMIGN Mask */

#define SCB_CCR_BFHFNMIGN_Pos               8U                                            /*!< SCB CCR: BFHFNMIGN Position */
#define SCB_CCR_BFHFNMIGN_Msk              (1UL << SCB_CCR_BFHFNMIGN_Pos)                 /*!< SCB CCR: BFHFNMIGN Mask */

#define SCB_CCR_DIV_0_TRP_Pos               4U                                            /*!< SCB CCR: DIV_0_TRP Position */
#define SCB_CCR_DIV_0_TRP_Msk              (1UL << SCB_CCR_DIV_0_TRP_Pos)                 /*!< SCB CCR: DIV_0_TRP Mask */

#define SCB_CCR_UNALIGN_TRP_Pos             3U                                            /*!< SCB CCR: UNALIGN_TRP Position */
#define SCB_CCR_UNALIGN_TRP_Msk            (1UL << SCB_CCR_UNALIGN_TRP_Pos)               /*!< SCB CCR: UNALIGN_TRP Mask */

#define SCB_CCR_USERSETMPEND_Pos            1U                                            /*!< SCB CCR: USERSETMPEND Position */
#define SCB_CCR_USERSETMPEND_Msk           (1UL << SCB_CCR_USERSETMPEND_Pos)              /*!< SCB CCR: USERSETMPEND Mask */

/* SCB System Handler Control and State Register Definitions */
#define SCB_SHCSR_HARDFAULTPENDED_Pos      21U                                            /*!< SCB SHCSR: HARDFAULTPENDED Position */
#define SCB_SHCSR_HARDFAULTPENDED_Msk      (1UL << SCB_SHCSR_HARDFAULTPENDED_Pos)         /*!< SCB SHCSR: HARDFAULTPENDED Mask */

#define SCB_SHCSR_SECUREFAULTPENDED_Pos    20U                                            /*!< SCB SHCSR: SECUREFAULTPENDED Position */
#define SCB_SHCSR_SECUREFAULTPENDED_Msk    (1UL << SCB_SHCSR_SECUREFAULTPENDED_Pos)       /*!< SCB SHCSR: SECUREFAULTPENDED Mask */

#define SCB_SHCSR_SECUREFAULTENA_Pos       19U                                            /*!< SCB SHCSR: SECUREFAULTENA Position */
#define SCB_SHCSR_SECUREFAULTENA_Msk       (1UL << SCB_SHCSR_SECUREFAULTENA_Pos)          /*!< SCB SHCSR: SECUREFAULTENA Mask */

#define SCB_SHCSR_USGFAULTENA_Pos          18U                                            /*!< SCB SHCSR: USGFAULTENA Position */
#define SCB_SHCSR_USGFAULTENA_Msk          (1UL << SCB_SHCSR_USGFAULTENA_Pos)             /*!< SCB SHCSR: USGFAULTENA Mask */

#define SCB_SHCSR_BUSFAULTENA_Pos          17U                                            /*!< SCB SHCSR: BUSFAULTENA Position */
#define SCB_SHCSR_BUSFAULTENA_Msk          (1UL << SCB_SHCSR_BUSFAULTENA_Pos)             /*!< SCB SHCSR: BUSFAULTENA Mask */

#define SCB_SHCSR_MEMFAULTENA_Pos          16U                                            /*!< SCB SHCSR: MEMFAULTENA Position */
#define SCB_SHCSR_MEMFAULTENA_Msk          (1UL << SCB_SHCSR_MEMFAULTENA_Pos)             /*!< SCB SHCSR: MEMFAULTENA Mask */

#define SCB_SHCSR_SVCALLPENDED_Pos         15U                                            /*!< SCB SHCSR: SVCALLPENDED Position */
#define SCB_SHCSR_SVCALLPENDED_Msk         (1UL << SCB_SHCSR_SVCALLPENDED_Pos)            /*!< SCB SHCSR: SVCALLPENDED Mask */

#define SCB_SHCSR_BUSFAULTPENDED_Pos       14U                                            /*!< SCB SHCSR: BUSFAULTPENDED Position */
#define SCB_SHCSR_BUSFAULTPENDED_Msk       (1UL << SCB_SHCSR_BUSFAULTPENDED_Pos)          /*!< SCB SHCSR: BUSFAULTPENDED Mask */

#define SCB_SHCSR_MEMFAULTPENDED_Pos       13U                                            /*!< SCB SHCSR: MEMFAULTPENDED Position */
#define SCB_SHCSR_MEMFAULTPENDED_Msk       (1UL << SCB_SHCSR_MEMFAULTPENDED_Pos)          /*!< SCB SHCSR: MEMFAULTPENDED Mask */

#define SCB_SHCSR_USGFAULTPENDED_Pos       12U                                            /*!< SCB SHCSR: USGFAULTPENDED Position */
#define SCB_SHCSR_USGFAULTPENDED_Msk       (1UL << SCB_SHCSR_USGFAULTPENDED_Pos)          /*!< SCB SHCSR: USGFAULTPENDED Mask */

#define SCB_SHCSR_SYSTICKACT_Pos           11U                                            /*!< SCB SHCSR: SYSTICKACT Position */
#define SCB_SHCSR_SYSTICKACT_Msk           (1UL << SCB_SHCSR_SYSTICKACT_Pos)              /*!< SCB SHCSR: SYSTICKACT Mask */

#define SCB_SHCSR_PENDSVACT_Pos            10U                                            /*!< SCB SHCSR: PENDSVACT Position */
#define SCB_SHCSR_PENDSVACT_Msk            (1UL << SCB_SHCSR_PENDSVACT_Pos)               /*!< SCB SHCSR: PENDSVACT Mask */

#define SCB_SHCSR_MONITORACT_Pos            8U                                            /*!< SCB SHCSR: MONITORACT Position */
#define SCB_SHCSR_MONITORACT_Msk           (1UL << SCB_SHCSR_MONITORACT_Pos)              /*!< SCB SHCSR: MONITORACT Mask */

#define SCB_SHCSR_SVCALLACT_Pos             7U                                            /*!< SCB SHCSR: SVCALLACT Position */
#define SCB_SHCSR_SVCALLACT_Msk            (1UL << SCB_SHCSR_SVCALLACT_Pos)               /*!< SCB SHCSR: SVCALLACT Mask */

#define SCB_SHCSR_NMIACT_Pos                5U                                            /*!< SCB SHCSR: NMIACT Position */
#define SCB_SHCSR_NMIACT_Msk               (1UL << SCB_SHCSR_NMIACT_Pos)                  /*!< SCB SHCSR: NMIACT Mask */

#define SCB_SHCSR_SECUREFAULTACT_Pos        4U                                            /*!< SCB SHCSR: SECUREFAULTACT Position */
#define SCB_SHCSR_SECUREFAULTACT_Msk       (1UL << SCB_SHCSR_SECUREFAULTACT_Pos)          /*!< SCB SHCSR: SECUREFAULTACT Mask */

#define SCB_SHCSR_USGFAULTACT_Pos           3U                                            /*!< SCB SHCSR: USGFAULTACT Position */
#define SCB_SHCSR_USGFAULTACT_Msk          (1UL << SCB_SHCSR_USGFAULTACT_Pos)             /*!< SCB SHCSR: USGFAULTACT Mask */

#define SCB_SHCSR_HARDFAULTACT_Pos          2U                                            /*!< SCB SHCSR: HARDFAULTACT Position */
#define SCB_SHCSR_HARDFAULTACT_Msk         (1UL << SCB_SHCSR_HARDFAULTACT_Pos)            /*!< SCB SHCSR: HARDFAULTACT Mask */

#define SCB_SHCSR_BUSFAULTACT_Pos           1U                                            /*!< SCB SHCSR: BUSFAULTACT Position */
#define SCB_SHCSR_BUSFAULTACT_Msk          (1UL << SCB_SHCSR_BUSFAULTACT_Pos)             /*!< SCB SHCSR: BUSFAULTACT Mask */

#define SCB_SHCSR_MEMFAULTACT_Pos           0U                                            /*!< SCB SHCSR: MEMFAULTACT Position */
#define SCB_SHCSR_MEMFAULTACT_Msk          (1UL /*<< SCB_SHCSR_MEMFAULTACT_Pos*/)         /*!< SCB SHCSR: MEMFAULTACT Mask */

/* SCB Configurable Fault Status Register Definitions */
#define SCB_CFSR_USGFAULTSR_Pos            16U                                            /*!< SCB CFSR: Usage Fault Status Register Position */
#define SCB_CFSR_USGFAULTSR_Msk            (0xFFFFUL << SCB_CFSR_USGFAULTSR_Pos)          /*!< SCB CFSR: Usage Fault Status Register Mask */

#define SCB_CFSR_BUSFAULTSR_Pos             8U                                            /*!< SCB CFSR: Bus Fault Status Register Position */
#define SCB_CFSR_BUSFAULTSR_Msk            (0xFFUL << SCB_CFSR_BUSFAULTSR_Pos)            /*!< SCB CFSR: Bus Fault Status Register Mask */

#define SCB_CFSR_MEMFAULTSR_Pos             0U                                            /*!< SCB CFSR: Memory Manage Fault Status Register Position */
#define SCB_CFSR_MEMFAULTSR_Msk            (0xFFUL /*<< SCB_CFSR_MEMFAULTSR_Pos*/)        /*!< SCB CFSR: Memory Manage Fault Status Register Mask */

/* MemManage Fault Status Register (part of SCB Configurable Fault Status Register) */
#define SCB_CFSR_MMARVALID_Pos             (SCB_CFSR_MEMFAULTSR_Pos + 7U)                 /*!< SCB CFSR (MMFSR): MMARVALID Position */
#define SCB_CFSR_MMARVALID_Msk             (1UL << SCB_CFSR_MMARVALID_Pos)                /*!< SCB CFSR (MMFSR): MMARVALID Mask */

#define SCB_CFSR_MLSPERR_Pos               (SCB_CFSR_MEMFAULTSR_Pos + 5U)                 /*!< SCB CFSR (MMFSR): MLSPERR Position */
#define SCB_CFSR_MLSPERR_Msk               (1UL << SCB_CFSR_MLSPERR_Pos)                  /*!< SCB CFSR (MMFSR): MLSPERR Mask */

#define SCB_CFSR_MSTKERR_Pos               (SCB_CFSR_MEMFAULTSR_Pos + 4U)                 /*!< SCB CFSR (MMFSR): MSTKERR Position */
#define SCB_CFSR_MSTKERR_Msk               (1UL << SCB_CFSR_MSTKERR_Pos)                  /*!< SCB CFSR (MMFSR): MSTKERR Mask */

#define SCB_CFSR_MUNSTKERR_Pos             (SCB_CFSR_MEMFAULTSR_Pos + 3U)                 /*!< SCB CFSR (MMFSR): MUNSTKERR Position */
#define SCB_CFSR_MUNSTKERR_Msk             (1UL << SCB_CFSR_MUNSTKERR_Pos)                /*!< SCB CFSR (MMFSR): MUNSTKERR Mask */

#define SCB_CFSR_DACCVIOL_Pos              (SCB_CFSR_MEMFAULTSR_Pos + 1U)                 /*!< SCB CFSR (MMFSR): DACCVIOL Position */
#define SCB_CFSR_DACCVIOL_Msk              (1UL << SCB_CFSR_DACCVIOL_Pos)                 /*!< SCB CFSR (MMFSR): DACCVIOL Mask */

#define SCB_CFSR_IACCVIOL_Pos              (SCB_CFSR_MEMFAULTSR_Pos + 0U)                 /*!< SCB CFSR (MMFSR): IACCVIOL Position */
#define SCB_CFSR_IACCVIOL_Msk              (1UL /*<< SCB_CFSR_IACCVIOL_Pos*/)             /*!< SCB CFSR (MMFSR): IACCVIOL Mask */

/* BusFault Status Register (part of SCB Configurable Fault Status Register) */
#define SCB_CFSR_BFARVALID_Pos            (SCB_CFSR_BUSFAULTSR_Pos + 7U)                  /*!< SCB CFSR (BFSR): BFARVALID Position */
#define SCB_CFSR_BFARVALID_Msk            (1UL << SCB_CFSR_BFARVALID_Pos)                 /*!< SCB CFSR (BFSR): BFARVALID Mask */

#define SCB_CFSR_LSPERR_Pos               (SCB_CFSR_BUSFAULTSR_Pos + 5U)                  /*!< SCB CFSR (BFSR): LSPERR Position */
#define SCB_CFSR_LSPERR_Msk               (1UL << SCB_CFSR_LSPERR_Pos)                    /*!< SCB CFSR (BFSR): LSPERR Mask */

#define SCB_CFSR_STKERR_Pos               (SCB_CFSR_BUSFAULTSR_Pos + 4U)                  /*!< SCB CFSR (BFSR): STKERR Position */
#define SCB_CFSR_STKERR_Msk               (1UL << SCB_CFSR_STKERR_Pos)                    /*!< SCB CFSR (BFSR): STKERR Mask */

#define SCB_CFSR_UNSTKERR_Pos             (SCB_CFSR_BUSFAULTSR_Pos + 3U)                  /*!< SCB CFSR (BFSR): UNSTKERR Position */
#define SCB_CFSR_UNSTKERR_Msk             (1UL << SCB_CFSR_UNSTKERR_Pos)                  /*!< SCB CFSR (BFSR): UNSTKERR Mask */

#define SCB_CFSR_IMPRECISERR_Pos          (SCB_CFSR_BUSFAULTSR_Pos + 2U)                  /*!< SCB CFSR (BFSR): IMPRECISERR Position */
#define SCB_CFSR_IMPRECISERR_Msk          (1UL << SCB_CFSR_IMPRECISERR_Pos)               /*!< SCB CFSR (BFSR): IMPRECISERR Mask */

#define SCB_CFSR_PRECISERR_Pos            (SCB_CFSR_BUSFAULTSR_Pos + 1U)                  /*!< SCB CFSR (BFSR): PRECISERR Position */
#define SCB_CFSR_PRECISERR_Msk            (1UL << SCB_CFSR_PRECISERR_Pos)                 /*!< SCB CFSR (BFSR): PRECISERR Mask */

#define SCB_CFSR_IBUSERR_Pos              (SCB_CFSR_BUSFAULTSR_Pos + 0U)                  /*!< SCB CFSR (BFSR): IBUSERR Position */
#define SCB_CFSR_IBUSERR_Msk              (1UL << SCB_CFSR_IBUSERR_Pos)                   /*!< SCB CFSR (BFSR): IBUSERR Mask */

/* UsageFault Status Register (part of SCB Configurable Fault Status Register) */
#define SCB_CFSR_DIVBYZERO_Pos            (SCB_CFSR_USGFAULTSR_Pos + 9U)                  /*!< SCB CFSR (UFSR): DIVBYZERO Position */
#define SCB_CFSR_DIVBYZERO_Msk            (1UL << SCB_CFSR_DIVBYZERO_Pos)                 /*!< SCB CFSR (UFSR): DIVBYZERO Mask */

#define SCB_CFSR_UNALIGNED_Pos            (SCB_CFSR_USGFAULTSR_Pos + 8U)                  /*!< SCB CFSR (UFSR): UNALIGNED Position */
#define SCB_CFSR_UNALIGNED_Msk            (1UL << SCB_CFSR_UNALIGNED_Pos)                 /*!< SCB CFSR (UFSR): UNALIGNED Mask */

#define SCB_CFSR_STKOF_Pos                (SCB_CFSR_USGFAULTSR_Pos + 4U)                  /*!< SCB CFSR (UFSR): STKOF Position */
#define SCB_CFSR_STKOF_Msk                (1UL << SCB_CFSR_STKOF_Pos)                     /*!< SCB CFSR (UFSR): STKOF Mask */

#define SCB_CFSR_NOCP_Pos                 (SCB_CFSR_USGFAULTSR_Pos + 3U)                  /*!< SCB CFSR (UFSR): NOCP Position */
#define SCB_CFSR_NOCP_Msk                 (1UL << SCB_CFSR_NOCP_Pos)                      /*!< SCB CFSR (UFSR): NOCP Mask */

#define SCB_CFSR_INVPC_Pos                (SCB_CFSR_USGFAULTSR_Pos + 2U)                  /*!< SCB CFSR (UFSR): INVPC Position */
#define SCB_CFSR_INVPC_Msk                (1UL << SCB_CFSR_INVPC_Pos)                     /*!< SCB CFSR (UFSR): INVPC Mask */

#define SCB_CFSR_INVSTATE_Pos             (SCB_CFSR_USGFAULTSR_Pos + 1U)                  /*!< SCB CFSR (UFSR): INVSTATE Position */
#define SCB_CFSR_INVSTATE_Msk             (1UL << SCB_CFSR_INVSTATE_Pos)                  /*!< SCB CFSR (UFSR): INVSTATE Mask */

#define SCB_CFSR_UNDEFINSTR_Pos           (SCB_CFSR_USGFAULTSR_Pos + 0U)                  /*!< SCB CFSR (UFSR): UNDEFINSTR Position */
#define SCB_CFSR_UNDEFINSTR_Msk           (1UL << SCB_CFSR_UNDEFINSTR_Pos)                /*!< SCB CFSR (UFSR): UNDEFINSTR Mask */

/* SCB Hard Fault Status Register Definitions */
#define SCB_HFSR_DEBUGEVT_Pos              31U                                            /*!< SCB HFSR: DEBUGEVT Position */
#define SCB_HFSR_DEBUGEVT_Msk              (1UL << SCB_HFSR_DEBUGEVT_Pos)                 /*!< SCB HFSR: DEBUGEVT Mask */

#define SCB_HFSR_FORCED_Pos                30U                                            /*!< SCB HFSR: FORCED Position */
#define SCB_HFSR_FORCED_Msk                (1UL << SCB_HFSR_FORCED_Pos)                   /*!< SCB HFSR: FORCED Mask */

#define SCB_HFSR_VECTTBL_Pos                1U                                            /*!< SCB HFSR: VECTTBL Position */
#define SCB_HFSR_VECTTBL_Msk               (1UL << SCB_HFSR_VECTTBL_Pos)                  /*!< SCB HFSR: VECTTBL Mask */

/* SCB Debug Fault Status Register Definitions */
#define SCB_DFSR_PMU_Pos                    5U                                            /*!< SCB DFSR: PMU Position */
#define SCB_DFSR_PMU_Msk                   (1UL << SCB_DFSR_PMU_Pos)                      /*!< SCB DFSR: PMU Mask */

#define SCB_DFSR_EXTERNAL_Pos               4U                                            /*!< SCB DFSR: EXTERNAL Position */
#define SCB_DFSR_EXTERNAL_Msk              (1UL << SCB_DFSR_EXTERNAL_Pos)                 /*!< SCB DFSR: EXTERNAL Mask */

#define SCB_DFSR_VCATCH_Pos                 3U                                            /*!< SCB DFSR: VCATCH Position */
#define SCB_DFSR_VCATCH_Msk                (1UL << SCB_DFSR_VCATCH_Pos)                   /*!< SCB DFSR: VCATCH Mask */

#define SCB_DFSR_DWTTRAP_Pos                2U                                            /*!< SCB DFSR: DWTTRAP Position */
#define SCB_DFSR_DWTTRAP_Msk               (1UL << SCB_DFSR_DWTTRAP_Pos)                  /*!< SCB DFSR: DWTTRAP Mask */

#define SCB_DFSR_BKPT_Pos                   1U                                            /*!< SCB DFSR: BKPT Position */
#define SCB_DFSR_BKPT_Msk                  (1UL << SCB_DFSR_BKPT_Pos)                     /*!< SCB DFSR: BKPT Mask */

#define SCB_DFSR_HALTED_Pos                 0U                                            /*!< SCB DFSR: HALTED Position */
#define SCB_DFSR_HALTED_Msk                (1UL /*<< SCB_DFSR_HALTED_Pos*/)               /*!< SCB DFSR: HALTED Mask */

/* SCB Non-Secure Access Control Register Definitions */
#define SCB_NSACR_CP11_Pos                 11U                                            /*!< SCB NSACR: CP11 Position */
#define SCB_NSACR_CP11_Msk                 (1UL << SCB_NSACR_CP11_Pos)                    /*!< SCB NSACR: CP11 Mask */

#define SCB_NSACR_CP10_Pos                 10U                                            /*!< SCB NSACR: CP10 Position */
#define SCB_NSACR_CP10_Msk                 (1UL << SCB_NSACR_CP10_Pos)                    /*!< SCB NSACR: CP10 Mask */

#define SCB_NSACR_CP7_Pos                   7U                                            /*!< SCB NSACR: CP7 Position */
#define SCB_NSACR_CP7_Msk                  (1UL << SCB_NSACR_CP7_Pos)                     /*!< SCB NSACR: CP7 Mask */

#define SCB_NSACR_CP6_Pos                   6U                                            /*!< SCB NSACR: CP6 Position */
#define SCB_NSACR_CP6_Msk                  (1UL << SCB_NSACR_CP6_Pos)                     /*!< SCB NSACR: CP6 Mask */

#define SCB_NSACR_CP5_Pos                   5U                                            /*!< SCB NSACR: CP5 Position */
#define SCB_NSACR_CP5_Msk                  (1UL << SCB_NSACR_CP5_Pos)                     /*!< SCB NSACR: CP5 Mask */

#define SCB_NSACR_CP4_Pos                   4U                                            /*!< SCB NSACR: CP4 Position */
#define SCB_NSACR_CP4_Msk                  (1UL << SCB_NSACR_CP4_Pos)                     /*!< SCB NSACR: CP4 Mask */

#define SCB_NSACR_CP3_Pos                   3U                                            /*!< SCB NSACR: CP3 Position */
#define SCB_NSACR_CP3_Msk                  (1UL << SCB_NSACR_CP3_Pos)                     /*!< SCB NSACR: CP3 Mask */

#define SCB_NSACR_CP2_Pos                   2U                                            /*!< SCB NSACR: CP2 Position */
#define SCB_NSACR_CP2_Msk                  (1UL << SCB_NSACR_CP2_Pos)                     /*!< SCB NSACR: CP2 Mask */

#define SCB_NSACR_CP1_Pos                   1U                                            /*!< SCB NSACR: CP1 Position */
#define SCB_NSACR_CP1_Msk                  (1UL << SCB_NSACR_CP1_Pos)                     /*!< SCB NSACR: CP1 Mask */

#define SCB_NSACR_CP0_Pos                   0U                                            /*!< SCB NSACR: CP0 Position */
#define SCB_NSACR_CP0_Msk                  (1UL /*<< SCB_NSACR_CP0_Pos*/)                 /*!< SCB NSACR: CP0 Mask */

/* SCB Debug Feature Register 0 Definitions */
#define SCB_ID_DFR_UDE_Pos                 28U                                            /*!< SCB ID_DFR: UDE Position */
#define SCB_ID_DFR_UDE_Msk                 (0xFUL << SCB_ID_DFR_UDE_Pos)                  /*!< SCB ID_DFR: UDE Mask */

#define SCB_ID_DFR_MProfDbg_Pos            20U                                            /*!< SCB ID_DFR: MProfDbg Position */
#define SCB_ID_DFR_MProfDbg_Msk            (0xFUL << SCB_ID_DFR_MProfDbg_Pos)             /*!< SCB ID_DFR: MProfDbg Mask */

/* SCB Cache Level ID Register Definitions */
#define SCB_CLIDR_LOUU_Pos                 27U                                            /*!< SCB CLIDR: LoUU Position */
#define SCB_CLIDR_LOUU_Msk                 (7UL << SCB_CLIDR_LOUU_Pos)                    /*!< SCB CLIDR: LoUU Mask */

#define SCB_CLIDR_LOC_Pos                  24U                                            /*!< SCB CLIDR: LoC Position */
#define SCB_CLIDR_LOC_Msk                  (7UL << SCB_CLIDR_LOC_Pos)                     /*!< SCB CLIDR: LoC Mask */

/* SCB Cache Type Register Definitions */
#define SCB_CTR_FORMAT_Pos                 29U                                            /*!< SCB CTR: Format Position */
#define SCB_CTR_FORMAT_Msk                 (7UL << SCB_CTR_FORMAT_Pos)                    /*!< SCB CTR: Format Mask */

#define SCB_CTR_CWG_Pos                    24U                                            /*!< SCB CTR: CWG Position */
#define SCB_CTR_CWG_Msk                    (0xFUL << SCB_CTR_CWG_Pos)                     /*!< SCB CTR: CWG Mask */

#define SCB_CTR_ERG_Pos                    20U                                            /*!< SCB CTR: ERG Position */
#define SCB_CTR_ERG_Msk                    (0xFUL << SCB_CTR_ERG_Pos)                     /*!< SCB CTR: ERG Mask */

#define SCB_CTR_DMINLINE_Pos               16U                                            /*!< SCB CTR: DminLine Position */
#define SCB_CTR_DMINLINE_Msk               (0xFUL << SCB_CTR_DMINLINE_Pos)                /*!< SCB CTR: DminLine Mask */

#define SCB_CTR_IMINLINE_Pos                0U                                            /*!< SCB CTR: ImInLine Position */
#define SCB_CTR_IMINLINE_Msk               (0xFUL /*<< SCB_CTR_IMINLINE_Pos*/)            /*!< SCB CTR: ImInLine Mask */

/* SCB Cache Size ID Register Definitions */
#define SCB_CCSIDR_WT_Pos                  31U                                            /*!< SCB CCSIDR: WT Position */
#define SCB_CCSIDR_WT_Msk                  (1UL << SCB_CCSIDR_WT_Pos)                     /*!< SCB CCSIDR: WT Mask */

#define SCB_CCSIDR_WB_Pos                  30U                                            /*!< SCB CCSIDR: WB Position */
#define SCB_CCSIDR_WB_Msk                  (1UL << SCB_CCSIDR_WB_Pos)                     /*!< SCB CCSIDR: WB Mask */

#define SCB_CCSIDR_RA_Pos                  29U                                            /*!< SCB CCSIDR: RA Position */
#define SCB_CCSIDR_RA_Msk                  (1UL << SCB_CCSIDR_RA_Pos)                     /*!< SCB CCSIDR: RA Mask */

#define SCB_CCSIDR_WA_Pos                  28U                                            /*!< SCB CCSIDR: WA Position */
#define SCB_CCSIDR_WA_Msk                  (1UL << SCB_CCSIDR_WA_Pos)                     /*!< SCB CCSIDR: WA Mask */

#define SCB_CCSIDR_NUMSETS_Pos             13U                                            /*!< SCB CCSIDR: NumSets Position */
#define SCB_CCSIDR_NUMSETS_Msk             (0x7FFFUL << SCB_CCSIDR_NUMSETS_Pos)           /*!< SCB CCSIDR: NumSets Mask */

#define SCB_CCSIDR_ASSOCIATIVITY_Pos        3U                                            /*!< SCB CCSIDR: Associativity Position */
#define SCB_CCSIDR_ASSOCIATIVITY_Msk       (0x3FFUL << SCB_CCSIDR_ASSOCIATIVITY_Pos)      /*!< SCB CCSIDR: Associativity Mask */

#define SCB_CCSIDR_LINESIZE_Pos             0U                                            /*!< SCB CCSIDR: LineSize Position */
#define SCB_CCSIDR_LINESIZE_Msk            (7UL /*<< SCB_CCSIDR_LINESIZE_Pos*/)           /*!< SCB CCSIDR: LineSize Mask */

/* SCB Cache Size Selection Register Definitions */
#define SCB_CSSELR_LEVEL_Pos                1U                                            /*!< SCB CSSELR: Level Position */
#define SCB_CSSELR_LEVEL_Msk               (7UL << SCB_CSSELR_LEVEL_Pos)                  /*!< SCB CSSELR: Level Mask */

#define SCB_CSSELR_IND_Pos                  0U                                            /*!< SCB CSSELR: InD Position */
#define SCB_CSSELR_IND_Msk                 (1UL /*<< SCB_CSSELR_IND_Pos*/)                /*!< SCB CSSELR: InD Mask */

/* SCB Software Triggered Interrupt Register Definitions */
#define SCB_STIR_INTID_Pos                  0U                                            /*!< SCB STIR: INTID Position */
#define SCB_STIR_INTID_Msk                 (0x1FFUL /*<< SCB_STIR_INTID_Pos*/)            /*!< SCB STIR: INTID Mask */

/* SCB RAS Fault Status Register Definitions */
#define SCB_RFSR_V_Pos                     31U                                            /*!< SCB RFSR: V Position */
#define SCB_RFSR_V_Msk                     (1UL << SCB_RFSR_V_Pos)                        /*!< SCB RFSR: V Mask */

#define SCB_RFSR_IS_Pos                    16U                                            /*!< SCB RFSR: IS Position */
#define SCB_RFSR_IS_Msk                    (0x7FFFUL << SCB_RFSR_IS_Pos)                  /*!< SCB RFSR: IS Mask */

#define SCB_RFSR_UET_Pos                    0U                                            /*!< SCB RFSR: UET Position */
#define SCB_RFSR_UET_Msk                   (3UL /*<< SCB_RFSR_UET_Pos*/)                  /*!< SCB RFSR: UET Mask */

/* SCB D-Cache Invalidate by Set-way Register Definitions */
#define SCB_DCISW_WAY_Pos                  30U                                            /*!< SCB DCISW: Way Position */
#define SCB_DCISW_WAY_Msk                  (3UL << SCB_DCISW_WAY_Pos)                     /*!< SCB DCISW: Way Mask */

#define SCB_DCISW_SET_Pos                   5U                                            /*!< SCB DCISW: Set Position */
#define SCB_DCISW_SET_Msk                  (0x1FFUL << SCB_DCISW_SET_Pos)                 /*!< SCB DCISW: Set Mask */

/* SCB D-Cache Clean by Set-way Register Definitions */
#define SCB_DCCSW_WAY_Pos                  30U                                            /*!< SCB DCCSW: Way Position */
#define SCB_DCCSW_WAY_Msk                  (3UL << SCB_DCCSW_WAY_Pos)                     /*!< SCB DCCSW: Way Mask */

#define SCB_DCCSW_SET_Pos                   5U                                            /*!< SCB DCCSW: Set Position */
#define SCB_DCCSW_SET_Msk                  (0x1FFUL << SCB_DCCSW_SET_Pos)                 /*!< SCB DCCSW: Set Mask */

/* SCB D-Cache Clean and Invalidate by Set-way Register Definitions */
#define SCB_DCCISW_WAY_Pos                 30U                                            /*!< SCB DCCISW: Way Position */
#define SCB_DCCISW_WAY_Msk                 (3UL << SCB_DCCISW_WAY_Pos)                    /*!< SCB DCCISW: Way Mask */

#define SCB_DCCISW_SET_Pos                  5U                                            /*!< SCB DCCISW: Set Position */
#define SCB_DCCISW_SET_Msk                 (0x1FFUL << SCB_DCCISW_SET_Pos)                /*!< SCB DCCISW: Set Mask */

/*@} end of group CMSIS_SCB */


/**
  \ingroup  CMSIS_core_register
  \defgroup CMSIS_ICB Implementation Control Block register (ICB)
  \brief    Type definitions for the Implementation Control Block Register
  @{
 */

/**
  \brief  Structure type to access the Implementation Control Block (ICB).
 */
typedef struct
{
        uint32_t RESERVED0[1U];
  __IM  uint32_t ICTR;                   /*!< Offset: 0x004 (R/ )  Interrupt Controller Type Register */
  __IOM uint32_t ACTLR;                  /*!< Offset: 0x008 (R/W)  Auxiliary Control Register */
  __IOM uint32_t CPPWR;                  /*!< Offset: 0x00C (R/W)  Coprocessor Power Control  Register */
} ICB_Type;

/* Auxiliary Control Register Definitions */
#define ICB_ACTLR_DISCRITAXIRUW_Pos     27U                                               /*!< ACTLR: DISCRITAXIRUW Position */
#define ICB_ACTLR_DISCRITAXIRUW_Msk     (1UL << ICB_ACTLR_DISCRITAXIRUW_Pos)              /*!< ACTLR: DISCRITAXIRUW Mask */

#define ICB_ACTLR_DISDI_Pos             16U                                               /*!< ACTLR: DISDI Position */
#define ICB_ACTLR_DISDI_Msk             (3UL << ICB_ACTLR_DISDI_Pos)                      /*!< ACTLR: DISDI Mask */

#define ICB_ACTLR_DISCRITAXIRUR_Pos     15U                                               /*!< ACTLR: DISCRITAXIRUR Position */
#define ICB_ACTLR_DISCRITAXIRUR_Msk     (1UL << ICB_ACTLR_DISCRITAXIRUR_Pos)              /*!< ACTLR: DISCRITAXIRUR Mask */

#define ICB_ACTLR_EVENTBUSEN_Pos        14U                                               /*!< ACTLR: EVENTBUSEN Position */
#define ICB_ACTLR_EVENTBUSEN_Msk        (1UL << ICB_ACTLR_EVENTBUSEN_Pos)                 /*!< ACTLR: EVENTBUSEN Mask */

#define ICB_ACTLR_EVENTBUSEN_S_Pos      13U                                               /*!< ACTLR: EVENTBUSEN_S Position */
#define ICB_ACTLR_EVENTBUSEN_S_Msk      (1UL << ICB_ACTLR_EVENTBUSEN_S_Pos)               /*!< ACTLR: EVENTBUSEN_S Mask */

#define ICB_ACTLR_DISITMATBFLUSH_Pos    12U                                               /*!< ACTLR: DISITMATBFLUSH Position */
#define ICB_ACTLR_DISITMATBFLUSH_Msk    (1UL << ICB_ACTLR_DISITMATBFLUSH_Pos)             /*!< ACTLR: DISITMATBFLUSH Mask */

#define ICB_ACTLR_DISNWAMODE_Pos        11U                                               /*!< ACTLR: DISNWAMODE Position */
#define ICB_ACTLR_DISNWAMODE_Msk        (1UL << ICB_ACTLR_DISNWAMODE_Pos)                 /*!< ACTLR: DISNWAMODE Mask */

#define ICB_ACTLR_FPEXCODIS_Pos         10U                                               /*!< ACTLR: FPEXCODIS Position */
#define ICB_ACTLR_FPEXCODIS_Msk         (1UL << ICB_ACTLR_FPEXCODIS_Pos)                  /*!< ACTLR: FPEXCODIS Mask */

#define ICB_ACTLR_DISOLAP_Pos            7U                                               /*!< ACTLR: DISOLAP Position */
#define ICB_ACTLR_DISOLAP_Msk           (1UL << ICB_ACTLR_DISOLAP_Pos)                    /*!< ACTLR: DISOLAP Mask */

#define ICB_ACTLR_DISOLAPS_Pos           6U                                               /*!< ACTLR: DISOLAPS Position */
#define ICB_ACTLR_DISOLAPS_Msk          (1UL << ICB_ACTLR_DISOLAPS_Pos)                   /*!< ACTLR: DISOLAPS Mask */

#define ICB_ACTLR_DISLOBR_Pos            5U                                               /*!< ACTLR: DISLOBR Position */
#define ICB_ACTLR_DISLOBR_Msk           (1UL << ICB_ACTLR_DISLOBR_Pos)                    /*!< ACTLR: DISLOBR Mask */

#define ICB_ACTLR_DISLO_Pos              4U                                               /*!< ACTLR: DISLO Position */
#define ICB_ACTLR_DISLO_Msk             (1UL << ICB_ACTLR_DISLO_Pos)                      /*!< ACTLR: DISLO Mask */

#define ICB_ACTLR_DISLOLEP_Pos           3U                                               /*!< ACTLR: DISLOLEP Position */
#define ICB_ACTLR_DISLOLEP_Msk          (1UL << ICB_ACTLR_DISLOLEP_Pos)                   /*!< ACTLR: DISLOLEP Mask */

#define ICB_ACTLR_DISFOLD_Pos            2U                                               /*!< ACTLR: DISFOLD Position */
#define ICB_ACTLR_DISFOLD_Msk           (1UL << ICB_ACTLR_DISFOLD_Pos)                    /*!< ACTLR: DISFOLD Mask */

/* Interrupt Controller Type Register Definitions */
#define ICB_ICTR_INTLINESNUM_Pos         0U                                               /*!< ICTR: INTLINESNUM Position */
#define ICB_ICTR_INTLINESNUM_Msk        (0xFUL /*<< ICB_ICTR_INTLINESNUM_Pos*/)           /*!< ICTR: INTLINESNUM Mask */

/*@} end of group CMSIS_ICB */


/**
  \ingroup  CMSIS_core_register
  \defgroup CMSIS_SysTick     System Tick Timer (SysTick)
  \brief    Type definitions for the System Timer Registers.
  @{
 */

/**
  \brief  Structure type to access the System Timer (SysTick).
 */
typedef struct
{
  __IOM uint32_t CTRL;                   /*!< Offset: 0x000 (R/W)  SysTick Control and Status Register */
  __IOM uint32_t LOAD;                   /*!< Offset: 0x004 (R/W)  SysTick Reload Value Register */
  __IOM uint32_t VAL;                    /*!< Offset: 0x008 (R/W)  SysTick Current Value Register */
  __IM  uint32_t CALIB;                  /*!< Offset: 0x00C (R/ )  SysTick Calibration Register */
} SysTick_Type;

/* SysTick Control / Status Register Definitions */
#define SysTick_CTRL_COUNTFLAG_Pos         16U                                            /*!< SysTick CTRL: COUNTFLAG Position */
#define SysTick_CTRL_COUNTFLAG_Msk         (1UL << SysTick_CTRL_COUNTFLAG_Pos)            /*!< SysTick CTRL: COUNTFLAG Mask */

#define SysTick_CTRL_CLKSOURCE_Pos          2U                                            /*!< SysTick CTRL: CLKSOURCE Position */
#define SysTick_CTRL_CLKSOURCE_Msk         (1UL << SysTick_CTRL_CLKSOURCE_Pos)            /*!< SysTick CTRL: CLKSOURCE Mask */

#define SysTick_CTRL_TICKINT_Pos            1U                                            /*!< SysTick CTRL: TICKINT Position */
#define SysTick_CTRL_TICKINT_Msk           (1UL << SysTick_CTRL_TICKINT_Pos)              /*!< SysTick CTRL: TICKINT Mask */

#define SysTick_CTRL_ENABLE_Pos             0U                                            /*!< SysTick CTRL: ENABLE Position */
#define SysTick_CTRL_ENABLE_Msk            (1UL /*<< SysTick_CTRL_ENABLE_Pos*/)           /*!< SysTick CTRL: ENABLE Mask */

/* SysTick Reload Register Definitions */
#define SysTick_LOAD_RELOAD_Pos             0U                                            /*!< SysTick LOAD: RELOAD Position */
#define SysTick_LOAD_RELOAD_Msk            (0xFFFFFFUL /*<< SysTick_LOAD_RELOAD_Pos*/)    /*!< SysTick LOAD: RELOAD Mask */

/* SysTick Current Register Definitions */
#define SysTick_VAL_CURRENT_Pos             0U                                            /*!< SysTick VAL: CURRENT Position */
#define SysTick_VAL_CURRENT_Msk            (0xFFFFFFUL /*<< SysTick_VAL_CURRENT_Pos*/)    /*!< SysTick VAL: CURRENT Mask */

/* SysTick Calibration Register Definitions */
#define SysTick_CALIB_NOREF_Pos            31U                                            /*!< SysTick CALIB: NOREF Position */
#define SysTick_CALIB_NOREF_Msk            (1UL << SysTick_CALIB_NOREF_Pos)               /*!< SysTick CALIB: NOREF Mask */

#define SysTick_CALIB_SKEW_Pos             30U                                            /*!< SysTick CALIB: SKEW Position */
#define SysTick_CALIB_SKEW_Msk             (1UL << SysTick_CALIB_SKEW_Pos)                /*!< SysTick CALIB: SKEW Mask */

#define SysTick_CALIB_TENMS_Pos             0U                                            /*!< SysTick CALIB: TENMS Position */
#define SysTick_CALIB_TENMS_Msk            (0xFFFFFFUL /*<< SysTick_CALIB_TENMS_Pos*/)    /*!< SysTick CALIB: TENMS Mask */

/*@} end of group CMSIS_SysTick */


/**
  \ingroup  CMSIS_core_register
  \defgroup CMSIS_ITM     Instrumentation Trace Macrocell (ITM)
  \brief    Type definitions for the Instrumentation Trace Macrocell (ITM)
  @{
 */

/**
  \brief  Structure type to access the Instrumentation Trace Macrocell Register (ITM).
 */
typedef struct
{
  __OM  union
  {
    __OM  uint8_t    u8;                 /*!< Offset: 0x000 ( /W)  ITM Stimulus Port 8-bit */
    __OM  uint16_t   u16;                /*!< Offset: 0x000 ( /W)  ITM Stimulus Port 16-bit */
    __OM  uint32_t   u32;                /*!< Offset: 0x000 ( /W)  ITM Stimulus Port 32-bit */
  }  PORT [32U];                         /*!< Offset: 0x000 ( /W)  ITM Stimulus Port Registers */
        uint32_t RESERVED0[864U];
  __IOM uint32_t TER;                    /*!< Offset: 0xE00 (R/W)  ITM Trace Enable Register */
        uint32_t RESERVED1[15U];
  __IOM uint32_t TPR;                    /*!< Offset: 0xE40 (R/W)  ITM Trace Privilege Register */
        uint32_t RESERVED2[15U];
  __IOM uint32_t TCR;                    /*!< Offset: 0xE80 (R/W)  ITM Trace Control Register */
        uint32_t RESERVED3[32U];
        uint32_t RESERVED4[43U];
  __OM  uint32_t LAR;                    /*!< Offset: 0xFB0 ( /W)  ITM Lock Access Register */
  __IM  uint32_t LSR;                    /*!< Offset: 0xFB4 (R/ )  ITM Lock Status Register */
        uint32_t RESERVED5[1U];
  __IM  uint32_t DEVARCH;                /*!< Offset: 0xFBC (R/ )  ITM Device Architecture Register */
        uint32_t RESERVED6[3U];
  __IM  uint32_t DEVTYPE;                /*!< Offset: 0xFCC (R/ )  ITM Device Type Register */
  __IM  uint32_t PID4;                   /*!< Offset: 0xFD0 (R/ )  ITM Peripheral Identification Register #4 */
  __IM  uint32_t PID5;                   /*!< Offset: 0xFD4 (R/ )  ITM Peripheral Identification Register #5 */
  __IM  uint32_t PID6;                   /*!< Offset: 0xFD8 (R/ )  ITM Peripheral Identification Register #6 */
  __IM  uint32_t PID7;                   /*!< Offset: 0xFDC (R/ )  ITM Peripheral Identification Register #7 */
  __IM  uint32_t PID0;                   /*!< Offset: 0xFE0 (R/ )  ITM Peripheral Identification Register #0 */
  __IM  uint32_t PID1;                   /*!< Offset: 0xFE4 (R/ )  ITM Peripheral Identification Register #1 */
  __IM  uint32_t PID2;                   /*!< Offset: 0xFE8 (R/ )  ITM Peripheral Identification Register #2 */
  __IM  uint32_t PID3;                   /*!< Offset: 0xFEC (R/ )  ITM Peripheral Identification Register #3 */
  __IM  uint32_t CID0;                   /*!< Offset: 0xFF0 (R/ )  ITM Component  Identification Register #0 */
  __IM  uint32_t CID1;                   /*!< Offset: 0xFF4 (R/ )  ITM Component  Identification Register #1 */
  __IM  uint32_t CID2;                   /*!< Offset: 0xFF8 (R/ )  ITM Component  Identification Register #2 */
  __IM  uint32_t CID3;                   /*!< Offset: 0xFFC (R/ )  ITM Component  Identification Register #3 */
} ITM_Type;

/* ITM Stimulus Port Register Definitions */
#define ITM_STIM_DISABLED_Pos               1U                                            /*!< ITM STIM: DISABLED Position */
#define ITM_STIM_DISABLED_Msk              (0x1UL << ITM_STIM_DISABLED_Pos)               /*!< ITM STIM: DISABLED Mask */

#define ITM_STIM_FIFOREADY_Pos              0U                                            /*!< ITM STIM: FIFOREADY Position */
#define ITM_STIM_FIFOREADY_Msk             (0x1UL /*<< ITM_STIM_FIFOREADY_Pos*/)          /*!< ITM STIM: FIFOREADY Mask */

/* ITM Trace Privilege Register Definitions */
#define ITM_TPR_PRIVMASK_Pos                0U                                            /*!< ITM TPR: PRIVMASK Position */
#define ITM_TPR_PRIVMASK_Msk               (0xFUL /*<< ITM_TPR_PRIVMASK_Pos*/)            /*!< ITM TPR: PRIVMASK Mask */

/* ITM Trace Control Register Definitions */
#define ITM_TCR_BUSY_Pos                   23U                                            /*!< ITM TCR: BUSY Position */
#define ITM_TCR_BUSY_Msk                   (1UL << ITM_TCR_BUSY_Pos)                      /*!< ITM TCR: BUSY Mask */

#define ITM_TCR_TRACEBUSID_Pos             16U                                            /*!< ITM TCR: ATBID Position */
#define ITM_TCR_TRACEBUSID_Msk             (0x7FUL << ITM_TCR_TRACEBUSID_Pos)             /*!< ITM TCR: ATBID Mask */

#define ITM_TCR_GTSFREQ_Pos                10U                                            /*!< ITM TCR: Global timestamp frequency Position */
#define ITM_TCR_GTSFREQ_Msk                (3UL << ITM_TCR_GTSFREQ_Pos)                   /*!< ITM TCR: Global timestamp frequency Mask */

#define ITM_TCR_TSPRESCALE_Pos              8U                                            /*!< ITM TCR: TSPRESCALE Position */
#define ITM_TCR_TSPRESCALE_Msk             (3UL << ITM_TCR_TSPRESCALE_Pos)                /*!< ITM TCR: TSPRESCALE Mask */

#define ITM_TCR_STALLENA_Pos                5U                                            /*!< ITM TCR: STALLENA Position */
#define ITM_TCR_STALLENA_Msk               (1UL << ITM_TCR_STALLENA_Pos)                  /*!< ITM TCR: STALLENA Mask */

#define ITM_TCR_SWOENA_Pos                  4U                                            /*!< ITM TCR: SWOENA Position */
#define ITM_TCR_SWOENA_Msk                 (1UL << ITM_TCR_SWOENA_Pos)                    /*!< ITM TCR: SWOENA Mask */

#define ITM_TCR_DWTENA_Pos                  3U                                            /*!< ITM TCR: DWTENA Position */
#define ITM_TCR_DWTENA_Msk                 (1UL << ITM_TCR_DWTENA_Pos)                    /*!< ITM TCR: DWTENA Mask */

#define ITM_TCR_SYNCENA_Pos                 2U                                            /*!< ITM TCR: SYNCENA Position */
#define ITM_TCR_SYNCENA_Msk                (1UL << ITM_TCR_SYNCENA_Pos)                   /*!< ITM TCR: SYNCENA Mask */

#define ITM_TCR_TSENA_Pos                   1U                                            /*!< ITM TCR: TSENA Position */
#define ITM_TCR_TSENA_Msk                  (1UL << ITM_TCR_TSENA_Pos)                     /*!< ITM TCR: TSENA Mask */

#define ITM_TCR_ITMENA_Pos                  0U                                            /*!< ITM TCR: ITM Enable bit Position */
#define ITM_TCR_ITMENA_Msk                 (1UL /*<< ITM_TCR_ITMENA_Pos*/)                /*!< ITM TCR: ITM Enable bit Mask */

/* ITM Lock Status Register Definitions */
#define ITM_LSR_ByteAcc_Pos                 2U                                            /*!< ITM LSR: ByteAcc Position */
#define ITM_LSR_ByteAcc_Msk                (1UL << ITM_LSR_ByteAcc_Pos)                   /*!< ITM LSR: ByteAcc Mask */

#define ITM_LSR_Access_Pos                  1U                                            /*!< ITM LSR: Access Position */
#define ITM_LSR_Access_Msk                 (1UL << ITM_LSR_Access_Pos)                    /*!< ITM LSR: Access Mask */

#define ITM_LSR_Present_Pos                 0U                                            /*!< ITM LSR: Present Position */
#define ITM_LSR_Present_Msk                (1UL /*<< ITM_LSR_Present_Pos*/)               /*!< ITM LSR: Present Mask */

/*@}*/ /* end of group CMSIS_ITM */


/**
  \ingroup  CMSIS_core_register
  \defgroup CMSIS_DWT     Data Watchpoint and Trace (DWT)
  \brief    Type definitions for the Data Watchpoint and Trace (DWT)
  @{
 */

/**
  \brief  Structure type to access the Data Watchpoint and Trace Register (DWT).
 */
typedef struct
{
  __IOM uint32_t CTRL;                   /*!< Offset: 0x000 (R/W)  Control Register */
  __IOM uint32_t CYCCNT;                 /*!< Offset: 0x004 (R/W)  Cycle Count Register */
  __IOM uint32_t CPICNT;                 /*!< Offset: 0x008 (R/W)  CPI Count Register */
  __IOM uint32_t EXCCNT;                 /*!< Offset: 0x00C (R/W)  Exception Overhead Count Register */
  __IOM uint32_t SLEEPCNT;               /*!< Offset: 0x010 (R/W)  Sleep Count Register */
  __IOM uint32_t LSUCNT;                 /*!< Offset: 0x014 (R/W)  LSU Count Register */
  __IOM uint32_t FOLDCNT;                /*!< Offset: 0x018 (R/W)  Folded-instruction Count Register */
  __IM  uint32_t PCSR;                   /*!< Offset: 0x01C (R/ )  Program Counter Sample Register */
  __IOM uint32_t COMP0;                  /*!< Offset: 0x020 (R/W)  Comparator Register 0 */
        uint32_t RESERVED1[1U];
  __IOM uint32_t FUNCTION0;              /*!< Offset: 0x028 (R/W)  Function Register 0 */
        uint32_t RESERVED2[1U];
  __IOM uint32_t COMP1;                  /*!< Offset: 0x030 (R/W)  Comparator Register 1 */
        uint32_t RESERVED3[1U];
  __IOM uint32_t FUNCTION1;              /*!< Offset: 0x038 (R/W)  Function Register 1 */
        uint32_t RESERVED4[1U];
  __IOM uint32_t COMP2;                  /*!< Offset: 0x040 (R/W)  Comparator Register 2 */
        uint32_t RESERVED5[1U];
  __IOM uint32_t FUNCTION2;              /*!< Offset: 0x048 (R/W)  Function Register 2 */
        uint32_t RESERVED6[1U];
  __IOM uint32_t COMP3;                  /*!< Offset: 0x050 (R/W)  Comparator Register 3 */
        uint32_t RESERVED7[1U];
  __IOM uint32_t FUNCTION3;              /*!< Offset: 0x058 (R/W)  Function Register 3 */
        uint32_t RESERVED8[1U];
  __IOM uint32_t COMP4;                  /*!< Offset: 0x060 (R/W)  Comparator Register 4 */
        uint32_t RESERVED9[1U];
  __IOM uint32_t FUNCTION4;              /*!< Offset: 0x068 (R/W)  Function Register 4 */
        uint32_t RESERVED10[1U];
  __IOM uint32_t COMP5;                  /*!< Offset: 0x070 (R/W)  Comparator Register 5 */
        uint32_t RESERVED11[1U];
  __IOM uint32_t FUNCTION5;              /*!< Offset: 0x078 (R/W)  Function Register 5 */
        uint32_t RESERVED12[1U];
  __IOM uint32_t COMP6;                  /*!< Offset: 0x080 (R/W)  Comparator Register 6 */
        uint32_t RESERVED13[1U];
  __IOM uint32_t FUNCTION6;              /*!< Offset: 0x088 (R/W)  Function Register 6 */
        uint32_t RESERVED14[1U];
  __IOM uint32_t COMP7;                  /*!< Offset: 0x090 (R/W)  Comparator Register 7 */
        uint32_t RESERVED15[1U];
  __IOM uint32_t FUNCTION7;              /*!< Offset: 0x098 (R/W)  Function Register 7 */
        uint32_t RESERVED16[1U];
  __IOM uint32_t COMP8;                  /*!< Offset: 0x0A0 (R/W)  Comparator Register 8 */
        uint32_t RESERVED17[1U];
  __IOM uint32_t FUNCTION8;              /*!< Offset: 0x0A8 (R/W)  Function Register 8 */
        uint32_t RESERVED18[1U];
  __IOM uint32_t COMP9;                  /*!< Offset: 0x0B0 (R/W)  Comparator Register 9 */
        uint32_t RESERVED19[1U];
  __IOM uint32_t FUNCTION9;              /*!< Offset: 0x0B8 (R/W)  Function Register 9 */
        uint32_t RESERVED20[1U];
  __IOM uint32_t COMP10;                 /*!< Offset: 0x0C0 (R/W)  Comparator Register 10 */
        uint32_t RESERVED21[1U];
  __IOM uint32_t FUNCTION10;             /*!< Offset: 0x0C8 (R/W)  Function Register 10 */
        uint32_t RESERVED22[1U];
  __IOM uint32_t COMP11;                 /*!< Offset: 0x0D0 (R/W)  Comparator Register 11 */
        uint32_t RESERVED23[1U];
  __IOM uint32_t FUNCTION11;             /*!< Offset: 0x0D8 (R/W)  Function Register 11 */
        uint32_t RESERVED24[1U];
  __IOM uint32_t COMP12;                 /*!< Offset: 0x0E0 (R/W)  Comparator Register 12 */
        uint32_t RESERVED25[1U];
  __IOM uint32_t FUNCTION12;             /*!< Offset: 0x0E8 (R/W)  Function Register 12 */
        uint32_t RESERVED26[1U];
  __IOM uint32_t COMP13;                 /*!< Offset: 0x0F0 (R/W)  Comparator Register 13 */
        uint32_t RESERVED27[1U];
  __IOM uint32_t FUNCTION13;             /*!< Offset: 0x0F8 (R/W)  Function Register 13 */
        uint32_t RESERVED28[1U];
  __IOM uint32_t COMP14;                 /*!< Offset: 0x100 (R/W)  Comparator Register 14 */
        uint32_t RESERVED29[1U];
  __IOM uint32_t FUNCTION14;             /*!< Offset: 0x108 (R/W)  Function Register 14 */
        uint32_t RESERVED30[1U];
  __IOM uint32_t COMP15;                 /*!< Offset: 0x110 (R/W)  Comparator Register 15 */
        uint32_t RESERVED31[1U];
  __IOM uint32_t FUNCTION15;             /*!< Offset: 0x118 (R/W)  Function Register 15 */
        uint32_t RESERVED32[934U];
  __IM  uint32_t LSR;                    /*!< Offset: 0xFB4 (R  )  Lock Status Register */
        uint32_t RESERVED33[1U];
  __IM  uint32_t DEVARCH;                /*!< Offset: 0xFBC (R/ )  Device Architecture Register */
} DWT_Type;

/* DWT Control Register Definitions */
#define DWT_CTRL_NUMCOMP_Pos               28U                                         /*!< DWT CTRL: NUMCOMP Position */
#define DWT_CTRL_NUMCOMP_Msk               (0xFUL << DWT_CTRL_NUMCOMP_Pos)             /*!< DWT CTRL: NUMCOMP Mask */

#define DWT_CTRL_NOTRCPKT_Pos              27U                                         /*!< DWT CTRL: NOTRCPKT Position */
#define DWT_CTRL_NOTRCPKT_Msk              (0x1UL << DWT_CTRL_NOTRCPKT_Pos)            /*!< DWT CTRL: NOTRCPKT Mask */

#define DWT_CTRL_NOEXTTRIG_Pos             26U                                         /*!< DWT CTRL: NOEXTTRIG Position */
#define DWT_CTRL_NOEXTTRIG_Msk             (0x1UL << DWT_CTRL_NOEXTTRIG_Pos)           /*!< DWT CTRL: NOEXTTRIG Mask */

#define DWT_CTRL_NOCYCCNT_Pos              25U                                         /*!< DWT CTRL: NOCYCCNT Position */
#define DWT_CTRL_NOCYCCNT_Msk              (0x1UL << DWT_CTRL_NOCYCCNT_Pos)            /*!< DWT CTRL: NOCYCCNT Mask */

#define DWT_CTRL_NOPRFCNT_Pos              24U                                         /*!< DWT CTRL: NOPRFCNT Position */
#define DWT_CTRL_NOPRFCNT_Msk              (0x1UL << DWT_CTRL_NOPRFCNT_Pos)            /*!< DWT CTRL: NOPRFCNT Mask */

#define DWT_CTRL_CYCDISS_Pos               23U                                         /*!< DWT CTRL: CYCDISS Position */
#define DWT_CTRL_CYCDISS_Msk               (0x1UL << DWT_CTRL_CYCDISS_Pos)             /*!< DWT CTRL: CYCDISS Mask */

#define DWT_CTRL_CYCEVTENA_Pos             22U                                         /*!< DWT CTRL: CYCEVTENA Position */
#define DWT_CTRL_CYCEVTENA_Msk             (0x1UL << DWT_CTRL_CYCEVTENA_Pos)           /*!< DWT CTRL: CYCEVTENA Mask */

#define DWT_CTRL_FOLDEVTENA_Pos            21U                                         /*!< DWT CTRL: FOLDEVTENA Position */
#define DWT_CTRL_FOLDEVTENA_Msk            (0x1UL << DWT_CTRL_FOLDEVTENA_Pos)          /*!< DWT CTRL: FOLDEVTENA Mask */

#define DWT_CTRL_LSUEVTENA_Pos             20U                                         /*!< DWT CTRL: LSUEVTENA Position */
#define DWT_CTRL_LSUEVTENA_Msk             (0x1UL << DWT_CTRL_LSUEVTENA_Pos)           /*!< DWT CTRL: LSUEVTENA Mask */

#define DWT_CTRL_SLEEPEVTENA_Pos           19U                                         /*!< DWT CTRL: SLEEPEVTENA Position */
#define DWT_CTRL_SLEEPEVTENA_Msk           (0x1UL << DWT_CTRL_SLEEPEVTENA_Pos)         /*!< DWT CTRL: SLEEPEVTENA Mask */

#define DWT_CTRL_EXCEVTENA_Pos             18U                                         /*!< DWT CTRL: EXCEVTENA Position */
#define DWT_CTRL_EXCEVTENA_Msk             (0x1UL << DWT_CTRL_EXCEVTENA_Pos)           /*!< DWT CTRL: EXCEVTENA Mask */

#define DWT_CTRL_CPIEVTENA_Pos             17U                                         /*!< DWT CTRL: CPIEVTENA Position */
#define DWT_CTRL_CPIEVTENA_Msk             (0x1UL << DWT_CTRL_CPIEVTENA_Pos)           /*!< DWT CTRL: CPIEVTENA Mask */

#define DWT_CTRL_EXCTRCENA_Pos             16U                                         /*!< DWT CTRL: EXCTRCENA Position */
#define DWT_CTRL_EXCTRCENA_Msk             (0x1UL << DWT_CTRL_EXCTRCENA_Pos)           /*!< DWT CTRL: EXCTRCENA Mask */

#define DWT_CTRL_PCSAMPLENA_Pos            12U                                         /*!< DWT CTRL: PCSAMPLENA Position */
#define DWT_CTRL_PCSAMPLENA_Msk            (0x1UL << DWT_CTRL_PCSAMPLENA_Pos)          /*!< DWT CTRL: PCSAMPLENA Mask */

#define DWT_CTRL_SYNCTAP_Pos               10U                                         /*!< DWT CTRL: SYNCTAP Position */
#define DWT_CTRL_SYNCTAP_Msk               (0x3UL << DWT_CTRL_SYNCTAP_Pos)             /*!< DWT CTRL: SYNCTAP Mask */

#define DWT_CTRL_CYCTAP_Pos                 9U                                         /*!< DWT CTRL: CYCTAP Position */
#define DWT_CTRL_CYCTAP_Msk                (0x1UL << DWT_CTRL_CYCTAP_Pos)              /*!< DWT CTRL: CYCTAP Mask */

#define DWT_CTRL_POSTINIT_Pos               5U                                         /*!< DWT CTRL: POSTINIT Position */
#define DWT_CTRL_POSTINIT_Msk              (0xFUL << DWT_CTRL_POSTINIT_Pos)            /*!< DWT CTRL: POSTINIT Mask */

#define DWT_CTRL_POSTPRESET_Pos             1U                                         /*!< DWT CTRL: POSTPRESET Position */
#define DWT_CTRL_POSTPRESET_Msk            (0xFUL << DWT_CTRL_POSTPRESET_Pos)          /*!< DWT CTRL: POSTPRESET Mask */

#define DWT_CTRL_CYCCNTENA_Pos              0U                                         /*!< DWT CTRL: CYCCNTENA Position */
#define DWT_CTRL_CYCCNTENA_Msk             (0x1UL /*<< DWT_CTRL_CYCCNTENA_Pos*/)       /*!< DWT CTRL: CYCCNTENA Mask */

/* DWT CPI Count Register Definitions */
#define DWT_CPICNT_CPICNT_Pos               0U                                         /*!< DWT CPICNT: CPICNT Position */
#define DWT_CPICNT_CPICNT_Msk              (0xFFUL /*<< DWT_CPICNT_CPICNT_Pos*/)       /*!< DWT CPICNT: CPICNT Mask */

/* DWT Exception Overhead Count Register Definitions */
#define DWT_EXCCNT_EXCCNT_Pos               0U                                         /*!< DWT EXCCNT: EXCCNT Position */
#define DWT_EXCCNT_EXCCNT_Msk              (0xFFUL /*<< DWT_EXCCNT_EXCCNT_Pos*/)       /*!< DWT EXCCNT: EXCCNT Mask */

/* DWT Sleep Count Register Definitions */
#define DWT_SLEEPCNT_SLEEPCNT_Pos           0U                                         /*!< DWT SLEEPCNT: SLEEPCNT Position */
#define DWT_SLEEPCNT_SLEEPCNT_Msk          (0xFFUL /*<< DWT_SLEEPCNT_SLEEPCNT_Pos*/)   /*!< DWT SLEEPCNT: SLEEPCNT Mask */

/* DWT LSU Count Register Definitions */
#define DWT_LSUCNT_LSUCNT_Pos               0U                                         /*!< DWT LSUCNT: LSUCNT Position */
#define DWT_LSUCNT_LSUCNT_Msk              (0xFFUL /*<< DWT_LSUCNT_LSUCNT_Pos*/)       /*!< DWT LSUCNT: LSUCNT Mask */

/* DWT Folded-instruction Count Register Definitions */
#define DWT_FOLDCNT_FOLDCNT_Pos             0U                                         /*!< DWT FOLDCNT: FOLDCNT Position */
#define DWT_FOLDCNT_FOLDCNT_Msk            (0xFFUL /*<< DWT_FOLDCNT_FOLDCNT_Pos*/)     /*!< DWT FOLDCNT: FOLDCNT Mask */

/* DWT Comparator Function Register Definitions */
#define DWT_FUNCTION_ID_Pos                27U                                         /*!< DWT FUNCTION: ID Position */
#define DWT_FUNCTION_ID_Msk                (0x1FUL << DWT_FUNCTION_ID_Pos)             /*!< DWT FUNCTION: ID Mask */

#define DWT_FUNCTION_MATCHED_Pos           24U                                         /*!< DWT FUNCTION: MATCHED Position */
#define DWT_FUNCTION_MATCHED_Msk           (0x1UL << DWT_FUNCTION_MATCHED_Pos)         /*!< DWT FUNCTION: MATCHED Mask */

#define DWT_FUNCTION_DATAVSIZE_Pos         10U                                         /*!< DWT FUNCTION: DATAVSIZE Position */
#define DWT_FUNCTION_DATAVSIZE_Msk         (0x3UL << DWT_FUNCTION_DATAVSIZE_Pos)       /*!< DWT FUNCTION: DATAVSIZE Mask */

#define DWT_FUNCTION_ACTION_Pos             4U                                         /*!< DWT FUNCTION: ACTION Position */
#define DWT_FUNCTION_ACTION_Msk            (0x1UL << DWT_FUNCTION_ACTION_Pos)          /*!< DWT FUNCTION: ACTION Mask */

#define DWT_FUNCTION_MATCH_Pos              0U                                         /*!< DWT FUNCTION: MATCH Position */
#define DWT_FUNCTION_MATCH_Msk             (0xFUL /*<< DWT_FUNCTION_MATCH_Pos*/)       /*!< DWT FUNCTION: MATCH Mask */

/*@}*/ /* end of group CMSIS_DWT */


/**
  \ingroup  CMSIS_core_register
  \defgroup MemSysCtl_Type     Memory System Control Registers (IMPLEMENTATION DEFINED)
  \brief    Type definitions for the Memory System Control Registers (MEMSYSCTL)
  @{
 */

/**
  \brief  Structure type to access the Memory System Control Registers (MEMSYSCTL).
 */
typedef struct
{
  __IOM uint32_t MSCR;                   /*!< Offset: 0x000 (R/W)  Memory System Control Register */
  __IOM uint32_t PFCR;                   /*!< Offset: 0x004 (R/W)  Prefetcher Control Register */
        uint32_t RESERVED1[2U];
  __IOM uint32_t ITCMCR;                 /*!< Offset: 0x010 (R/W)  ITCM Control Register */
  __IOM uint32_t DTCMCR;                 /*!< Offset: 0x014 (R/W)  DTCM Control Register */
  __IOM uint32_t PAHBCR;                 /*!< Offset: 0x018 (R/W)  P-AHB Control Register */
        uint32_t RESERVED2[313U];
  __IOM uint32_t ITGU_CTRL;              /*!< Offset: 0x500 (R/W)  ITGU Control Register */
  __IOM uint32_t ITGU_CFG;               /*!< Offset: 0x504 (R/W)  ITGU Configuration Register */
        uint32_t RESERVED3[2U];
  __IOM uint32_t ITGU_LUT[16U];          /*!< Offset: 0x510 (R/W)  ITGU Look Up Table Register */
        uint32_t RESERVED4[44U];
  __IOM uint32_t DTGU_CTRL;              /*!< Offset: 0x600 (R/W)  DTGU Control Registers */
  __IOM uint32_t DTGU_CFG;               /*!< Offset: 0x604 (R/W)  DTGU Configuration Register */
        uint32_t RESERVED5[2U];
  __IOM uint32_t DTGU_LUT[16U];          /*!< Offset: 0x610 (R/W)  DTGU Look Up Table Register */
} MemSysCtl_Type;

/* MEMSYSCTL Memory System Control Register (MSCR) Register Definitions */
#define MEMSYSCTL_MSCR_CPWRDN_Pos          17U                                         /*!< MEMSYSCTL MSCR: CPWRDN Position */
#define MEMSYSCTL_MSCR_CPWRDN_Msk          (0x1UL << MEMSYSCTL_MSCR_CPWRDN_Pos)        /*!< MEMSYSCTL MSCR: CPWRDN Mask */

#define MEMSYSCTL_MSCR_DCCLEAN_Pos         16U                                         /*!< MEMSYSCTL MSCR: DCCLEAN Position */
#define MEMSYSCTL_MSCR_DCCLEAN_Msk         (0x1UL << MEMSYSCTL_MSCR_DCCLEAN_Pos)       /*!< MEMSYSCTL MSCR: DCCLEAN Mask */

#define MEMSYSCTL_MSCR_ICACTIVE_Pos        13U                                         /*!< MEMSYSCTL MSCR: ICACTIVE Position */
#define MEMSYSCTL_MSCR_ICACTIVE_Msk        (0x1UL << MEMSYSCTL_MSCR_ICACTIVE_Pos)      /*!< MEMSYSCTL MSCR: ICACTIVE Mask */

#define MEMSYSCTL_MSCR_DCACTIVE_Pos        12U                                         /*!< MEMSYSCTL MSCR: DCACTIVE Position */
#define MEMSYSCTL_MSCR_DCACTIVE_Msk        (0x1UL << MEMSYSCTL_MSCR_DCACTIVE_Pos)      /*!< MEMSYSCTL MSCR: DCACTIVE Mask */

#define MEMSYSCTL_MSCR_TECCCHKDIS_Pos       4U                                         /*!< MEMSYSCTL MSCR: TECCCHKDIS Position */
#define MEMSYSCTL_MSCR_TECCCHKDIS_Msk      (0x1UL << MEMSYSCTL_MSCR_TECCCHKDIS_Pos)    /*!< MEMSYSCTL MSCR: TECCCHKDIS Mask */

#define MEMSYSCTL_MSCR_EVECCFAULT_Pos       3U                                         /*!< MEMSYSCTL MSCR: EVECCFAULT Position */
#define MEMSYSCTL_MSCR_EVECCFAULT_Msk      (0x1UL << MEMSYSCTL_MSCR_EVECCFAULT_Pos)    /*!< MEMSYSCTL MSCR: EVECCFAULT Mask */

#define MEMSYSCTL_MSCR_FORCEWT_Pos          2U                                         /*!< MEMSYSCTL MSCR: FORCEWT Position */
#define MEMSYSCTL_MSCR_FORCEWT_Msk         (0x1UL << MEMSYSCTL_MSCR_FORCEWT_Pos)       /*!< MEMSYSCTL MSCR: FORCEWT Mask */

#define MEMSYSCTL_MSCR_ECCEN_Pos            1U                                         /*!< MEMSYSCTL MSCR: ECCEN Position */
#define MEMSYSCTL_MSCR_ECCEN_Msk           (0x1UL << MEMSYSCTL_MSCR_ECCEN_Pos)         /*!< MEMSYSCTL MSCR: ECCEN Mask */

/* MEMSYSCTL Prefetcher Control Register (PFCR) Register Definitions */
#define MEMSYSCTL_PFCR_MAX_OS_Pos           7U                                         /*!< MEMSYSCTL PFCR: MAX_OS Position */
#define MEMSYSCTL_PFCR_MAX_OS_Msk          (0x7UL << MEMSYSCTL_PFCR_MAX_OS_Pos)        /*!< MEMSYSCTL PFCR: MAX_OS Mask */

#define MEMSYSCTL_PFCR_MAX_LA_Pos           4U                                         /*!< MEMSYSCTL PFCR: MAX_LA Position */
#define MEMSYSCTL_PFCR_MAX_LA_Msk          (0x7UL << MEMSYSCTL_PFCR_MAX_LA_Pos)        /*!< MEMSYSCTL PFCR: MAX_LA Mask */

#define MEMSYSCTL_PFCR_MIN_LA_Pos           1U                                         /*!< MEMSYSCTL PFCR: MIN_LA Position */
#define MEMSYSCTL_PFCR_MIN_LA_Msk          (0x7UL << MEMSYSCTL_PFCR_MIN_LA_Pos)        /*!< MEMSYSCTL PFCR: MIN_LA Mask */

#define MEMSYSCTL_PFCR_ENABLE_Pos           0U                                         /*!< MEMSYSCTL PFCR: ENABLE Position */
#define MEMSYSCTL_PFCR_ENABLE_Msk          (0x1UL /*<< MEMSYSCTL_PFCR_ENABLE_Pos*/)    /*!< MEMSYSCTL PFCR: ENABLE Mask */

/* MEMSYSCTL ITCM Control Register (ITCMCR) Register Definitions */
#define MEMSYSCTL_ITCMCR_SZ_Pos             3U                                         /*!< MEMSYSCTL ITCMCR: SZ Position */
#define MEMSYSCTL_ITCMCR_SZ_Msk            (0xFUL << MEMSYSCTL_ITCMCR_SZ_Pos)          /*!< MEMSYSCTL ITCMCR: SZ Mask */

#define MEMSYSCTL_ITCMCR_EN_Pos             0U                                         /*!< MEMSYSCTL ITCMCR: EN Position */
#define MEMSYSCTL_ITCMCR_EN_Msk            (0x1UL /*<< MEMSYSCTL_ITCMCR_EN_Pos*/)      /*!< MEMSYSCTL ITCMCR: EN Mask */

/* MEMSYSCTL DTCM Control Register (DTCMCR) Register Definitions */
#define MEMSYSCTL_DTCMCR_SZ_Pos             3U                                         /*!< MEMSYSCTL DTCMCR: SZ Position */
#define MEMSYSCTL_DTCMCR_SZ_Msk            (0xFUL << MEMSYSCTL_DTCMCR_SZ_Pos)          /*!< MEMSYSCTL DTCMCR: SZ Mask */

#define MEMSYSCTL_DTCMCR_EN_Pos             0U                                         /*!< MEMSYSCTL DTCMCR: EN Position */
#define MEMSYSCTL_DTCMCR_EN_Msk            (0x1UL /*<< MEMSYSCTL_DTCMCR_EN_Pos*/)      /*!< MEMSYSCTL DTCMCR: EN Mask */

/* MEMSYSCTL P-AHB Control Register (PAHBCR) Register Definitions */
#define MEMSYSCTL_PAHBCR_SZ_Pos             1U                                         /*!< MEMSYSCTL PAHBCR: SZ Position */
#define MEMSYSCTL_PAHBCR_SZ_Msk            (0x7UL << MEMSYSCTL_PAHBCR_SZ_Pos)          /*!< MEMSYSCTL PAHBCR: SZ Mask */

#define MEMSYSCTL_PAHBCR_EN_Pos             0U                                         /*!< MEMSYSCTL PAHBCR: EN Position */
#define MEMSYSCTL_PAHBCR_EN_Msk            (0x1UL /*<< MEMSYSCTL_PAHBCR_EN_Pos*/)      /*!< MEMSYSCTL PAHBCR: EN Mask */

/* MEMSYSCTL ITGU Control Register (ITGU_CTRL) Register Definitions */
#define MEMSYSCTL_ITGU_CTRL_DEREN_Pos       1U                                         /*!< MEMSYSCTL ITGU_CTRL: DEREN Position */
#define MEMSYSCTL_ITGU_CTRL_DEREN_Msk      (0x1UL << MEMSYSCTL_ITGU_CTRL_DEREN_Pos)    /*!< MEMSYSCTL ITGU_CTRL: DEREN Mask */

#define MEMSYSCTL_ITGU_CTRL_DBFEN_Pos       0U                                         /*!< MEMSYSCTL ITGU_CTRL: DBFEN Position */
#define MEMSYSCTL_ITGU_CTRL_DBFEN_Msk      (0x1UL /*<< MEMSYSCTL_ITGU_CTRL_DBFEN_Pos*/) /*!< MEMSYSCTL ITGU_CTRL: DBFEN Mask */

/* MEMSYSCTL ITGU Configuration Register (ITGU_CFG) Register Definitions */
#define MEMSYSCTL_ITGU_CFG_PRESENT_Pos     31U                                         /*!< MEMSYSCTL ITGU_CFG: PRESENT Position */
#define MEMSYSCTL_ITGU_CFG_PRESENT_Msk     (0x1UL << MEMSYSCTL_ITGU_CFG_PRESENT_Pos)   /*!< MEMSYSCTL ITGU_CFG: PRESENT Mask */

#define MEMSYSCTL_ITGU_CFG_NUMBLKS_Pos      8U                                         /*!< MEMSYSCTL ITGU_CFG: NUMBLKS Position */
#define MEMSYSCTL_ITGU_CFG_NUMBLKS_Msk     (0xFUL << MEMSYSCTL_ITGU_CFG_NUMBLKS_Pos)   /*!< MEMSYSCTL ITGU_CFG: NUMBLKS Mask */

#define MEMSYSCTL_ITGU_CFG_BLKSZ_Pos        0U                                         /*!< MEMSYSCTL ITGU_CFG: BLKSZ Position */
#define MEMSYSCTL_ITGU_CFG_BLKSZ_Msk       (0xFUL /*<< MEMSYSCTL_ITGU_CFG_BLKSZ_Pos*/) /*!< MEMSYSCTL ITGU_CFG: BLKSZ Mask */

/* MEMSYSCTL DTGU Control Registers (DTGU_CTRL) Register Definitions */
#define MEMSYSCTL_DTGU_CTRL_DEREN_Pos       1U                                         /*!< MEMSYSCTL DTGU_CTRL: DEREN Position */
#define MEMSYSCTL_DTGU_CTRL_DEREN_Msk      (0x1UL << MEMSYSCTL_DTGU_CTRL_DEREN_Pos)    /*!< MEMSYSCTL DTGU_CTRL: DEREN Mask */

#define MEMSYSCTL_DTGU_CTRL_DBFEN_Pos       0U                                         /*!< MEMSYSCTL DTGU_CTRL: DBFEN Position */
#define MEMSYSCTL_DTGU_CTRL_DBFEN_Msk      (0x1UL /*<< MEMSYSCTL_DTGU_CTRL_DBFEN_Pos*/) /*!< MEMSYSCTL DTGU_CTRL: DBFEN Mask */

/* MEMSYSCTL DTGU Configuration Register (DTGU_CFG) Register Definitions */
#define MEMSYSCTL_DTGU_CFG_PRESENT_Pos     31U                                         /*!< MEMSYSCTL DTGU_CFG: PRESENT Position */
#define MEMSYSCTL_DTGU_CFG_PRESENT_Msk     (0x1UL << MEMSYSCTL_DTGU_CFG_PRESENT_Pos)   /*!< MEMSYSCTL DTGU_CFG: PRESENT Mask */

#define MEMSYSCTL_DTGU_CFG_NUMBLKS_Pos      8U                                         /*!< MEMSYSCTL DTGU_CFG: NUMBLKS Position */
#define MEMSYSCTL_DTGU_CFG_NUMBLKS_Msk     (0xFUL << MEMSYSCTL_DTGU_CFG_NUMBLKS_Pos)   /*!< MEMSYSCTL DTGU_CFG: NUMBLKS Mask */

#define MEMSYSCTL_DTGU_CFG_BLKSZ_Pos        0U                                         /*!< MEMSYSCTL DTGU_CFG: BLKSZ Position */
#define MEMSYSCTL_DTGU_CFG_BLKSZ_Msk       (0xFUL /*<< MEMSYSCTL_DTGU_CFG_BLKSZ_Pos*/) /*!< MEMSYSCTL DTGU_CFG: BLKSZ Mask */


/*@}*/ /* end of group MemSysCtl_Type */


/**
  \ingroup  CMSIS_core_register
  \defgroup PwrModCtl_Type     Power Mode Control Registers
  \brief    Type definitions for the Power Mode Control Registers (PWRMODCTL)
  @{
 */

/**
  \brief  Structure type to access the Power Mode Control Registers (PWRMODCTL).
 */
typedef struct
{
  __IOM uint32_t CPDLPSTATE;             /*!< Offset: 0x000 (R/W)  Core Power Domain Low Power State Register */
  __IOM uint32_t DPDLPSTATE;             /*!< Offset: 0x004 (R/W)  Debug Power Domain Low Power State Register */
} PwrModCtl_Type;

/* PWRMODCTL Core Power Domain Low Power State (CPDLPSTATE) Register Definitions */
#define PWRMODCTL_CPDLPSTATE_RLPSTATE_Pos   8U                                              /*!< PWRMODCTL CPDLPSTATE: RLPSTATE Position */
#define PWRMODCTL_CPDLPSTATE_RLPSTATE_Msk  (0x3UL << PWRMODCTL_CPDLPSTATE_RLPSTATE_Pos)     /*!< PWRMODCTL CPDLPSTATE: RLPSTATE Mask */

#define PWRMODCTL_CPDLPSTATE_ELPSTATE_Pos   4U                                              /*!< PWRMODCTL CPDLPSTATE: ELPSTATE Position */
#define PWRMODCTL_CPDLPSTATE_ELPSTATE_Msk  (0x3UL << PWRMODCTL_CPDLPSTATE_ELPSTATE_Pos)     /*!< PWRMODCTL CPDLPSTATE: ELPSTATE Mask */

#define PWRMODCTL_CPDLPSTATE_CLPSTATE_Pos   0U                                              /*!< PWRMODCTL CPDLPSTATE: CLPSTATE Position */
#define PWRMODCTL_CPDLPSTATE_CLPSTATE_Msk  (0x3UL /*<< PWRMODCTL_CPDLPSTATE_CLPSTATE_Pos*/) /*!< PWRMODCTL CPDLPSTATE: CLPSTATE Mask */

/* PWRMODCTL Debug Power Domain Low Power State (DPDLPSTATE) Register Definitions */
#define PWRMODCTL_DPDLPSTATE_DLPSTATE_Pos   0U                                              /*!< PWRMODCTL DPDLPSTATE: DLPSTATE Position */
#define PWRMODCTL_DPDLPSTATE_DLPSTATE_Msk  (0x3UL /*<< PWRMODCTL_DPDLPSTATE_DLPSTATE_Pos*/) /*!< PWRMODCTL DPDLPSTATE: DLPSTATE Mask */

/*@}*/ /* end of group PwrModCtl_Type */


/**
  \ingroup  CMSIS_core_register
  \defgroup EWIC_Type     External Wakeup Interrupt Controller Registers
  \brief    Type definitions for the External Wakeup Interrupt Controller Registers (EWIC)
  @{
 */

/**
  \brief  Structure type to access the External Wakeup Interrupt Controller Registers (EWIC).
 */
typedef struct
{
  __OM  uint32_t EVENTSPR;               /*!< Offset: 0x000 ( /W)  Event Set Pending Register */
        uint32_t RESERVED0[31U];
  __IM  uint32_t EVENTMASKA;             /*!< Offset: 0x080 (R/W)  Event Mask A Register */
  __IM  uint32_t EVENTMASK[15];          /*!< Offset: 0x084 (R/W)  Event Mask Register */
} EWIC_Type;

/* EWIC External Wakeup Interrupt Controller (EVENTSPR) Register Definitions */
#define EWIC_EVENTSPR_EDBGREQ_Pos   2U                                                 /*!< EWIC EVENTSPR: EDBGREQ Position */
#define EWIC_EVENTSPR_EDBGREQ_Msk  (0x1UL << EWIC_EVENTSPR_EDBGREQ_Pos)                /*!< EWIC EVENTSPR: EDBGREQ Mask */

#define EWIC_EVENTSPR_NMI_Pos   1U                                                     /*!< EWIC EVENTSPR: NMI Position */
#define EWIC_EVENTSPR_NMI_Msk  (0x1UL << EWIC_EVENTSPR_NMI_Pos)                        /*!< EWIC EVENTSPR: NMI Mask */

#define EWIC_EVENTSPR_EVENT_Pos   0U                                                   /*!< EWIC EVENTSPR: EVENT Position */
#define EWIC_EVENTSPR_EVENT_Msk  (0x1UL /*<< EWIC_EVENTSPR_EVENT_Pos*/)                /*!< EWIC EVENTSPR: EVENT Mask */

/* EWIC External Wakeup Interrupt Controller (EVENTMASKA) Register Definitions */
#define EWIC_EVENTMASKA_EDBGREQ_Pos   2U                                               /*!< EWIC EVENTMASKA: EDBGREQ Position */
#define EWIC_EVENTMASKA_EDBGREQ_Msk  (0x1UL << EWIC_EVENTMASKA_EDBGREQ_Pos)            /*!< EWIC EVENTMASKA: EDBGREQ Mask */

#define EWIC_EVENTMASKA_NMI_Pos   1U                                                   /*!< EWIC EVENTMASKA: NMI Position */
#define EWIC_EVENTMASKA_NMI_Msk  (0x1UL << EWIC_EVENTMASKA_NMI_Pos)                    /*!< EWIC EVENTMASKA: NMI Mask */

#define EWIC_EVENTMASKA_EVENT_Pos   0U                                                 /*!< EWIC EVENTMASKA: EVENT Position */
#define EWIC_EVENTMASKA_EVENT_Msk  (0x1UL /*<< EWIC_EVENTMASKA_EVENT_Pos*/)            /*!< EWIC EVENTMASKA: EVENT Mask */

/* EWIC External Wakeup Interrupt Controller (EVENTMASK) Register Definitions */
#define EWIC_EVENTMASK_IRQ_Pos   0U                                                    /*!< EWIC EVENTMASKA: IRQ Position */
#define EWIC_EVENTMASK_IRQ_Msk  (0xFFFFFFFFUL /*<< EWIC_EVENTMASKA_IRQ_Pos*/)          /*!< EWIC EVENTMASKA: IRQ Mask */

/*@}*/ /* end of group EWIC_Type */


/**
  \ingroup  CMSIS_core_register
  \defgroup ErrBnk_Type     Error Banking Registers (IMPLEMENTATION DEFINED)
  \brief    Type definitions for the Error Banking Registers (ERRBNK)
  @{
 */

/**
  \brief  Structure type to access the Error Banking Registers (ERRBNK).
 */
typedef struct
{
  __IOM uint32_t IEBR0;                  /*!< Offset: 0x000 (R/W)  Instruction Cache Error Bank Register 0 */
  __IOM uint32_t IEBR1;                  /*!< Offset: 0x004 (R/W)  Instruction Cache Error Bank Register 1 */
        uint32_t RESERVED0[2U];
  __IOM uint32_t DEBR0;                  /*!< Offset: 0x010 (R/W)  Data Cache Error Bank Register 0 */
  __IOM uint32_t DEBR1;                  /*!< Offset: 0x014 (R/W)  Data Cache Error Bank Register 1 */
        uint32_t RESERVED1[2U];
  __IOM uint32_t TEBR0;                  /*!< Offset: 0x020 (R/W)  TCM Error Bank Register 0 */
        uint32_t RESERVED2[1U];
  __IOM uint32_t TEBR1;                  /*!< Offset: 0x028 (R/W)  TCM Error Bank Register 1 */
} ErrBnk_Type;

/* ERRBNK Instruction Cache Error Bank Register 0 (IEBR0) Register Definitions */
#define ERRBNK_IEBR0_SWDEF_Pos             30U                                         /*!< ERRBNK IEBR0: SWDEF Position */
#define ERRBNK_IEBR0_SWDEF_Msk             (0x3UL << ERRBNK_IEBR0_SWDEF_Pos)           /*!< ERRBNK IEBR0: SWDEF Mask */

#define ERRBNK_IEBR0_BANK_Pos              16U                                         /*!< ERRBNK IEBR0: BANK Position */
#define ERRBNK_IEBR0_BANK_Msk              (0x1UL << ERRBNK_IEBR0_BANK_Pos)            /*!< ERRBNK IEBR0: BANK Mask */

#define ERRBNK_IEBR0_LOCATION_Pos           2U                                         /*!< ERRBNK IEBR0: LOCATION Position */
#define ERRBNK_IEBR0_LOCATION_Msk          (0x3FFFUL << ERRBNK_IEBR0_LOCATION_Pos)     /*!< ERRBNK IEBR0: LOCATION Mask */

#define ERRBNK_IEBR0_LOCKED_Pos             1U                                         /*!< ERRBNK IEBR0: LOCKED Position */
#define ERRBNK_IEBR0_LOCKED_Msk            (0x1UL << ERRBNK_IEBR0_LOCKED_Pos)          /*!< ERRBNK IEBR0: LOCKED Mask */

#define ERRBNK_IEBR0_VALID_Pos              0U                                         /*!< ERRBNK IEBR0: VALID Position */
#define ERRBNK_IEBR0_VALID_Msk             (0x1UL << /*ERRBNK_IEBR0_VALID_Pos*/)       /*!< ERRBNK IEBR0: VALID Mask */

/* ERRBNK Instruction Cache Error Bank Register 1 (IEBR1) Register Definitions */
#define ERRBNK_IEBR1_SWDEF_Pos             30U                                         /*!< ERRBNK IEBR1: SWDEF Position */
#define ERRBNK_IEBR1_SWDEF_Msk             (0x3UL << ERRBNK_IEBR1_SWDEF_Pos)           /*!< ERRBNK IEBR1: SWDEF Mask */

#define ERRBNK_IEBR1_BANK_Pos              16U                                         /*!< ERRBNK IEBR1: BANK Position */
#define ERRBNK_IEBR1_BANK_Msk              (0x1UL << ERRBNK_IEBR1_BANK_Pos)            /*!< ERRBNK IEBR1: BANK Mask */

#define ERRBNK_IEBR1_LOCATION_Pos           2U                                         /*!< ERRBNK IEBR1: LOCATION Position */
#define ERRBNK_IEBR1_LOCATION_Msk          (0x3FFFUL << ERRBNK_IEBR1_LOCATION_Pos)     /*!< ERRBNK IEBR1: LOCATION Mask */

#define ERRBNK_IEBR1_LOCKED_Pos             1U                                         /*!< ERRBNK IEBR1: LOCKED Position */
#define ERRBNK_IEBR1_LOCKED_Msk            (0x1UL << ERRBNK_IEBR1_LOCKED_Pos)          /*!< ERRBNK IEBR1: LOCKED Mask */

#define ERRBNK_IEBR1_VALID_Pos              0U                                         /*!< ERRBNK IEBR1: VALID Position */
#define ERRBNK_IEBR1_VALID_Msk             (0x1UL << /*ERRBNK_IEBR1_VALID_Pos*/)       /*!< ERRBNK IEBR1: VALID Mask */

/* ERRBNK Data Cache Error Bank Register 0 (DEBR0) Register Definitions */
#define ERRBNK_DEBR0_SWDEF_Pos             30U                                         /*!< ERRBNK DEBR0: SWDEF Position */
#define ERRBNK_DEBR0_SWDEF_Msk             (0x3UL << ERRBNK_DEBR0_SWDEF_Pos)           /*!< ERRBNK DEBR0: SWDEF Mask */

#define ERRBNK_DEBR0_TYPE_Pos              17U                                         /*!< ERRBNK DEBR0: TYPE Position */
#define ERRBNK_DEBR0_TYPE_Msk              (0x1UL << ERRBNK_DEBR0_TYPE_Pos)            /*!< ERRBNK DEBR0: TYPE Mask */

#define ERRBNK_DEBR0_BANK_Pos              16U                                         /*!< ERRBNK DEBR0: BANK Position */
#define ERRBNK_DEBR0_BANK_Msk              (0x1UL << ERRBNK_DEBR0_BANK_Pos)            /*!< ERRBNK DEBR0: BANK Mask */

#define ERRBNK_DEBR0_LOCATION_Pos           2U                                         /*!< ERRBNK DEBR0: LOCATION Position */
#define ERRBNK_DEBR0_LOCATION_Msk          (0x3FFFUL << ERRBNK_DEBR0_LOCATION_Pos)     /*!< ERRBNK DEBR0: LOCATION Mask */

#define ERRBNK_DEBR0_LOCKED_Pos             1U                                         /*!< ERRBNK DEBR0: LOCKED Position */
#define ERRBNK_DEBR0_LOCKED_Msk            (0x1UL << ERRBNK_DEBR0_LOCKED_Pos)          /*!< ERRBNK DEBR0: LOCKED Mask */

#define ERRBNK_DEBR0_VALID_Pos              0U                                         /*!< ERRBNK DEBR0: VALID Position */
#define ERRBNK_DEBR0_VALID_Msk             (0x1UL << /*ERRBNK_DEBR0_VALID_Pos*/)       /*!< ERRBNK DEBR0: VALID Mask */

/* ERRBNK Data Cache Error Bank Register 1 (DEBR1) Register Definitions */
#define ERRBNK_DEBR1_SWDEF_Pos             30U                                         /*!< ERRBNK DEBR1: SWDEF Position */
#define ERRBNK_DEBR1_SWDEF_Msk             (0x3UL << ERRBNK_DEBR1_SWDEF_Pos)           /*!< ERRBNK DEBR1: SWDEF Mask */

#define ERRBNK_DEBR1_TYPE_Pos              17U                                         /*!< ERRBNK DEBR1: TYPE Position */
#define ERRBNK_DEBR1_TYPE_Msk              (0x1UL << ERRBNK_DEBR1_TYPE_Pos)            /*!< ERRBNK DEBR1: TYPE Mask */

#define ERRBNK_DEBR1_BANK_Pos              16U                                         /*!< ERRBNK DEBR1: BANK Position */
#define ERRBNK_DEBR1_BANK_Msk              (0x1UL << ERRBNK_DEBR1_BANK_Pos)            /*!< ERRBNK DEBR1: BANK Mask */

#define ERRBNK_DEBR1_LOCATION_Pos           2U                                         /*!< ERRBNK DEBR1: LOCATION Position */
#define ERRBNK_DEBR1_LOCATION_Msk          (0x3FFFUL << ERRBNK_DEBR1_LOCATION_Pos)     /*!< ERRBNK DEBR1: LOCATION Mask */

#define ERRBNK_DEBR1_LOCKED_Pos             1U                                         /*!< ERRBNK DEBR1: LOCKED Position */
#define ERRBNK_DEBR1_LOCKED_Msk            (0x1UL << ERRBNK_DEBR1_LOCKED_Pos)          /*!< ERRBNK DEBR1: LOCKED Mask */

#define ERRBNK_DEBR1_VALID_Pos              0U                                         /*!< ERRBNK DEBR1: VALID Position */
#define ERRBNK_DEBR1_VALID_Msk             (0x1UL << /*ERRBNK_DEBR1_VALID_Pos*/)       /*!< ERRBNK DEBR1: VALID Mask */

/* ERRBNK TCM Error Bank Register 0 (TEBR0) Register Definitions */
#define ERRBNK_TEBR0_SWDEF_Pos             30U                                         /*!< ERRBNK TEBR0: SWDEF Position */
#define ERRBNK_TEBR0_SWDEF_Msk             (0x3UL << ERRBNK_TEBR0_SWDEF_Pos)           /*!< ERRBNK TEBR0: SWDEF Mask */

#define ERRBNK_TEBR0_POISON_Pos            28U                                         /*!< ERRBNK TEBR0: POISON Position */
#define ERRBNK_TEBR0_POISON_Msk            (0x1UL << ERRBNK_TEBR0_POISON_Pos)          /*!< ERRBNK TEBR0: POISON Mask */

#define ERRBNK_TEBR0_TYPE_Pos              27U                                         /*!< ERRBNK TEBR0: TYPE Position */
#define ERRBNK_TEBR0_TYPE_Msk              (0x1UL << ERRBNK_TEBR0_TYPE_Pos)            /*!< ERRBNK TEBR0: TYPE Mask */

#define ERRBNK_TEBR0_BANK_Pos              24U                                         /*!< ERRBNK TEBR0: BANK Position */
#define ERRBNK_TEBR0_BANK_Msk              (0x3UL << ERRBNK_TEBR0_BANK_Pos)            /*!< ERRBNK TEBR0: BANK Mask */

#define ERRBNK_TEBR0_LOCATION_Pos           2U                                         /*!< ERRBNK TEBR0: LOCATION Position */
#define ERRBNK_TEBR0_LOCATION_Msk          (0x3FFFFFUL << ERRBNK_TEBR0_LOCATION_Pos)   /*!< ERRBNK TEBR0: LOCATION Mask */

#define ERRBNK_TEBR0_LOCKED_Pos             1U                                         /*!< ERRBNK TEBR0: LOCKED Position */
#define ERRBNK_TEBR0_LOCKED_Msk            (0x1UL << ERRBNK_TEBR0_LOCKED_Pos)          /*!< ERRBNK TEBR0: LOCKED Mask */

#define ERRBNK_TEBR0_VALID_Pos              0U                                         /*!< ERRBNK TEBR0: VALID Position */
#define ERRBNK_TEBR0_VALID_Msk             (0x1UL << /*ERRBNK_TEBR0_VALID_Pos*/)       /*!< ERRBNK TEBR0: VALID Mask */

/* ERRBNK TCM Error Bank Register 1 (TEBR1) Register Definitions */
#define ERRBNK_TEBR1_SWDEF_Pos             30U                                         /*!< ERRBNK TEBR1: SWDEF Position */
#define ERRBNK_TEBR1_SWDEF_Msk             (0x3UL << ERRBNK_TEBR1_SWDEF_Pos)           /*!< ERRBNK TEBR1: SWDEF Mask */

#define ERRBNK_TEBR1_POISON_Pos            28U                                         /*!< ERRBNK TEBR1: POISON Position */
#define ERRBNK_TEBR1_POISON_Msk            (0x1UL << ERRBNK_TEBR1_POISON_Pos)          /*!< ERRBNK TEBR1: POISON Mask */

#define ERRBNK_TEBR1_TYPE_Pos              27U                                         /*!< ERRBNK TEBR1: TYPE Position */
#define ERRBNK_TEBR1_TYPE_Msk              (0x1UL << ERRBNK_TEBR1_TYPE_Pos)            /*!< ERRBNK TEBR1: TYPE Mask */

#define ERRBNK_TEBR1_BANK_Pos              24U                                         /*!< ERRBNK TEBR1: BANK Position */
#define ERRBNK_TEBR1_BANK_Msk              (0x3UL << ERRBNK_TEBR1_BANK_Pos)            /*!< ERRBNK TEBR1: BANK Mask */

#define ERRBNK_TEBR1_LOCATION_Pos           2U                                         /*!< ERRBNK TEBR1: LOCATION Position */
#define ERRBNK_TEBR1_LOCATION_Msk          (0x3FFFFFUL << ERRBNK_TEBR1_LOCATION_Pos)   /*!< ERRBNK TEBR1: LOCATION Mask */

#define ERRBNK_TEBR1_LOCKED_Pos             1U                                         /*!< ERRBNK TEBR1: LOCKED Position */
#define ERRBNK_TEBR1_LOCKED_Msk            (0x1UL << ERRBNK_TEBR1_LOCKED_Pos)          /*!< ERRBNK TEBR1: LOCKED Mask */

#define ERRBNK_TEBR1_VALID_Pos              0U                                         /*!< ERRBNK TEBR1: VALID Position */
#define ERRBNK_TEBR1_VALID_Msk             (0x1UL << /*ERRBNK_TEBR1_VALID_Pos*/)       /*!< ERRBNK TEBR1: VALID Mask */

/*@}*/ /* end of group ErrBnk_Type */


/**
  \ingroup  CMSIS_core_register
  \defgroup PrcCfgInf_Type     Processor Configuration Information Registers (IMPLEMENTATION DEFINED)
  \brief    Type definitions for the Processor Configuration Information Registerss (PRCCFGINF)
  @{
 */

/**
  \brief  Structure type to access the Processor Configuration Information Registerss (PRCCFGINF).
 */
typedef struct
{
  __OM  uint32_t CFGINFOSEL;             /*!< Offset: 0x000 ( /W)  Processor Configuration Information Selection Register */
  __IM  uint32_t CFGINFORD;              /*!< Offset: 0x004 (R/ )  Processor Configuration Information Read Data Register */
} PrcCfgInf_Type;

/* PRCCFGINF Processor Configuration Information Selection Register (CFGINFOSEL) Definitions */

/* PRCCFGINF Processor Configuration Information Read Data Register (CFGINFORD) Definitions */

/*@}*/ /* end of group PrcCfgInf_Type */


/**
  \ingroup  CMSIS_core_register
  \defgroup STL_Type     Software Test Library Observation Registers
  \brief    Type definitions for the Software Test Library Observation Registerss (STL)
  @{
 */

/**
  \brief  Structure type to access the Software Test Library Observation Registerss (STL).
 */
typedef struct
{
  __IM  uint32_t STLNVICPENDOR;          /*!< Offset: 0x000 (R/ )  NVIC Pending Priority Tree Register */
  __IM  uint32_t STLNVICACTVOR;          /*!< Offset: 0x004 (R/ )  NVIC Active Priority Tree Register */
        uint32_t RESERVED0[2U];
  __OM  uint32_t STLIDMPUSR;             /*!< Offset: 0x010 ( /W)  MPU Sanple Register */
  __IM  uint32_t STLIMPUOR;              /*!< Offset: 0x014 (R/ )  MPU Region Hit Register */
  __IM  uint32_t STLD0MPUOR;             /*!< Offset: 0x018 (R/ )  MPU Memory Attributes Register 0 */
  __IM  uint32_t STLD1MPUOR;             /*!< Offset: 0x01C (R/ )  MPU Memory Attributes Register 1 */

} STL_Type;

/* STL Software Test Library Observation Register (STLNVICPENDOR) Definitions */
#define STL_STLNVICPENDOR_VALID_Pos        18U                                         /*!< STL STLNVICPENDOR: VALID Position */
#define STL_STLNVICPENDOR_VALID_Msk        (0x1UL << STL_STLNVICPENDOR_VALID_Pos)      /*!< STL STLNVICPENDOR: VALID Mask */

#define STL_STLNVICPENDOR_TARGET_Pos       17U                                         /*!< STL STLNVICPENDOR: TARGET Position */
#define STL_STLNVICPENDOR_TARGET_Msk       (0x1UL << STL_STLNVICPENDOR_TARGET_Pos)     /*!< STL STLNVICPENDOR: TARGET Mask */

#define STL_STLNVICPENDOR_PRIORITY_Pos      9U                                         /*!< STL STLNVICPENDOR: PRIORITY Position */
#define STL_STLNVICPENDOR_PRIORITY_Msk     (0xFFUL << STL_STLNVICPENDOR_PRIORITY_Pos)  /*!< STL STLNVICPENDOR: PRIORITY Mask */

#define STL_STLNVICPENDOR_INTNUM_Pos        0U                                         /*!< STL STLNVICPENDOR: INTNUM Position */
#define STL_STLNVICPENDOR_INTNUM_Msk       (0x1FFUL /*<< STL_STLNVICPENDOR_INTNUM_Pos*/) /*!< STL STLNVICPENDOR: INTNUM Mask */

/* STL Software Test Library Observation Register (STLNVICACTVOR) Definitions */
#define STL_STLNVICACTVOR_VALID_Pos        18U                                         /*!< STL STLNVICACTVOR: VALID Position */
#define STL_STLNVICACTVOR_VALID_Msk        (0x1UL << STL_STLNVICACTVOR_VALID_Pos)      /*!< STL STLNVICACTVOR: VALID Mask */

#define STL_STLNVICACTVOR_TARGET_Pos       17U                                         /*!< STL STLNVICACTVOR: TARGET Position */
#define STL_STLNVICACTVOR_TARGET_Msk       (0x1UL << STL_STLNVICACTVOR_TARGET_Pos)     /*!< STL STLNVICACTVOR: TARGET Mask */

#define STL_STLNVICACTVOR_PRIORITY_Pos      9U                                         /*!< STL STLNVICACTVOR: PRIORITY Position */
#define STL_STLNVICACTVOR_PRIORITY_Msk     (0xFFUL << STL_STLNVICACTVOR_PRIORITY_Pos)  /*!< STL STLNVICACTVOR: PRIORITY Mask */

#define STL_STLNVICACTVOR_INTNUM_Pos        0U                                         /*!< STL STLNVICACTVOR: INTNUM Position */
#define STL_STLNVICACTVOR_INTNUM_Msk       (0x1FFUL /*<< STL_STLNVICACTVOR_INTNUM_Pos*/) /*!< STL STLNVICACTVOR: INTNUM Mask */

/* STL Software Test Library Observation Register (STLIDMPUSR) Definitions */
#define STL_STLIDMPUSR_ADDR_Pos             5U                                         /*!< STL STLIDMPUSR: ADDR Position */
#define STL_STLIDMPUSR_ADDR_Msk            (0x7FFFFFFUL << STL_STLIDMPUSR_ADDR_Pos)    /*!< STL STLIDMPUSR: ADDR Mask */

#define STL_STLIDMPUSR_INSTR_Pos            2U                                         /*!< STL STLIDMPUSR: INSTR Position */
#define STL_STLIDMPUSR_INSTR_Msk           (0x1UL << STL_STLIDMPUSR_INSTR_Pos)         /*!< STL STLIDMPUSR: INSTR Mask */

#define STL_STLIDMPUSR_DATA_Pos             1U                                         /*!< STL STLIDMPUSR: DATA Position */
#define STL_STLIDMPUSR_DATA_Msk            (0x1UL << STL_STLIDMPUSR_DATA_Pos)          /*!< STL STLIDMPUSR: DATA Mask */

/* STL Software Test Library Observation Register (STLIMPUOR) Definitions */
#define STL_STLIMPUOR_HITREGION_Pos         9U                                         /*!< STL STLIMPUOR: HITREGION Position */
#define STL_STLIMPUOR_HITREGION_Msk        (0xFFUL << STL_STLIMPUOR_HITREGION_Pos)     /*!< STL STLIMPUOR: HITREGION Mask */

#define STL_STLIMPUOR_ATTR_Pos              0U                                         /*!< STL STLIMPUOR: ATTR Position */
#define STL_STLIMPUOR_ATTR_Msk             (0x1FFUL /*<< STL_STLIMPUOR_ATTR_Pos*/)     /*!< STL STLIMPUOR: ATTR Mask */

/* STL Software Test Library Observation Register (STLD0MPUOR) Definitions */
#define STL_STLD0MPUOR_HITREGION_Pos        9U                                         /*!< STL STLD0MPUOR: HITREGION Position */
#define STL_STLD0MPUOR_HITREGION_Msk       (0xFFUL << STL_STLD0MPUOR_HITREGION_Pos)    /*!< STL STLD0MPUOR: HITREGION Mask */

#define STL_STLD0MPUOR_ATTR_Pos             0U                                         /*!< STL STLD0MPUOR: ATTR Position */
#define STL_STLD0MPUOR_ATTR_Msk            (0x1FFUL /*<< STL_STLD0MPUOR_ATTR_Pos*/)    /*!< STL STLD0MPUOR: ATTR Mask */

/* STL Software Test Library Observation Register (STLD1MPUOR) Definitions */
#define STL_STLD1MPUOR_HITREGION_Pos        9U                                         /*!< STL STLD1MPUOR: HITREGION Position */
#define STL_STLD1MPUOR_HITREGION_Msk       (0xFFUL << STL_STLD1MPUOR_HITREGION_Pos)    /*!< STL STLD1MPUOR: HITREGION Mask */

#define STL_STLD1MPUOR_ATTR_Pos             0U                                         /*!< STL STLD1MPUOR: ATTR Position */
#define STL_STLD1MPUOR_ATTR_Msk            (0x1FFUL /*<< STL_STLD1MPUOR_ATTR_Pos*/)    /*!< STL STLD1MPUOR: ATTR Mask */

/*@}*/ /* end of group STL_Type */


/**
  \ingroup  CMSIS_core_register
  \defgroup CMSIS_TPI     Trace Port Interface (TPI)
  \brief    Type definitions for the Trace Port Interface (TPI)
  @{
 */

/**
  \brief  Structure type to access the Trace Port Interface Register (TPI).
 */
typedef struct
{
  __IM  uint32_t SSPSR;                  /*!< Offset: 0x000 (R/ )  Supported Parallel Port Sizes Register */
  __IOM uint32_t CSPSR;                  /*!< Offset: 0x004 (R/W)  Current Parallel Port Sizes Register */
        uint32_t RESERVED0[2U];
  __IOM uint32_t ACPR;                   /*!< Offset: 0x010 (R/W)  Asynchronous Clock Prescaler Register */
        uint32_t RESERVED1[55U];
  __IOM uint32_t SPPR;                   /*!< Offset: 0x0F0 (R/W)  Selected Pin Protocol Register */
        uint32_t RESERVED2[131U];
  __IM  uint32_t FFSR;                   /*!< Offset: 0x300 (R/ )  Formatter and Flush Status Register */
  __IOM uint32_t FFCR;                   /*!< Offset: 0x304 (R/W)  Formatter and Flush Control Register */
  __IOM uint32_t PSCR;                   /*!< Offset: 0x308 (R/W)  Periodic Synchronization Control Register */
        uint32_t RESERVED3[809U];
  __OM  uint32_t LAR;                    /*!< Offset: 0xFB0 ( /W)  Software Lock Access Register */
  __IM  uint32_t LSR;                    /*!< Offset: 0xFB4 (R/ )  Software Lock Status Register */
        uint32_t RESERVED4[4U];
  __IM  uint32_t TYPE;                   /*!< Offset: 0xFC8 (R/ )  Device Identifier Register */
  __IM  uint32_t DEVTYPE;                /*!< Offset: 0xFCC (R/ )  Device Type Register */
} TPI_Type;

/* TPI Asynchronous Clock Prescaler Register Definitions */
#define TPI_ACPR_SWOSCALER_Pos              0U                                         /*!< TPI ACPR: SWOSCALER Position */
#define TPI_ACPR_SWOSCALER_Msk             (0xFFFFUL /*<< TPI_ACPR_SWOSCALER_Pos*/)    /*!< TPI ACPR: SWOSCALER Mask */

/* TPI Selected Pin Protocol Register Definitions */
#define TPI_SPPR_TXMODE_Pos                 0U                                         /*!< TPI SPPR: TXMODE Position */
#define TPI_SPPR_TXMODE_Msk                (0x3UL /*<< TPI_SPPR_TXMODE_Pos*/)          /*!< TPI SPPR: TXMODE Mask */

/* TPI Formatter and Flush Status Register Definitions */
#define TPI_FFSR_FtNonStop_Pos              3U                                         /*!< TPI FFSR: FtNonStop Position */
#define TPI_FFSR_FtNonStop_Msk             (0x1UL << TPI_FFSR_FtNonStop_Pos)           /*!< TPI FFSR: FtNonStop Mask */

#define TPI_FFSR_TCPresent_Pos              2U                                         /*!< TPI FFSR: TCPresent Position */
#define TPI_FFSR_TCPresent_Msk             (0x1UL << TPI_FFSR_TCPresent_Pos)           /*!< TPI FFSR: TCPresent Mask */

#define TPI_FFSR_FtStopped_Pos              1U                                         /*!< TPI FFSR: FtStopped Position */
#define TPI_FFSR_FtStopped_Msk             (0x1UL << TPI_FFSR_FtStopped_Pos)           /*!< TPI FFSR: FtStopped Mask */

#define TPI_FFSR_FlInProg_Pos               0U                                         /*!< TPI FFSR: FlInProg Position */
#define TPI_FFSR_FlInProg_Msk              (0x1UL /*<< TPI_FFSR_FlInProg_Pos*/)        /*!< TPI FFSR: FlInProg Mask */

/* TPI Formatter and Flush Control Register Definitions */
#define TPI_FFCR_TrigIn_Pos                 8U                                         /*!< TPI FFCR: TrigIn Position */
#define TPI_FFCR_TrigIn_Msk                (0x1UL << TPI_FFCR_TrigIn_Pos)              /*!< TPI FFCR: TrigIn Mask */

#define TPI_FFCR_FOnMan_Pos                 6U                                         /*!< TPI FFCR: FOnMan Position */
#define TPI_FFCR_FOnMan_Msk                (0x1UL << TPI_FFCR_FOnMan_Pos)              /*!< TPI FFCR: FOnMan Mask */

#define TPI_FFCR_EnFmt_Pos                  0U                                         /*!< TPI FFCR: EnFmt Position */
#define TPI_FFCR_EnFmt_Msk                 (0x3UL << /*TPI_FFCR_EnFmt_Pos*/)           /*!< TPI FFCR: EnFmt Mask */

/* TPI Periodic Synchronization Control Register Definitions */
#define TPI_PSCR_PSCount_Pos                0U                                         /*!< TPI PSCR: PSCount Position */
#define TPI_PSCR_PSCount_Msk               (0x1FUL /*<< TPI_PSCR_PSCount_Pos*/)        /*!< TPI PSCR: TPSCount Mask */

/* TPI Software Lock Status Register Definitions */
#define TPI_LSR_nTT_Pos                     1U                                         /*!< TPI LSR: Not thirty-two bit. Position */
#define TPI_LSR_nTT_Msk                    (0x1UL << TPI_LSR_nTT_Pos)                  /*!< TPI LSR: Not thirty-two bit. Mask */

#define TPI_LSR_SLK_Pos                     1U                                         /*!< TPI LSR: Software Lock status Position */
#define TPI_LSR_SLK_Msk                    (0x1UL << TPI_LSR_SLK_Pos)                  /*!< TPI LSR: Software Lock status Mask */

#define TPI_LSR_SLI_Pos                     0U                                         /*!< TPI LSR: Software Lock implemented Position */
#define TPI_LSR_SLI_Msk                    (0x1UL /*<< TPI_LSR_SLI_Pos*/)              /*!< TPI LSR: Software Lock implemented Mask */

/* TPI DEVID Register Definitions */
#define TPI_DEVID_NRZVALID_Pos             11U                                         /*!< TPI DEVID: NRZVALID Position */
#define TPI_DEVID_NRZVALID_Msk             (0x1UL << TPI_DEVID_NRZVALID_Pos)           /*!< TPI DEVID: NRZVALID Mask */

#define TPI_DEVID_MANCVALID_Pos            10U                                         /*!< TPI DEVID: MANCVALID Position */
#define TPI_DEVID_MANCVALID_Msk            (0x1UL << TPI_DEVID_MANCVALID_Pos)          /*!< TPI DEVID: MANCVALID Mask */

#define TPI_DEVID_PTINVALID_Pos             9U                                         /*!< TPI DEVID: PTINVALID Position */
#define TPI_DEVID_PTINVALID_Msk            (0x1UL << TPI_DEVID_PTINVALID_Pos)          /*!< TPI DEVID: PTINVALID Mask */

#define TPI_DEVID_FIFOSZ_Pos                6U                                         /*!< TPI DEVID: FIFO depth Position */
#define TPI_DEVID_FIFOSZ_Msk               (0x7UL << TPI_DEVID_FIFOSZ_Pos)             /*!< TPI DEVID: FIFO depth Mask */

/* TPI DEVTYPE Register Definitions */
#define TPI_DEVTYPE_SubType_Pos             4U                                         /*!< TPI DEVTYPE: SubType Position */
#define TPI_DEVTYPE_SubType_Msk            (0xFUL /*<< TPI_DEVTYPE_SubType_Pos*/)      /*!< TPI DEVTYPE: SubType Mask */

#define TPI_DEVTYPE_MajorType_Pos           0U                                         /*!< TPI DEVTYPE: MajorType Position */
#define TPI_DEVTYPE_MajorType_Msk          (0xFUL << TPI_DEVTYPE_MajorType_Pos)        /*!< TPI DEVTYPE: MajorType Mask */

/*@}*/ /* end of group CMSIS_TPI */

#if defined (__PMU_PRESENT) && (__PMU_PRESENT == 1U)
/**
  \ingroup  CMSIS_core_register
  \defgroup CMSIS_PMU     Performance Monitoring Unit (PMU)
  \brief    Type definitions for the Performance Monitoring Unit (PMU)
  @{
 */

/**
  \brief  Structure type to access the Performance Monitoring Unit (PMU).
 */
typedef struct
{
  __IOM uint32_t EVCNTR[__PMU_NUM_EVENTCNT];        /*!< Offset: 0x0 (R/W)    PMU Event Counter Registers */
#if __PMU_NUM_EVENTCNT<31
        uint32_t RESERVED0[31U-__PMU_NUM_EVENTCNT];
#endif
  __IOM uint32_t CCNTR;                             /*!< Offset: 0x7C (R/W)   PMU Cycle Counter Register */
        uint32_t RESERVED1[224];
  __IOM uint32_t EVTYPER[__PMU_NUM_EVENTCNT];       /*!< Offset: 0x400 (R/W)  PMU Event Type and Filter Registers */
#if __PMU_NUM_EVENTCNT<31
        uint32_t RESERVED2[31U-__PMU_NUM_EVENTCNT];
#endif
  __IOM uint32_t CCFILTR;                           /*!< Offset: 0x47C (R/W)  PMU Cycle Counter Filter Register */
        uint32_t RESERVED3[480];
  __IOM uint32_t CNTENSET;                          /*!< Offset: 0xC00 (R/W)  PMU Count Enable Set Register */
        uint32_t RESERVED4[7];
  __IOM uint32_t CNTENCLR;                          /*!< Offset: 0xC20 (R/W)  PMU Count Enable Clear Register */
        uint32_t RESERVED5[7];
  __IOM uint32_t INTENSET;                          /*!< Offset: 0xC40 (R/W)  PMU Interrupt Enable Set Register */
        uint32_t RESERVED6[7];
  __IOM uint32_t INTENCLR;                          /*!< Offset: 0xC60 (R/W)  PMU Interrupt Enable Clear Register */
        uint32_t RESERVED7[7];
  __IOM uint32_t OVSCLR;                            /*!< Offset: 0xC80 (R/W)  PMU Overflow Flag Status Clear Register */
        uint32_t RESERVED8[7];
  __IOM uint32_t SWINC;                             /*!< Offset: 0xCA0 (R/W)  PMU Software Increment Register */
        uint32_t RESERVED9[7];
  __IOM uint32_t OVSSET;                            /*!< Offset: 0xCC0 (R/W)  PMU Overflow Flag Status Set Register */
        uint32_t RESERVED10[79];
  __IOM uint32_t TYPE;                              /*!< Offset: 0xE00 (R/W)  PMU Type Register */
  __IOM uint32_t CTRL;                              /*!< Offset: 0xE04 (R/W)  PMU Control Register */
        uint32_t RESERVED11[108];
  __IOM uint32_t AUTHSTATUS;                        /*!< Offset: 0xFB8 (R/W)  PMU Authentication Status Register */
  __IOM uint32_t DEVARCH;                           /*!< Offset: 0xFBC (R/W)  PMU Device Architecture Register */
        uint32_t RESERVED12[3];
  __IOM uint32_t DEVTYPE;                           /*!< Offset: 0xFCC (R/W)  PMU Device Type Register */
  __IOM uint32_t PIDR4;                             /*!< Offset: 0xFD0 (R/W)  PMU Peripheral Identification Register 4 */
        uint32_t RESERVED13[3];
  __IOM uint32_t PIDR0;                             /*!< Offset: 0xFE0 (R/W)  PMU Peripheral Identification Register 0 */
  __IOM uint32_t PIDR1;                             /*!< Offset: 0xFE4 (R/W)  PMU Peripheral Identification Register 1 */
  __IOM uint32_t PIDR2;                             /*!< Offset: 0xFE8 (R/W)  PMU Peripheral Identification Register 2 */
  __IOM uint32_t PIDR3;                             /*!< Offset: 0xFEC (R/W)  PMU Peripheral Identification Register 3 */
  __IOM uint32_t CIDR0;                             /*!< Offset: 0xFF0 (R/W)  PMU Component Identification Register 0 */
  __IOM uint32_t CIDR1;                             /*!< Offset: 0xFF4 (R/W)  PMU Component Identification Register 1 */
  __IOM uint32_t CIDR2;                             /*!< Offset: 0xFF8 (R/W)  PMU Component Identification Register 2 */
  __IOM uint32_t CIDR3;                             /*!< Offset: 0xFFC (R/W)  PMU Component Identification Register 3 */
} PMU_Type;

/** \brief PMU Event Counter Registers (0-30) Definitions  */

#define PMU_EVCNTR_CNT_Pos                    0U                                           /*!< PMU EVCNTR: Counter Position */
#define PMU_EVCNTR_CNT_Msk                   (0xFFFFUL /*<< PMU_EVCNTRx_CNT_Pos*/)         /*!< PMU EVCNTR: Counter Mask */

/** \brief PMU Event Type and Filter Registers (0-30) Definitions  */

#define PMU_EVTYPER_EVENTTOCNT_Pos            0U                                           /*!< PMU EVTYPER: Event to Count Position */
#define PMU_EVTYPER_EVENTTOCNT_Msk           (0xFFFFUL /*<< EVTYPERx_EVENTTOCNT_Pos*/)     /*!< PMU EVTYPER: Event to Count Mask */

/** \brief PMU Count Enable Set Register Definitions */

#define PMU_CNTENSET_CNT0_ENABLE_Pos          0U                                           /*!< PMU CNTENSET: Event Counter 0 Enable Set Position */
#define PMU_CNTENSET_CNT0_ENABLE_Msk         (1UL /*<< PMU_CNTENSET_CNT0_ENABLE_Pos*/)     /*!< PMU CNTENSET: Event Counter 0 Enable Set Mask */

#define PMU_CNTENSET_CNT1_ENABLE_Pos          1U                                           /*!< PMU CNTENSET: Event Counter 1 Enable Set Position */
#define PMU_CNTENSET_CNT1_ENABLE_Msk         (1UL << PMU_CNTENSET_CNT1_ENABLE_Pos)         /*!< PMU CNTENSET: Event Counter 1 Enable Set Mask */

#define PMU_CNTENSET_CNT2_ENABLE_Pos          2U                                           /*!< PMU CNTENSET: Event Counter 2 Enable Set Position */
#define PMU_CNTENSET_CNT2_ENABLE_Msk         (1UL << PMU_CNTENSET_CNT2_ENABLE_Pos)         /*!< PMU CNTENSET: Event Counter 2 Enable Set Mask */

#define PMU_CNTENSET_CNT3_ENABLE_Pos          3U                                           /*!< PMU CNTENSET: Event Counter 3 Enable Set Position */
#define PMU_CNTENSET_CNT3_ENABLE_Msk         (1UL << PMU_CNTENSET_CNT3_ENABLE_Pos)         /*!< PMU CNTENSET: Event Counter 3 Enable Set Mask */

#define PMU_CNTENSET_CNT4_ENABLE_Pos          4U                                           /*!< PMU CNTENSET: Event Counter 4 Enable Set Position */
#define PMU_CNTENSET_CNT4_ENABLE_Msk         (1UL << PMU_CNTENSET_CNT4_ENABLE_Pos)         /*!< PMU CNTENSET: Event Counter 4 Enable Set Mask */

#define PMU_CNTENSET_CNT5_ENABLE_Pos          5U                                           /*!< PMU CNTENSET: Event Counter 5 Enable Set Position */
#define PMU_CNTENSET_CNT5_ENABLE_Msk         (1UL << PMU_CNTENSET_CNT5_ENABLE_Pos)         /*!< PMU CNTENSET: Event Counter 5 Enable Set Mask */

#define PMU_CNTENSET_CNT6_ENABLE_Pos          6U                                           /*!< PMU CNTENSET: Event Counter 6 Enable Set Position */
#define PMU_CNTENSET_CNT6_ENABLE_Msk         (1UL << PMU_CNTENSET_CNT6_ENABLE_Pos)         /*!< PMU CNTENSET: Event Counter 6 Enable Set Mask */

#define PMU_CNTENSET_CNT7_ENABLE_Pos          7U                                           /*!< PMU CNTENSET: Event Counter 7 Enable Set Position */
#define PMU_CNTENSET_CNT7_ENABLE_Msk         (1UL << PMU_CNTENSET_CNT7_ENABLE_Pos)         /*!< PMU CNTENSET: Event Counter 7 Enable Set Mask */

#define PMU_CNTENSET_CNT8_ENABLE_Pos          8U                                           /*!< PMU CNTENSET: Event Counter 8 Enable Set Position */
#define PMU_CNTENSET_CNT8_ENABLE_Msk         (1UL << PMU_CNTENSET_CNT8_ENABLE_Pos)         /*!< PMU CNTENSET: Event Counter 8 Enable Set Mask */

#define PMU_CNTENSET_CNT9_ENABLE_Pos          9U                                           /*!< PMU CNTENSET: Event Counter 9 Enable Set Position */
#define PMU_CNTENSET_CNT9_ENABLE_Msk         (1UL << PMU_CNTENSET_CNT9_ENABLE_Pos)         /*!< PMU CNTENSET: Event Counter 9 Enable Set Mask */

#define PMU_CNTENSET_CNT10_ENABLE_Pos         10U                                          /*!< PMU CNTENSET: Event Counter 10 Enable Set Position */
#define PMU_CNTENSET_CNT10_ENABLE_Msk        (1UL << PMU_CNTENSET_CNT10_ENABLE_Pos)        /*!< PMU CNTENSET: Event Counter 10 Enable Set Mask */

#define PMU_CNTENSET_CNT11_ENABLE_Pos         11U                                          /*!< PMU CNTENSET: Event Counter 11 Enable Set Position */
#define PMU_CNTENSET_CNT11_ENABLE_Msk        (1UL << PMU_CNTENSET_CNT11_ENABLE_Pos)        /*!< PMU CNTENSET: Event Counter 11 Enable Set Mask */

#define PMU_CNTENSET_CNT12_ENABLE_Pos         12U                                          /*!< PMU CNTENSET: Event Counter 12 Enable Set Position */
#define PMU_CNTENSET_CNT12_ENABLE_Msk        (1UL << PMU_CNTENSET_CNT12_ENABLE_Pos)        /*!< PMU CNTENSET: Event Counter 12 Enable Set Mask */

#define PMU_CNTENSET_CNT13_ENABLE_Pos         13U                                          /*!< PMU CNTENSET: Event Counter 13 Enable Set Position */
#define PMU_CNTENSET_CNT13_ENABLE_Msk        (1UL << PMU_CNTENSET_CNT13_ENABLE_Pos)        /*!< PMU CNTENSET: Event Counter 13 Enable Set Mask */

#define PMU_CNTENSET_CNT14_ENABLE_Pos         14U                                          /*!< PMU CNTENSET: Event Counter 14 Enable Set Position */
#define PMU_CNTENSET_CNT14_ENABLE_Msk        (1UL << PMU_CNTENSET_CNT14_ENABLE_Pos)        /*!< PMU CNTENSET: Event Counter 14 Enable Set Mask */

#define PMU_CNTENSET_CNT15_ENABLE_Pos         15U                                          /*!< PMU CNTENSET: Event Counter 15 Enable Set Position */
#define PMU_CNTENSET_CNT15_ENABLE_Msk        (1UL << PMU_CNTENSET_CNT15_ENABLE_Pos)        /*!< PMU CNTENSET: Event Counter 15 Enable Set Mask */

#define PMU_CNTENSET_CNT16_ENABLE_Pos         16U                                          /*!< PMU CNTENSET: Event Counter 16 Enable Set Position */
#define PMU_CNTENSET_CNT16_ENABLE_Msk        (1UL << PMU_CNTENSET_CNT16_ENABLE_Pos)        /*!< PMU CNTENSET: Event Counter 16 Enable Set Mask */

#define PMU_CNTENSET_CNT17_ENABLE_Pos         17U                                          /*!< PMU CNTENSET: Event Counter 17 Enable Set Position */
#define PMU_CNTENSET_CNT17_ENABLE_Msk        (1UL << PMU_CNTENSET_CNT17_ENABLE_Pos)        /*!< PMU CNTENSET: Event Counter 17 Enable Set Mask */

#define PMU_CNTENSET_CNT18_ENABLE_Pos         18U                                          /*!< PMU CNTENSET: Event Counter 18 Enable Set Position */
#define PMU_CNTENSET_CNT18_ENABLE_Msk        (1UL << PMU_CNTENSET_CNT18_ENABLE_Pos)        /*!< PMU CNTENSET: Event Counter 18 Enable Set Mask */

#define PMU_CNTENSET_CNT19_ENABLE_Pos         19U                                          /*!< PMU CNTENSET: Event Counter 19 Enable Set Position */
#define PMU_CNTENSET_CNT19_ENABLE_Msk        (1UL << PMU_CNTENSET_CNT19_ENABLE_Pos)        /*!< PMU CNTENSET: Event Counter 19 Enable Set Mask */

#define PMU_CNTENSET_CNT20_ENABLE_Pos         20U                                          /*!< PMU CNTENSET: Event Counter 20 Enable Set Position */
#define PMU_CNTENSET_CNT20_ENABLE_Msk        (1UL << PMU_CNTENSET_CNT20_ENABLE_Pos)        /*!< PMU CNTENSET: Event Counter 20 Enable Set Mask */

#define PMU_CNTENSET_CNT21_ENABLE_Pos         21U                                          /*!< PMU CNTENSET: Event Counter 21 Enable Set Position */
#define PMU_CNTENSET_CNT21_ENABLE_Msk        (1UL << PMU_CNTENSET_CNT21_ENABLE_Pos)        /*!< PMU CNTENSET: Event Counter 21 Enable Set Mask */

#define PMU_CNTENSET_CNT22_ENABLE_Pos         22U                                          /*!< PMU CNTENSET: Event Counter 22 Enable Set Position */
#define PMU_CNTENSET_CNT22_ENABLE_Msk        (1UL << PMU_CNTENSET_CNT22_ENABLE_Pos)        /*!< PMU CNTENSET: Event Counter 22 Enable Set Mask */

#define PMU_CNTENSET_CNT23_ENABLE_Pos         23U                                          /*!< PMU CNTENSET: Event Counter 23 Enable Set Position */
#define PMU_CNTENSET_CNT23_ENABLE_Msk        (1UL << PMU_CNTENSET_CNT23_ENABLE_Pos)        /*!< PMU CNTENSET: Event Counter 23 Enable Set Mask */

#define PMU_CNTENSET_CNT24_ENABLE_Pos         24U                                          /*!< PMU CNTENSET: Event Counter 24 Enable Set Position */
#define PMU_CNTENSET_CNT24_ENABLE_Msk        (1UL << PMU_CNTENSET_CNT24_ENABLE_Pos)        /*!< PMU CNTENSET: Event Counter 24 Enable Set Mask */

#define PMU_CNTENSET_CNT25_ENABLE_Pos         25U                                          /*!< PMU CNTENSET: Event Counter 25 Enable Set Position */
#define PMU_CNTENSET_CNT25_ENABLE_Msk        (1UL << PMU_CNTENSET_CNT25_ENABLE_Pos)        /*!< PMU CNTENSET: Event Counter 25 Enable Set Mask */

#define PMU_CNTENSET_CNT26_ENABLE_Pos         26U                                          /*!< PMU CNTENSET: Event Counter 26 Enable Set Position */
#define PMU_CNTENSET_CNT26_ENABLE_Msk        (1UL << PMU_CNTENSET_CNT26_ENABLE_Pos)        /*!< PMU CNTENSET: Event Counter 26 Enable Set Mask */

#define PMU_CNTENSET_CNT27_ENABLE_Pos         27U                                          /*!< PMU CNTENSET: Event Counter 27 Enable Set Position */
#define PMU_CNTENSET_CNT27_ENABLE_Msk        (1UL << PMU_CNTENSET_CNT27_ENABLE_Pos)        /*!< PMU CNTENSET: Event Counter 27 Enable Set Mask */

#define PMU_CNTENSET_CNT28_ENABLE_Pos         28U                                          /*!< PMU CNTENSET: Event Counter 28 Enable Set Position */
#define PMU_CNTENSET_CNT28_ENABLE_Msk        (1UL << PMU_CNTENSET_CNT28_ENABLE_Pos)        /*!< PMU CNTENSET: Event Counter 28 Enable Set Mask */

#define PMU_CNTENSET_CNT29_ENABLE_Pos         29U                                          /*!< PMU CNTENSET: Event Counter 29 Enable Set Position */
#define PMU_CNTENSET_CNT29_ENABLE_Msk        (1UL << PMU_CNTENSET_CNT29_ENABLE_Pos)        /*!< PMU CNTENSET: Event Counter 29 Enable Set Mask */

#define PMU_CNTENSET_CNT30_ENABLE_Pos         30U                                          /*!< PMU CNTENSET: Event Counter 30 Enable Set Position */
#define PMU_CNTENSET_CNT30_ENABLE_Msk        (1UL << PMU_CNTENSET_CNT30_ENABLE_Pos)        /*!< PMU CNTENSET: Event Counter 30 Enable Set Mask */

#define PMU_CNTENSET_CCNTR_ENABLE_Pos         31U                                          /*!< PMU CNTENSET: Cycle Counter Enable Set Position */
#define PMU_CNTENSET_CCNTR_ENABLE_Msk        (1UL << PMU_CNTENSET_CCNTR_ENABLE_Pos)        /*!< PMU CNTENSET: Cycle Counter Enable Set Mask */

/** \brief PMU Count Enable Clear Register Definitions */

#define PMU_CNTENSET_CNT0_ENABLE_Pos          0U                                           /*!< PMU CNTENCLR: Event Counter 0 Enable Clear Position */
#define PMU_CNTENCLR_CNT0_ENABLE_Msk         (1UL /*<< PMU_CNTENCLR_CNT0_ENABLE_Pos*/)     /*!< PMU CNTENCLR: Event Counter 0 Enable Clear Mask */

#define PMU_CNTENCLR_CNT1_ENABLE_Pos          1U                                           /*!< PMU CNTENCLR: Event Counter 1 Enable Clear Position */
#define PMU_CNTENCLR_CNT1_ENABLE_Msk         (1UL << PMU_CNTENCLR_CNT1_ENABLE_Pos)         /*!< PMU CNTENCLR: Event Counter 1 Enable Clear */

#define PMU_CNTENCLR_CNT2_ENABLE_Pos          2U                                           /*!< PMU CNTENCLR: Event Counter 2 Enable Clear Position */
#define PMU_CNTENCLR_CNT2_ENABLE_Msk         (1UL << PMU_CNTENCLR_CNT2_ENABLE_Pos)         /*!< PMU CNTENCLR: Event Counter 2 Enable Clear Mask */

#define PMU_CNTENCLR_CNT3_ENABLE_Pos          3U                                           /*!< PMU CNTENCLR: Event Counter 3 Enable Clear Position */
#define PMU_CNTENCLR_CNT3_ENABLE_Msk         (1UL << PMU_CNTENCLR_CNT3_ENABLE_Pos)         /*!< PMU CNTENCLR: Event Counter 3 Enable Clear Mask */

#define PMU_CNTENCLR_CNT4_ENABLE_Pos          4U                                           /*!< PMU CNTENCLR: Event Counter 4 Enable Clear Position */
#define PMU_CNTENCLR_CNT4_ENABLE_Msk         (1UL << PMU_CNTENCLR_CNT4_ENABLE_Pos)         /*!< PMU CNTENCLR: Event Counter 4 Enable Clear Mask */

#define PMU_CNTENCLR_CNT5_ENABLE_Pos          5U                                           /*!< PMU CNTENCLR: Event Counter 5 Enable Clear Position */
#define PMU_CNTENCLR_CNT5_ENABLE_Msk         (1UL << PMU_CNTENCLR_CNT5_ENABLE_Pos)         /*!< PMU CNTENCLR: Event Counter 5 Enable Clear Mask */

#define PMU_CNTENCLR_CNT6_ENABLE_Pos          6U                                           /*!< PMU CNTENCLR: Event Counter 6 Enable Clear Position */
#define PMU_CNTENCLR_CNT6_ENABLE_Msk         (1UL << PMU_CNTENCLR_CNT6_ENABLE_Pos)         /*!< PMU CNTENCLR: Event Counter 6 Enable Clear Mask */

#define PMU_CNTENCLR_CNT7_ENABLE_Pos          7U                                           /*!< PMU CNTENCLR: Event Counter 7 Enable Clear Position */
#define PMU_CNTENCLR_CNT7_ENABLE_Msk         (1UL << PMU_CNTENCLR_CNT7_ENABLE_Pos)         /*!< PMU CNTENCLR: Event Counter 7 Enable Clear Mask */

#define PMU_CNTENCLR_CNT8_ENABLE_Pos          8U                                           /*!< PMU CNTENCLR: Event Counter 8 Enable Clear Position */
#define PMU_CNTENCLR_CNT8_ENABLE_Msk         (1UL << PMU_CNTENCLR_CNT8_ENABLE_Pos)         /*!< PMU CNTENCLR: Event Counter 8 Enable Clear Mask */

#define PMU_CNTENCLR_CNT9_ENABLE_Pos          9U                                           /*!< PMU CNTENCLR: Event Counter 9 Enable Clear Position */
#define PMU_CNTENCLR_CNT9_ENABLE_Msk         (1UL << PMU_CNTENCLR_CNT9_ENABLE_Pos)         /*!< PMU CNTENCLR: Event Counter 9 Enable Clear Mask */

#define PMU_CNTENCLR_CNT10_ENABLE_Pos         10U                                          /*!< PMU CNTENCLR: Event Counter 10 Enable Clear Position */
#define PMU_CNTENCLR_CNT10_ENABLE_Msk        (1UL << PMU_CNTENCLR_CNT10_ENABLE_Pos)        /*!< PMU CNTENCLR: Event Counter 10 Enable Clear Mask */

#define PMU_CNTENCLR_CNT11_ENABLE_Pos         11U                                          /*!< PMU CNTENCLR: Event Counter 11 Enable Clear Position */
#define PMU_CNTENCLR_CNT11_ENABLE_Msk        (1UL << PMU_CNTENCLR_CNT11_ENABLE_Pos)        /*!< PMU CNTENCLR: Event Counter 11 Enable Clear Mask */

#define PMU_CNTENCLR_CNT12_ENABLE_Pos         12U                                          /*!< PMU CNTENCLR: Event Counter 12 Enable Clear Position */
#define PMU_CNTENCLR_CNT12_ENABLE_Msk        (1UL << PMU_CNTENCLR_CNT12_ENABLE_Pos)        /*!< PMU CNTENCLR: Event Counter 12 Enable Clear Mask */

#define PMU_CNTENCLR_CNT13_ENABLE_Pos         13U                                          /*!< PMU CNTENCLR: Event Counter 13 Enable Clear Position */
#define PMU_CNTENCLR_CNT13_ENABLE_Msk        (1UL << PMU_CNTENCLR_CNT13_ENABLE_Pos)        /*!< PMU CNTENCLR: Event Counter 13 Enable Clear Mask */

#define PMU_CNTENCLR_CNT14_ENABLE_Pos         14U                                          /*!< PMU CNTENCLR: Event Counter 14 Enable Clear Position */
#define PMU_CNTENCLR_CNT14_ENABLE_Msk        (1UL << PMU_CNTENCLR_CNT14_ENABLE_Pos)        /*!< PMU CNTENCLR: Event Counter 14 Enable Clear Mask */

#define PMU_CNTENCLR_CNT15_ENABLE_Pos         15U                                          /*!< PMU CNTENCLR: Event Counter 15 Enable Clear Position */
#define PMU_CNTENCLR_CNT15_ENABLE_Msk        (1UL << PMU_CNTENCLR_CNT15_ENABLE_Pos)        /*!< PMU CNTENCLR: Event Counter 15 Enable Clear Mask */

#define PMU_CNTENCLR_CNT16_ENABLE_Pos         16U                                          /*!< PMU CNTENCLR: Event Counter 16 Enable Clear Position */
#define PMU_CNTENCLR_CNT16_ENABLE_Msk        (1UL << PMU_CNTENCLR_CNT16_ENABLE_Pos)        /*!< PMU CNTENCLR: Event Counter 16 Enable Clear Mask */

#define PMU_CNTENCLR_CNT17_ENABLE_Pos         17U                                          /*!< PMU CNTENCLR: Event Counter 17 Enable Clear Position */
#define PMU_CNTENCLR_CNT17_ENABLE_Msk        (1UL << PMU_CNTENCLR_CNT17_ENABLE_Pos)        /*!< PMU CNTENCLR: Event Counter 17 Enable Clear Mask */

#define PMU_CNTENCLR_CNT18_ENABLE_Pos         18U                                          /*!< PMU CNTENCLR: Event Counter 18 Enable Clear Position */
#define PMU_CNTENCLR_CNT18_ENABLE_Msk        (1UL << PMU_CNTENCLR_CNT18_ENABLE_Pos)        /*!< PMU CNTENCLR: Event Counter 18 Enable Clear Mask */

#define PMU_CNTENCLR_CNT19_ENABLE_Pos         19U                                          /*!< PMU CNTENCLR: Event Counter 19 Enable Clear Position */
#define PMU_CNTENCLR_CNT19_ENABLE_Msk        (1UL << PMU_CNTENCLR_CNT19_ENABLE_Pos)        /*!< PMU CNTENCLR: Event Counter 19 Enable Clear Mask */

#define PMU_CNTENCLR_CNT20_ENABLE_Pos         20U                                          /*!< PMU CNTENCLR: Event Counter 20 Enable Clear Position */
#define PMU_CNTENCLR_CNT20_ENABLE_Msk        (1UL << PMU_CNTENCLR_CNT20_ENABLE_Pos)        /*!< PMU CNTENCLR: Event Counter 20 Enable Clear Mask */

#define PMU_CNTENCLR_CNT21_ENABLE_Pos         21U                                          /*!< PMU CNTENCLR: Event Counter 21 Enable Clear Position */
#define PMU_CNTENCLR_CNT21_ENABLE_Msk        (1UL << PMU_CNTENCLR_CNT21_ENABLE_Pos)        /*!< PMU CNTENCLR: Event Counter 21 Enable Clear Mask */

#define PMU_CNTENCLR_CNT22_ENABLE_Pos         22U                                          /*!< PMU CNTENCLR: Event Counter 22 Enable Clear Position */
#define PMU_CNTENCLR_CNT22_ENABLE_Msk        (1UL << PMU_CNTENCLR_CNT22_ENABLE_Pos)        /*!< PMU CNTENCLR: Event Counter 22 Enable Clear Mask */

#define PMU_CNTENCLR_CNT23_ENABLE_Pos         23U                                          /*!< PMU CNTENCLR: Event Counter 23 Enable Clear Position */
#define PMU_CNTENCLR_CNT23_ENABLE_Msk        (1UL << PMU_CNTENCLR_CNT23_ENABLE_Pos)        /*!< PMU CNTENCLR: Event Counter 23 Enable Clear Mask */

#define PMU_CNTENCLR_CNT24_ENABLE_Pos         24U                                          /*!< PMU CNTENCLR: Event Counter 24 Enable Clear Position */
#define PMU_CNTENCLR_CNT24_ENABLE_Msk        (1UL << PMU_CNTENCLR_CNT24_ENABLE_Pos)        /*!< PMU CNTENCLR: Event Counter 24 Enable Clear Mask */

#define PMU_CNTENCLR_CNT25_ENABLE_Pos         25U                                          /*!< PMU CNTENCLR: Event Counter 25 Enable Clear Position */
#define PMU_CNTENCLR_CNT25_ENABLE_Msk        (1UL << PMU_CNTENCLR_CNT25_ENABLE_Pos)        /*!< PMU CNTENCLR: Event Counter 25 Enable Clear Mask */

#define PMU_CNTENCLR_CNT26_ENABLE_Pos         26U                                          /*!< PMU CNTENCLR: Event Counter 26 Enable Clear Position */
#define PMU_CNTENCLR_CNT26_ENABLE_Msk        (1UL << PMU_CNTENCLR_CNT26_ENABLE_Pos)        /*!< PMU CNTENCLR: Event Counter 26 Enable Clear Mask */

#define PMU_CNTENCLR_CNT27_ENABLE_Pos         27U                                          /*!< PMU CNTENCLR: Event Counter 27 Enable Clear Position */
#define PMU_CNTENCLR_CNT27_ENABLE_Msk        (1UL << PMU_CNTENCLR_CNT27_ENABLE_Pos)        /*!< PMU CNTENCLR: Event Counter 27 Enable Clear Mask */

#define PMU_CNTENCLR_CNT28_ENABLE_Pos         28U                                          /*!< PMU CNTENCLR: Event Counter 28 Enable Clear Position */
#define PMU_CNTENCLR_CNT28_ENABLE_Msk        (1UL << PMU_CNTENCLR_CNT28_ENABLE_Pos)        /*!< PMU CNTENCLR: Event Counter 28 Enable Clear Mask */

#define PMU_CNTENCLR_CNT29_ENABLE_Pos         29U                                          /*!< PMU CNTENCLR: Event Counter 29 Enable Clear Position */
#define PMU_CNTENCLR_CNT29_ENABLE_Msk        (1UL << PMU_CNTENCLR_CNT29_ENABLE_Pos)        /*!< PMU CNTENCLR: Event Counter 29 Enable Clear Mask */

#define PMU_CNTENCLR_CNT30_ENABLE_Pos         30U                                          /*!< PMU CNTENCLR: Event Counter 30 Enable Clear Position */
#define PMU_CNTENCLR_CNT30_ENABLE_Msk        (1UL << PMU_CNTENCLR_CNT30_ENABLE_Pos)        /*!< PMU CNTENCLR: Event Counter 30 Enable Clear Mask */

#define PMU_CNTENCLR_CCNTR_ENABLE_Pos         31U                                          /*!< PMU CNTENCLR: Cycle Counter Enable Clear Position */
#define PMU_CNTENCLR_CCNTR_ENABLE_Msk        (1UL << PMU_CNTENCLR_CCNTR_ENABLE_Pos)        /*!< PMU CNTENCLR: Cycle Counter Enable Clear Mask */

/** \brief PMU Interrupt Enable Set Register Definitions */

#define PMU_INTENSET_CNT0_ENABLE_Pos          0U                                           /*!< PMU INTENSET: Event Counter 0 Interrupt Enable Set Position */
#define PMU_INTENSET_CNT0_ENABLE_Msk         (1UL /*<< PMU_INTENSET_CNT0_ENABLE_Pos*/)     /*!< PMU INTENSET: Event Counter 0 Interrupt Enable Set Mask */

#define PMU_INTENSET_CNT1_ENABLE_Pos          1U                                           /*!< PMU INTENSET: Event Counter 1 Interrupt Enable Set Position */
#define PMU_INTENSET_CNT1_ENABLE_Msk         (1UL << PMU_INTENSET_CNT1_ENABLE_Pos)         /*!< PMU INTENSET: Event Counter 1 Interrupt Enable Set Mask */

#define PMU_INTENSET_CNT2_ENABLE_Pos          2U                                           /*!< PMU INTENSET: Event Counter 2 Interrupt Enable Set Position */
#define PMU_INTENSET_CNT2_ENABLE_Msk         (1UL << PMU_INTENSET_CNT2_ENABLE_Pos)         /*!< PMU INTENSET: Event Counter 2 Interrupt Enable Set Mask */

#define PMU_INTENSET_CNT3_ENABLE_Pos          3U                                           /*!< PMU INTENSET: Event Counter 3 Interrupt Enable Set Position */
#define PMU_INTENSET_CNT3_ENABLE_Msk         (1UL << PMU_INTENSET_CNT3_ENABLE_Pos)         /*!< PMU INTENSET: Event Counter 3 Interrupt Enable Set Mask */

#define PMU_INTENSET_CNT4_ENABLE_Pos          4U                                           /*!< PMU INTENSET: Event Counter 4 Interrupt Enable Set Position */
#define PMU_INTENSET_CNT4_ENABLE_Msk         (1UL << PMU_INTENSET_CNT4_ENABLE_Pos)         /*!< PMU INTENSET: Event Counter 4 Interrupt Enable Set Mask */

#define PMU_INTENSET_CNT5_ENABLE_Pos          5U                                           /*!< PMU INTENSET: Event Counter 5 Interrupt Enable Set Position */
#define PMU_INTENSET_CNT5_ENABLE_Msk         (1UL << PMU_INTENSET_CNT5_ENABLE_Pos)         /*!< PMU INTENSET: Event Counter 5 Interrupt Enable Set Mask */

#define PMU_INTENSET_CNT6_ENABLE_Pos          6U                                           /*!< PMU INTENSET: Event Counter 6 Interrupt Enable Set Position */
#define PMU_INTENSET_CNT6_ENABLE_Msk         (1UL << PMU_INTENSET_CNT6_ENABLE_Pos)         /*!< PMU INTENSET: Event Counter 6 Interrupt Enable Set Mask */

#define PMU_INTENSET_CNT7_ENABLE_Pos          7U                                           /*!< PMU INTENSET: Event Counter 7 Interrupt Enable Set Position */
#define PMU_INTENSET_CNT7_ENABLE_Msk         (1UL << PMU_INTENSET_CNT7_ENABLE_Pos)         /*!< PMU INTENSET: Event Counter 7 Interrupt Enable Set Mask */

#define PMU_INTENSET_CNT8_ENABLE_Pos          8U                                           /*!< PMU INTENSET: Event Counter 8 Interrupt Enable Set Position */
#define PMU_INTENSET_CNT8_ENABLE_Msk         (1UL << PMU_INTENSET_CNT8_ENABLE_Pos)         /*!< PMU INTENSET: Event Counter 8 Interrupt Enable Set Mask */

#define PMU_INTENSET_CNT9_ENABLE_Pos          9U                                           /*!< PMU INTENSET: Event Counter 9 Interrupt Enable Set Position */
#define PMU_INTENSET_CNT9_ENABLE_Msk         (1UL << PMU_INTENSET_CNT9_ENABLE_Pos)         /*!< PMU INTENSET: Event Counter 9 Interrupt Enable Set Mask */

#define PMU_INTENSET_CNT10_ENABLE_Pos         10U                                          /*!< PMU INTENSET: Event Counter 10 Interrupt Enable Set Position */
#define PMU_INTENSET_CNT10_ENABLE_Msk        (1UL << PMU_INTENSET_CNT10_ENABLE_Pos)        /*!< PMU INTENSET: Event Counter 10 Interrupt Enable Set Mask */

#define PMU_INTENSET_CNT11_ENABLE_Pos         11U                                          /*!< PMU INTENSET: Event Counter 11 Interrupt Enable Set Position */
#define PMU_INTENSET_CNT11_ENABLE_Msk        (1UL << PMU_INTENSET_CNT11_ENABLE_Pos)        /*!< PMU INTENSET: Event Counter 11 Interrupt Enable Set Mask */

#define PMU_INTENSET_CNT12_ENABLE_Pos         12U                                          /*!< PMU INTENSET: Event Counter 12 Interrupt Enable Set Position */
#define PMU_INTENSET_CNT12_ENABLE_Msk        (1UL << PMU_INTENSET_CNT12_ENABLE_Pos)        /*!< PMU INTENSET: Event Counter 12 Interrupt Enable Set Mask */

#define PMU_INTENSET_CNT13_ENABLE_Pos         13U                                          /*!< PMU INTENSET: Event Counter 13 Interrupt Enable Set Position */
#define PMU_INTENSET_CNT13_ENABLE_Msk        (1UL << PMU_INTENSET_CNT13_ENABLE_Pos)        /*!< PMU INTENSET: Event Counter 13 Interrupt Enable Set Mask */

#define PMU_INTENSET_CNT14_ENABLE_Pos         14U                                          /*!< PMU INTENSET: Event Counter 14 Interrupt Enable Set Position */
#define PMU_INTENSET_CNT14_ENABLE_Msk        (1UL << PMU_INTENSET_CNT14_ENABLE_Pos)        /*!< PMU INTENSET: Event Counter 14 Interrupt Enable Set Mask */

#define PMU_INTENSET_CNT15_ENABLE_Pos         15U                                          /*!< PMU INTENSET: Event Counter 15 Interrupt Enable Set Position */
#define PMU_INTENSET_CNT15_ENABLE_Msk        (1UL << PMU_INTENSET_CNT15_ENABLE_Pos)        /*!< PMU INTENSET: Event Counter 15 Interrupt Enable Set Mask */

#define PMU_INTENSET_CNT16_ENABLE_Pos         16U                                          /*!< PMU INTENSET: Event Counter 16 Interrupt Enable Set Position */
#define PMU_INTENSET_CNT16_ENABLE_Msk        (1UL << PMU_INTENSET_CNT16_ENABLE_Pos)        /*!< PMU INTENSET: Event Counter 16 Interrupt Enable Set Mask */

#define PMU_INTENSET_CNT17_ENABLE_Pos         17U                                          /*!< PMU INTENSET: Event Counter 17 Interrupt Enable Set Position */
#define PMU_INTENSET_CNT17_ENABLE_Msk        (1UL << PMU_INTENSET_CNT17_ENABLE_Pos)        /*!< PMU INTENSET: Event Counter 17 Interrupt Enable Set Mask */

#define PMU_INTENSET_CNT18_ENABLE_Pos         18U                                          /*!< PMU INTENSET: Event Counter 18 Interrupt Enable Set Position */
#define PMU_INTENSET_CNT18_ENABLE_Msk        (1UL << PMU_INTENSET_CNT18_ENABLE_Pos)        /*!< PMU INTENSET: Event Counter 18 Interrupt Enable Set Mask */

#define PMU_INTENSET_CNT19_ENABLE_Pos         19U                                          /*!< PMU INTENSET: Event Counter 19 Interrupt Enable Set Position */
#define PMU_INTENSET_CNT19_ENABLE_Msk        (1UL << PMU_INTENSET_CNT19_ENABLE_Pos)        /*!< PMU INTENSET: Event Counter 19 Interrupt Enable Set Mask */

#define PMU_INTENSET_CNT20_ENABLE_Pos         20U                                          /*!< PMU INTENSET: Event Counter 20 Interrupt Enable Set Position */
#define PMU_INTENSET_CNT20_ENABLE_Msk        (1UL << PMU_INTENSET_CNT20_ENABLE_Pos)        /*!< PMU INTENSET: Event Counter 20 Interrupt Enable Set Mask */

#define PMU_INTENSET_CNT21_ENABLE_Pos         21U                                          /*!< PMU INTENSET: Event Counter 21 Interrupt Enable Set Position */
#define PMU_INTENSET_CNT21_ENABLE_Msk        (1UL << PMU_INTENSET_CNT21_ENABLE_Pos)        /*!< PMU INTENSET: Event Counter 21 Interrupt Enable Set Mask */

#define PMU_INTENSET_CNT22_ENABLE_Pos         22U                                          /*!< PMU INTENSET: Event Counter 22 Interrupt Enable Set Position */
#define PMU_INTENSET_CNT22_ENABLE_Msk        (1UL << PMU_INTENSET_CNT22_ENABLE_Pos)        /*!< PMU INTENSET: Event Counter 22 Interrupt Enable Set Mask */

#define PMU_INTENSET_CNT23_ENABLE_Pos         23U                                          /*!< PMU INTENSET: Event Counter 23 Interrupt Enable Set Position */
#define PMU_INTENSET_CNT23_ENABLE_Msk        (1UL << PMU_INTENSET_CNT23_ENABLE_Pos)        /*!< PMU INTENSET: Event Counter 23 Interrupt Enable Set Mask */

#define PMU_INTENSET_CNT24_ENABLE_Pos         24U                                          /*!< PMU INTENSET: Event Counter 24 Interrupt Enable Set Position */
#define PMU_INTENSET_CNT24_ENABLE_Msk        (1UL << PMU_INTENSET_CNT24_ENABLE_Pos)        /*!< PMU INTENSET: Event Counter 24 Interrupt Enable Set Mask */

#define PMU_INTENSET_CNT25_ENABLE_Pos         25U                                          /*!< PMU INTENSET: Event Counter 25 Interrupt Enable Set Position */
#define PMU_INTENSET_CNT25_ENABLE_Msk        (1UL << PMU_INTENSET_CNT25_ENABLE_Pos)        /*!< PMU INTENSET: Event Counter 25 Interrupt Enable Set Mask */

#define PMU_INTENSET_CNT26_ENABLE_Pos         26U                                          /*!< PMU INTENSET: Event Counter 26 Interrupt Enable Set Position */
#define PMU_INTENSET_CNT26_ENABLE_Msk        (1UL << PMU_INTENSET_CNT26_ENABLE_Pos)        /*!< PMU INTENSET: Event Counter 26 Interrupt Enable Set Mask */

#define PMU_INTENSET_CNT27_ENABLE_Pos         27U                                          /*!< PMU INTENSET: Event Counter 27 Interrupt Enable Set Position */
#define PMU_INTENSET_CNT27_ENABLE_Msk        (1UL << PMU_INTENSET_CNT27_ENABLE_Pos)        /*!< PMU INTENSET: Event Counter 27 Interrupt Enable Set Mask */

#define PMU_INTENSET_CNT28_ENABLE_Pos         28U                                          /*!< PMU INTENSET: Event Counter 28 Interrupt Enable Set Position */
#define PMU_INTENSET_CNT28_ENABLE_Msk        (1UL << PMU_INTENSET_CNT28_ENABLE_Pos)        /*!< PMU INTENSET: Event Counter 28 Interrupt Enable Set Mask */

#define PMU_INTENSET_CNT29_ENABLE_Pos         29U                                          /*!< PMU INTENSET: Event Counter 29 Interrupt Enable Set Position */
#define PMU_INTENSET_CNT29_ENABLE_Msk        (1UL << PMU_INTENSET_CNT29_ENABLE_Pos)        /*!< PMU INTENSET: Event Counter 29 Interrupt Enable Set Mask */

#define PMU_INTENSET_CNT30_ENABLE_Pos         30U                                          /*!< PMU INTENSET: Event Counter 30 Interrupt Enable Set Position */
#define PMU_INTENSET_CNT30_ENABLE_Msk        (1UL << PMU_INTENSET_CNT30_ENABLE_Pos)        /*!< PMU INTENSET: Event Counter 30 Interrupt Enable Set Mask */

#define PMU_INTENSET_CYCCNT_ENABLE_Pos        31U                                          /*!< PMU INTENSET: Cycle Counter Interrupt Enable Set Position */
#define PMU_INTENSET_CCYCNT_ENABLE_Msk       (1UL << PMU_INTENSET_CYCCNT_ENABLE_Pos)       /*!< PMU INTENSET: Cycle Counter Interrupt Enable Set Mask */

/** \brief PMU Interrupt Enable Clear Register Definitions */

#define PMU_INTENSET_CNT0_ENABLE_Pos          0U                                           /*!< PMU INTENCLR: Event Counter 0 Interrupt Enable Clear Position */
#define PMU_INTENCLR_CNT0_ENABLE_Msk         (1UL /*<< PMU_INTENCLR_CNT0_ENABLE_Pos*/)     /*!< PMU INTENCLR: Event Counter 0 Interrupt Enable Clear Mask */

#define PMU_INTENCLR_CNT1_ENABLE_Pos          1U                                           /*!< PMU INTENCLR: Event Counter 1 Interrupt Enable Clear Position */
#define PMU_INTENCLR_CNT1_ENABLE_Msk         (1UL << PMU_INTENCLR_CNT1_ENABLE_Pos)         /*!< PMU INTENCLR: Event Counter 1 Interrupt Enable Clear */

#define PMU_INTENCLR_CNT2_ENABLE_Pos          2U                                           /*!< PMU INTENCLR: Event Counter 2 Interrupt Enable Clear Position */
#define PMU_INTENCLR_CNT2_ENABLE_Msk         (1UL << PMU_INTENCLR_CNT2_ENABLE_Pos)         /*!< PMU INTENCLR: Event Counter 2 Interrupt Enable Clear Mask */

#define PMU_INTENCLR_CNT3_ENABLE_Pos          3U                                           /*!< PMU INTENCLR: Event Counter 3 Interrupt Enable Clear Position */
#define PMU_INTENCLR_CNT3_ENABLE_Msk         (1UL << PMU_INTENCLR_CNT3_ENABLE_Pos)         /*!< PMU INTENCLR: Event Counter 3 Interrupt Enable Clear Mask */

#define PMU_INTENCLR_CNT4_ENABLE_Pos          4U                                           /*!< PMU INTENCLR: Event Counter 4 Interrupt Enable Clear Position */
#define PMU_INTENCLR_CNT4_ENABLE_Msk         (1UL << PMU_INTENCLR_CNT4_ENABLE_Pos)         /*!< PMU INTENCLR: Event Counter 4 Interrupt Enable Clear Mask */

#define PMU_INTENCLR_CNT5_ENABLE_Pos          5U                                           /*!< PMU INTENCLR: Event Counter 5 Interrupt Enable Clear Position */
#define PMU_INTENCLR_CNT5_ENABLE_Msk         (1UL << PMU_INTENCLR_CNT5_ENABLE_Pos)         /*!< PMU INTENCLR: Event Counter 5 Interrupt Enable Clear Mask */

#define PMU_INTENCLR_CNT6_ENABLE_Pos          6U                                           /*!< PMU INTENCLR: Event Counter 6 Interrupt Enable Clear Position */
#define PMU_INTENCLR_CNT6_ENABLE_Msk         (1UL << PMU_INTENCLR_CNT6_ENABLE_Pos)         /*!< PMU INTENCLR: Event Counter 6 Interrupt Enable Clear Mask */

#define PMU_INTENCLR_CNT7_ENABLE_Pos          7U                                           /*!< PMU INTENCLR: Event Counter 7 Interrupt Enable Clear Position */
#define PMU_INTENCLR_CNT7_ENABLE_Msk         (1UL << PMU_INTENCLR_CNT7_ENABLE_Pos)         /*!< PMU INTENCLR: Event Counter 7 Interrupt Enable Clear Mask */

#define PMU_INTENCLR_CNT8_ENABLE_Pos          8U                                           /*!< PMU INTENCLR: Event Counter 8 Interrupt Enable Clear Position */
#define PMU_INTENCLR_CNT8_ENABLE_Msk         (1UL << PMU_INTENCLR_CNT8_ENABLE_Pos)         /*!< PMU INTENCLR: Event Counter 8 Interrupt Enable Clear Mask */

#define PMU_INTENCLR_CNT9_ENABLE_Pos          9U                                           /*!< PMU INTENCLR: Event Counter 9 Interrupt Enable Clear Position */
#define PMU_INTENCLR_CNT9_ENABLE_Msk         (1UL << PMU_INTENCLR_CNT9_ENABLE_Pos)         /*!< PMU INTENCLR: Event Counter 9 Interrupt Enable Clear Mask */

#define PMU_INTENCLR_CNT10_ENABLE_Pos         10U                                          /*!< PMU INTENCLR: Event Counter 10 Interrupt Enable Clear Position */
#define PMU_INTENCLR_CNT10_ENABLE_Msk        (1UL << PMU_INTENCLR_CNT10_ENABLE_Pos)        /*!< PMU INTENCLR: Event Counter 10 Interrupt Enable Clear Mask */

#define PMU_INTENCLR_CNT11_ENABLE_Pos         11U                                          /*!< PMU INTENCLR: Event Counter 11 Interrupt Enable Clear Position */
#define PMU_INTENCLR_CNT11_ENABLE_Msk        (1UL << PMU_INTENCLR_CNT11_ENABLE_Pos)        /*!< PMU INTENCLR: Event Counter 11 Interrupt Enable Clear Mask */

#define PMU_INTENCLR_CNT12_ENABLE_Pos         12U                                          /*!< PMU INTENCLR: Event Counter 12 Interrupt Enable Clear Position */
#define PMU_INTENCLR_CNT12_ENABLE_Msk        (1UL << PMU_INTENCLR_CNT12_ENABLE_Pos)        /*!< PMU INTENCLR: Event Counter 12 Interrupt Enable Clear Mask */

#define PMU_INTENCLR_CNT13_ENABLE_Pos         13U                                          /*!< PMU INTENCLR: Event Counter 13 Interrupt Enable Clear Position */
#define PMU_INTENCLR_CNT13_ENABLE_Msk        (1UL << PMU_INTENCLR_CNT13_ENABLE_Pos)        /*!< PMU INTENCLR: Event Counter 13 Interrupt Enable Clear Mask */

#define PMU_INTENCLR_CNT14_ENABLE_Pos         14U                                          /*!< PMU INTENCLR: Event Counter 14 Interrupt Enable Clear Position */
#define PMU_INTENCLR_CNT14_ENABLE_Msk        (1UL << PMU_INTENCLR_CNT14_ENABLE_Pos)        /*!< PMU INTENCLR: Event Counter 14 Interrupt Enable Clear Mask */

#define PMU_INTENCLR_CNT15_ENABLE_Pos         15U                                          /*!< PMU INTENCLR: Event Counter 15 Interrupt Enable Clear Position */
#define PMU_INTENCLR_CNT15_ENABLE_Msk        (1UL << PMU_INTENCLR_CNT15_ENABLE_Pos)        /*!< PMU INTENCLR: Event Counter 15 Interrupt Enable Clear Mask */

#define PMU_INTENCLR_CNT16_ENABLE_Pos         16U                                          /*!< PMU INTENCLR: Event Counter 16 Interrupt Enable Clear Position */
#define PMU_INTENCLR_CNT16_ENABLE_Msk        (1UL << PMU_INTENCLR_CNT16_ENABLE_Pos)        /*!< PMU INTENCLR: Event Counter 16 Interrupt Enable Clear Mask */

#define PMU_INTENCLR_CNT17_ENABLE_Pos         17U                                          /*!< PMU INTENCLR: Event Counter 17 Interrupt Enable Clear Position */
#define PMU_INTENCLR_CNT17_ENABLE_Msk        (1UL << PMU_INTENCLR_CNT17_ENABLE_Pos)        /*!< PMU INTENCLR: Event Counter 17 Interrupt Enable Clear Mask */

#define PMU_INTENCLR_CNT18_ENABLE_Pos         18U                                          /*!< PMU INTENCLR: Event Counter 18 Interrupt Enable Clear Position */
#define PMU_INTENCLR_CNT18_ENABLE_Msk        (1UL << PMU_INTENCLR_CNT18_ENABLE_Pos)        /*!< PMU INTENCLR: Event Counter 18 Interrupt Enable Clear Mask */

#define PMU_INTENCLR_CNT19_ENABLE_Pos         19U                                          /*!< PMU INTENCLR: Event Counter 19 Interrupt Enable Clear Position */
#define PMU_INTENCLR_CNT19_ENABLE_Msk        (1UL << PMU_INTENCLR_CNT19_ENABLE_Pos)        /*!< PMU INTENCLR: Event Counter 19 Interrupt Enable Clear Mask */

#define PMU_INTENCLR_CNT20_ENABLE_Pos         20U                                          /*!< PMU INTENCLR: Event Counter 20 Interrupt Enable Clear Position */
#define PMU_INTENCLR_CNT20_ENABLE_Msk        (1UL << PMU_INTENCLR_CNT20_ENABLE_Pos)        /*!< PMU INTENCLR: Event Counter 20 Interrupt Enable Clear Mask */

#define PMU_INTENCLR_CNT21_ENABLE_Pos         21U                                          /*!< PMU INTENCLR: Event Counter 21 Interrupt Enable Clear Position */
#define PMU_INTENCLR_CNT21_ENABLE_Msk        (1UL << PMU_INTENCLR_CNT21_ENABLE_Pos)        /*!< PMU INTENCLR: Event Counter 21 Interrupt Enable Clear Mask */

#define PMU_INTENCLR_CNT22_ENABLE_Pos         22U                                          /*!< PMU INTENCLR: Event Counter 22 Interrupt Enable Clear Position */
#define PMU_INTENCLR_CNT22_ENABLE_Msk        (1UL << PMU_INTENCLR_CNT22_ENABLE_Pos)        /*!< PMU INTENCLR: Event Counter 22 Interrupt Enable Clear Mask */

#define PMU_INTENCLR_CNT23_ENABLE_Pos         23U                                          /*!< PMU INTENCLR: Event Counter 23 Interrupt Enable Clear Position */
#define PMU_INTENCLR_CNT23_ENABLE_Msk        (1UL << PMU_INTENCLR_CNT23_ENABLE_Pos)        /*!< PMU INTENCLR: Event Counter 23 Interrupt Enable Clear Mask */

#define PMU_INTENCLR_CNT24_ENABLE_Pos         24U                                          /*!< PMU INTENCLR: Event Counter 24 Interrupt Enable Clear Position */
#define PMU_INTENCLR_CNT24_ENABLE_Msk        (1UL << PMU_INTENCLR_CNT24_ENABLE_Pos)        /*!< PMU INTENCLR: Event Counter 24 Interrupt Enable Clear Mask */

#define PMU_INTENCLR_CNT25_ENABLE_Pos         25U                                          /*!< PMU INTENCLR: Event Counter 25 Interrupt Enable Clear Position */
#define PMU_INTENCLR_CNT25_ENABLE_Msk        (1UL << PMU_INTENCLR_CNT25_ENABLE_Pos)        /*!< PMU INTENCLR: Event Counter 25 Interrupt Enable Clear Mask */

#define PMU_INTENCLR_CNT26_ENABLE_Pos         26U                                          /*!< PMU INTENCLR: Event Counter 26 Interrupt Enable Clear Position */
#define PMU_INTENCLR_CNT26_ENABLE_Msk        (1UL << PMU_INTENCLR_CNT26_ENABLE_Pos)        /*!< PMU INTENCLR: Event Counter 26 Interrupt Enable Clear Mask */

#define PMU_INTENCLR_CNT27_ENABLE_Pos         27U                                          /*!< PMU INTENCLR: Event Counter 27 Interrupt Enable Clear Position */
#define PMU_INTENCLR_CNT27_ENABLE_Msk        (1UL << PMU_INTENCLR_CNT27_ENABLE_Pos)        /*!< PMU INTENCLR: Event Counter 27 Interrupt Enable Clear Mask */

#define PMU_INTENCLR_CNT28_ENABLE_Pos         28U                                          /*!< PMU INTENCLR: Event Counter 28 Interrupt Enable Clear Position */
#define PMU_INTENCLR_CNT28_ENABLE_Msk        (1UL << PMU_INTENCLR_CNT28_ENABLE_Pos)        /*!< PMU INTENCLR: Event Counter 28 Interrupt Enable Clear Mask */

#define PMU_INTENCLR_CNT29_ENABLE_Pos         29U                                          /*!< PMU INTENCLR: Event Counter 29 Interrupt Enable Clear Position */
#define PMU_INTENCLR_CNT29_ENABLE_Msk        (1UL << PMU_INTENCLR_CNT29_ENABLE_Pos)        /*!< PMU INTENCLR: Event Counter 29 Interrupt Enable Clear Mask */

#define PMU_INTENCLR_CNT30_ENABLE_Pos         30U                                          /*!< PMU INTENCLR: Event Counter 30 Interrupt Enable Clear Position */
#define PMU_INTENCLR_CNT30_ENABLE_Msk        (1UL << PMU_INTENCLR_CNT30_ENABLE_Pos)        /*!< PMU INTENCLR: Event Counter 30 Interrupt Enable Clear Mask */

#define PMU_INTENCLR_CYCCNT_ENABLE_Pos        31U                                          /*!< PMU INTENCLR: Cycle Counter Interrupt Enable Clear Position */
#define PMU_INTENCLR_CYCCNT_ENABLE_Msk       (1UL << PMU_INTENCLR_CYCCNT_ENABLE_Pos)       /*!< PMU INTENCLR: Cycle Counter Interrupt Enable Clear Mask */

/** \brief PMU Overflow Flag Status Set Register Definitions */

#define PMU_OVSSET_CNT0_STATUS_Pos            0U                                           /*!< PMU OVSSET: Event Counter 0 Overflow Set Position */
#define PMU_OVSSET_CNT0_STATUS_Msk           (1UL /*<< PMU_OVSSET_CNT0_STATUS_Pos*/)       /*!< PMU OVSSET: Event Counter 0 Overflow Set Mask */

#define PMU_OVSSET_CNT1_STATUS_Pos            1U                                           /*!< PMU OVSSET: Event Counter 1 Overflow Set Position */
#define PMU_OVSSET_CNT1_STATUS_Msk           (1UL << PMU_OVSSET_CNT1_STATUS_Pos)           /*!< PMU OVSSET: Event Counter 1 Overflow Set Mask */

#define PMU_OVSSET_CNT2_STATUS_Pos            2U                                           /*!< PMU OVSSET: Event Counter 2 Overflow Set Position */
#define PMU_OVSSET_CNT2_STATUS_Msk           (1UL << PMU_OVSSET_CNT2_STATUS_Pos)           /*!< PMU OVSSET: Event Counter 2 Overflow Set Mask */

#define PMU_OVSSET_CNT3_STATUS_Pos            3U                                           /*!< PMU OVSSET: Event Counter 3 Overflow Set Position */
#define PMU_OVSSET_CNT3_STATUS_Msk           (1UL << PMU_OVSSET_CNT3_STATUS_Pos)           /*!< PMU OVSSET: Event Counter 3 Overflow Set Mask */

#define PMU_OVSSET_CNT4_STATUS_Pos            4U                                           /*!< PMU OVSSET: Event Counter 4 Overflow Set Position */
#define PMU_OVSSET_CNT4_STATUS_Msk           (1UL << PMU_OVSSET_CNT4_STATUS_Pos)           /*!< PMU OVSSET: Event Counter 4 Overflow Set Mask */

#define PMU_OVSSET_CNT5_STATUS_Pos            5U                                           /*!< PMU OVSSET: Event Counter 5 Overflow Set Position */
#define PMU_OVSSET_CNT5_STATUS_Msk           (1UL << PMU_OVSSET_CNT5_STATUS_Pos)           /*!< PMU OVSSET: Event Counter 5 Overflow Set Mask */

#define PMU_OVSSET_CNT6_STATUS_Pos            6U                                           /*!< PMU OVSSET: Event Counter 6 Overflow Set Position */
#define PMU_OVSSET_CNT6_STATUS_Msk           (1UL << PMU_OVSSET_CNT6_STATUS_Pos)           /*!< PMU OVSSET: Event Counter 6 Overflow Set Mask */

#define PMU_OVSSET_CNT7_STATUS_Pos            7U                                           /*!< PMU OVSSET: Event Counter 7 Overflow Set Position */
#define PMU_OVSSET_CNT7_STATUS_Msk           (1UL << PMU_OVSSET_CNT7_STATUS_Pos)           /*!< PMU OVSSET: Event Counter 7 Overflow Set Mask */

#define PMU_OVSSET_CNT8_STATUS_Pos            8U                                           /*!< PMU OVSSET: Event Counter 8 Overflow Set Position */
#define PMU_OVSSET_CNT8_STATUS_Msk           (1UL << PMU_OVSSET_CNT8_STATUS_Pos)           /*!< PMU OVSSET: Event Counter 8 Overflow Set Mask */

#define PMU_OVSSET_CNT9_STATUS_Pos            9U                                           /*!< PMU OVSSET: Event Counter 9 Overflow Set Position */
#define PMU_OVSSET_CNT9_STATUS_Msk           (1UL << PMU_OVSSET_CNT9_STATUS_Pos)           /*!< PMU OVSSET: Event Counter 9 Overflow Set Mask */

#define PMU_OVSSET_CNT10_STATUS_Pos           10U                                          /*!< PMU OVSSET: Event Counter 10 Overflow Set Position */
#define PMU_OVSSET_CNT10_STATUS_Msk          (1UL << PMU_OVSSET_CNT10_STATUS_Pos)          /*!< PMU OVSSET: Event Counter 10 Overflow Set Mask */

#define PMU_OVSSET_CNT11_STATUS_Pos           11U                                          /*!< PMU OVSSET: Event Counter 11 Overflow Set Position */
#define PMU_OVSSET_CNT11_STATUS_Msk          (1UL << PMU_OVSSET_CNT11_STATUS_Pos)          /*!< PMU OVSSET: Event Counter 11 Overflow Set Mask */

#define PMU_OVSSET_CNT12_STATUS_Pos           12U                                          /*!< PMU OVSSET: Event Counter 12 Overflow Set Position */
#define PMU_OVSSET_CNT12_STATUS_Msk          (1UL << PMU_OVSSET_CNT12_STATUS_Pos)          /*!< PMU OVSSET: Event Counter 12 Overflow Set Mask */

#define PMU_OVSSET_CNT13_STATUS_Pos           13U                                          /*!< PMU OVSSET: Event Counter 13 Overflow Set Position */
#define PMU_OVSSET_CNT13_STATUS_Msk          (1UL << PMU_OVSSET_CNT13_STATUS_Pos)          /*!< PMU OVSSET: Event Counter 13 Overflow Set Mask */

#define PMU_OVSSET_CNT14_STATUS_Pos           14U                                          /*!< PMU OVSSET: Event Counter 14 Overflow Set Position */
#define PMU_OVSSET_CNT14_STATUS_Msk          (1UL << PMU_OVSSET_CNT14_STATUS_Pos)          /*!< PMU OVSSET: Event Counter 14 Overflow Set Mask */

#define PMU_OVSSET_CNT15_STATUS_Pos           15U                                          /*!< PMU OVSSET: Event Counter 15 Overflow Set Position */
#define PMU_OVSSET_CNT15_STATUS_Msk          (1UL << PMU_OVSSET_CNT15_STATUS_Pos)          /*!< PMU OVSSET: Event Counter 15 Overflow Set Mask */

#define PMU_OVSSET_CNT16_STATUS_Pos           16U                                          /*!< PMU OVSSET: Event Counter 16 Overflow Set Position */
#define PMU_OVSSET_CNT16_STATUS_Msk          (1UL << PMU_OVSSET_CNT16_STATUS_Pos)          /*!< PMU OVSSET: Event Counter 16 Overflow Set Mask */

#define PMU_OVSSET_CNT17_STATUS_Pos           17U                                          /*!< PMU OVSSET: Event Counter 17 Overflow Set Position */
#define PMU_OVSSET_CNT17_STATUS_Msk          (1UL << PMU_OVSSET_CNT17_STATUS_Pos)          /*!< PMU OVSSET: Event Counter 17 Overflow Set Mask */

#define PMU_OVSSET_CNT18_STATUS_Pos           18U                                          /*!< PMU OVSSET: Event Counter 18 Overflow Set Position */
#define PMU_OVSSET_CNT18_STATUS_Msk          (1UL << PMU_OVSSET_CNT18_STATUS_Pos)          /*!< PMU OVSSET: Event Counter 18 Overflow Set Mask */

#define PMU_OVSSET_CNT19_STATUS_Pos           19U                                          /*!< PMU OVSSET: Event Counter 19 Overflow Set Position */
#define PMU_OVSSET_CNT19_STATUS_Msk          (1UL << PMU_OVSSET_CNT19_STATUS_Pos)          /*!< PMU OVSSET: Event Counter 19 Overflow Set Mask */

#define PMU_OVSSET_CNT20_STATUS_Pos           20U                                          /*!< PMU OVSSET: Event Counter 20 Overflow Set Position */
#define PMU_OVSSET_CNT20_STATUS_Msk          (1UL << PMU_OVSSET_CNT20_STATUS_Pos)          /*!< PMU OVSSET: Event Counter 20 Overflow Set Mask */

#define PMU_OVSSET_CNT21_STATUS_Pos           21U                                          /*!< PMU OVSSET: Event Counter 21 Overflow Set Position */
#define PMU_OVSSET_CNT21_STATUS_Msk          (1UL << PMU_OVSSET_CNT21_STATUS_Pos)          /*!< PMU OVSSET: Event Counter 21 Overflow Set Mask */

#define PMU_OVSSET_CNT22_STATUS_Pos           22U                                          /*!< PMU OVSSET: Event Counter 22 Overflow Set Position */
#define PMU_OVSSET_CNT22_STATUS_Msk          (1UL << PMU_OVSSET_CNT22_STATUS_Pos)          /*!< PMU OVSSET: Event Counter 22 Overflow Set Mask */

#define PMU_OVSSET_CNT23_STATUS_Pos           23U                                          /*!< PMU OVSSET: Event Counter 23 Overflow Set Position */
#define PMU_OVSSET_CNT23_STATUS_Msk          (1UL << PMU_OVSSET_CNT23_STATUS_Pos)          /*!< PMU OVSSET: Event Counter 23 Overflow Set Mask */

#define PMU_OVSSET_CNT24_STATUS_Pos           24U                                          /*!< PMU OVSSET: Event Counter 24 Overflow Set Position */
#define PMU_OVSSET_CNT24_STATUS_Msk          (1UL << PMU_OVSSET_CNT24_STATUS_Pos)          /*!< PMU OVSSET: Event Counter 24 Overflow Set Mask */

#define PMU_OVSSET_CNT25_STATUS_Pos           25U                                          /*!< PMU OVSSET: Event Counter 25 Overflow Set Position */
#define PMU_OVSSET_CNT25_STATUS_Msk          (1UL << PMU_OVSSET_CNT25_STATUS_Pos)          /*!< PMU OVSSET: Event Counter 25 Overflow Set Mask */

#define PMU_OVSSET_CNT26_STATUS_Pos           26U                                          /*!< PMU OVSSET: Event Counter 26 Overflow Set Position */
#define PMU_OVSSET_CNT26_STATUS_Msk          (1UL << PMU_OVSSET_CNT26_STATUS_Pos)          /*!< PMU OVSSET: Event Counter 26 Overflow Set Mask */

#define PMU_OVSSET_CNT27_STATUS_Pos           27U                                          /*!< PMU OVSSET: Event Counter 27 Overflow Set Position */
#define PMU_OVSSET_CNT27_STATUS_Msk          (1UL << PMU_OVSSET_CNT27_STATUS_Pos)          /*!< PMU OVSSET: Event Counter 27 Overflow Set Mask */

#define PMU_OVSSET_CNT28_STATUS_Pos           28U                                          /*!< PMU OVSSET: Event Counter 28 Overflow Set Position */
#define PMU_OVSSET_CNT28_STATUS_Msk          (1UL << PMU_OVSSET_CNT28_STATUS_Pos)          /*!< PMU OVSSET: Event Counter 28 Overflow Set Mask */

#define PMU_OVSSET_CNT29_STATUS_Pos           29U                                          /*!< PMU OVSSET: Event Counter 29 Overflow Set Position */
#define PMU_OVSSET_CNT29_STATUS_Msk          (1UL << PMU_OVSSET_CNT29_STATUS_Pos)          /*!< PMU OVSSET: Event Counter 29 Overflow Set Mask */

#define PMU_OVSSET_CNT30_STATUS_Pos           30U                                          /*!< PMU OVSSET: Event Counter 30 Overflow Set Position */
#define PMU_OVSSET_CNT30_STATUS_Msk          (1UL << PMU_OVSSET_CNT30_STATUS_Pos)          /*!< PMU OVSSET: Event Counter 30 Overflow Set Mask */

#define PMU_OVSSET_CYCCNT_STATUS_Pos          31U                                          /*!< PMU OVSSET: Cycle Counter Overflow Set Position */
#define PMU_OVSSET_CYCCNT_STATUS_Msk         (1UL << PMU_OVSSET_CYCCNT_STATUS_Pos)         /*!< PMU OVSSET: Cycle Counter Overflow Set Mask */

/** \brief PMU Overflow Flag Status Clear Register Definitions */

#define PMU_OVSCLR_CNT0_STATUS_Pos            0U                                           /*!< PMU OVSCLR: Event Counter 0 Overflow Clear Position */
#define PMU_OVSCLR_CNT0_STATUS_Msk           (1UL /*<< PMU_OVSCLR_CNT0_STATUS_Pos*/)       /*!< PMU OVSCLR: Event Counter 0 Overflow Clear Mask */

#define PMU_OVSCLR_CNT1_STATUS_Pos            1U                                           /*!< PMU OVSCLR: Event Counter 1 Overflow Clear Position */
#define PMU_OVSCLR_CNT1_STATUS_Msk           (1UL << PMU_OVSCLR_CNT1_STATUS_Pos)           /*!< PMU OVSCLR: Event Counter 1 Overflow Clear */

#define PMU_OVSCLR_CNT2_STATUS_Pos            2U                                           /*!< PMU OVSCLR: Event Counter 2 Overflow Clear Position */
#define PMU_OVSCLR_CNT2_STATUS_Msk           (1UL << PMU_OVSCLR_CNT2_STATUS_Pos)           /*!< PMU OVSCLR: Event Counter 2 Overflow Clear Mask */

#define PMU_OVSCLR_CNT3_STATUS_Pos            3U                                           /*!< PMU OVSCLR: Event Counter 3 Overflow Clear Position */
#define PMU_OVSCLR_CNT3_STATUS_Msk           (1UL << PMU_OVSCLR_CNT3_STATUS_Pos)           /*!< PMU OVSCLR: Event Counter 3 Overflow Clear Mask */

#define PMU_OVSCLR_CNT4_STATUS_Pos            4U                                           /*!< PMU OVSCLR: Event Counter 4 Overflow Clear Position */
#define PMU_OVSCLR_CNT4_STATUS_Msk           (1UL << PMU_OVSCLR_CNT4_STATUS_Pos)           /*!< PMU OVSCLR: Event Counter 4 Overflow Clear Mask */

#define PMU_OVSCLR_CNT5_STATUS_Pos            5U                                           /*!< PMU OVSCLR: Event Counter 5 Overflow Clear Position */
#define PMU_OVSCLR_CNT5_STATUS_Msk           (1UL << PMU_OVSCLR_CNT5_STATUS_Pos)           /*!< PMU OVSCLR: Event Counter 5 Overflow Clear Mask */

#define PMU_OVSCLR_CNT6_STATUS_Pos            6U                                           /*!< PMU OVSCLR: Event Counter 6 Overflow Clear Position */
#define PMU_OVSCLR_CNT6_STATUS_Msk           (1UL << PMU_OVSCLR_CNT6_STATUS_Pos)           /*!< PMU OVSCLR: Event Counter 6 Overflow Clear Mask */

#define PMU_OVSCLR_CNT7_STATUS_Pos            7U                                           /*!< PMU OVSCLR: Event Counter 7 Overflow Clear Position */
#define PMU_OVSCLR_CNT7_STATUS_Msk           (1UL << PMU_OVSCLR_CNT7_STATUS_Pos)           /*!< PMU OVSCLR: Event Counter 7 Overflow Clear Mask */

#define PMU_OVSCLR_CNT8_STATUS_Pos            8U                                           /*!< PMU OVSCLR: Event Counter 8 Overflow Clear Position */
#define PMU_OVSCLR_CNT8_STATUS_Msk           (1UL << PMU_OVSCLR_CNT8_STATUS_Pos)           /*!< PMU OVSCLR: Event Counter 8 Overflow Clear Mask */

#define PMU_OVSCLR_CNT9_STATUS_Pos            9U                                           /*!< PMU OVSCLR: Event Counter 9 Overflow Clear Position */
#define PMU_OVSCLR_CNT9_STATUS_Msk           (1UL << PMU_OVSCLR_CNT9_STATUS_Pos)           /*!< PMU OVSCLR: Event Counter 9 Overflow Clear Mask */

#define PMU_OVSCLR_CNT10_STATUS_Pos           10U                                          /*!< PMU OVSCLR: Event Counter 10 Overflow Clear Position */
#define PMU_OVSCLR_CNT10_STATUS_Msk          (1UL << PMU_OVSCLR_CNT10_STATUS_Pos)          /*!< PMU OVSCLR: Event Counter 10 Overflow Clear Mask */

#define PMU_OVSCLR_CNT11_STATUS_Pos           11U                                          /*!< PMU OVSCLR: Event Counter 11 Overflow Clear Position */
#define PMU_OVSCLR_CNT11_STATUS_Msk          (1UL << PMU_OVSCLR_CNT11_STATUS_Pos)          /*!< PMU OVSCLR: Event Counter 11 Overflow Clear Mask */

#define PMU_OVSCLR_CNT12_STATUS_Pos           12U                                          /*!< PMU OVSCLR: Event Counter 12 Overflow Clear Position */
#define PMU_OVSCLR_CNT12_STATUS_Msk          (1UL << PMU_OVSCLR_CNT12_STATUS_Pos)          /*!< PMU OVSCLR: Event Counter 12 Overflow Clear Mask */

#define PMU_OVSCLR_CNT13_STATUS_Pos           13U                                          /*!< PMU OVSCLR: Event Counter 13 Overflow Clear Position */
#define PMU_OVSCLR_CNT13_STATUS_Msk          (1UL << PMU_OVSCLR_CNT13_STATUS_Pos)          /*!< PMU OVSCLR: Event Counter 13 Overflow Clear Mask */

#define PMU_OVSCLR_CNT14_STATUS_Pos           14U                                          /*!< PMU OVSCLR: Event Counter 14 Overflow Clear Position */
#define PMU_OVSCLR_CNT14_STATUS_Msk          (1UL << PMU_OVSCLR_CNT14_STATUS_Pos)          /*!< PMU OVSCLR: Event Counter 14 Overflow Clear Mask */

#define PMU_OVSCLR_CNT15_STATUS_Pos           15U                                          /*!< PMU OVSCLR: Event Counter 15 Overflow Clear Position */
#define PMU_OVSCLR_CNT15_STATUS_Msk          (1UL << PMU_OVSCLR_CNT15_STATUS_Pos)          /*!< PMU OVSCLR: Event Counter 15 Overflow Clear Mask */

#define PMU_OVSCLR_CNT16_STATUS_Pos           16U                                          /*!< PMU OVSCLR: Event Counter 16 Overflow Clear Position */
#define PMU_OVSCLR_CNT16_STATUS_Msk          (1UL << PMU_OVSCLR_CNT16_STATUS_Pos)          /*!< PMU OVSCLR: Event Counter 16 Overflow Clear Mask */

#define PMU_OVSCLR_CNT17_STATUS_Pos           17U                                          /*!< PMU OVSCLR: Event Counter 17 Overflow Clear Position */
#define PMU_OVSCLR_CNT17_STATUS_Msk          (1UL << PMU_OVSCLR_CNT17_STATUS_Pos)          /*!< PMU OVSCLR: Event Counter 17 Overflow Clear Mask */

#define PMU_OVSCLR_CNT18_STATUS_Pos           18U                                          /*!< PMU OVSCLR: Event Counter 18 Overflow Clear Position */
#define PMU_OVSCLR_CNT18_STATUS_Msk          (1UL << PMU_OVSCLR_CNT18_STATUS_Pos)          /*!< PMU OVSCLR: Event Counter 18 Overflow Clear Mask */

#define PMU_OVSCLR_CNT19_STATUS_Pos           19U                                          /*!< PMU OVSCLR: Event Counter 19 Overflow Clear Position */
#define PMU_OVSCLR_CNT19_STATUS_Msk          (1UL << PMU_OVSCLR_CNT19_STATUS_Pos)          /*!< PMU OVSCLR: Event Counter 19 Overflow Clear Mask */

#define PMU_OVSCLR_CNT20_STATUS_Pos           20U                                          /*!< PMU OVSCLR: Event Counter 20 Overflow Clear Position */
#define PMU_OVSCLR_CNT20_STATUS_Msk          (1UL << PMU_OVSCLR_CNT20_STATUS_Pos)          /*!< PMU OVSCLR: Event Counter 20 Overflow Clear Mask */

#define PMU_OVSCLR_CNT21_STATUS_Pos           21U                                          /*!< PMU OVSCLR: Event Counter 21 Overflow Clear Position */
#define PMU_OVSCLR_CNT21_STATUS_Msk          (1UL << PMU_OVSCLR_CNT21_STATUS_Pos)          /*!< PMU OVSCLR: Event Counter 21 Overflow Clear Mask */

#define PMU_OVSCLR_CNT22_STATUS_Pos           22U                                          /*!< PMU OVSCLR: Event Counter 22 Overflow Clear Position */
#define PMU_OVSCLR_CNT22_STATUS_Msk          (1UL << PMU_OVSCLR_CNT22_STATUS_Pos)          /*!< PMU OVSCLR: Event Counter 22 Overflow Clear Mask */

#define PMU_OVSCLR_CNT23_STATUS_Pos           23U                                          /*!< PMU OVSCLR: Event Counter 23 Overflow Clear Position */
#define PMU_OVSCLR_CNT23_STATUS_Msk          (1UL << PMU_OVSCLR_CNT23_STATUS_Pos)          /*!< PMU OVSCLR: Event Counter 23 Overflow Clear Mask */

#define PMU_OVSCLR_CNT24_STATUS_Pos           24U                                          /*!< PMU OVSCLR: Event Counter 24 Overflow Clear Position */
#define PMU_OVSCLR_CNT24_STATUS_Msk          (1UL << PMU_OVSCLR_CNT24_STATUS_Pos)          /*!< PMU OVSCLR: Event Counter 24 Overflow Clear Mask */

#define PMU_OVSCLR_CNT25_STATUS_Pos           25U                                          /*!< PMU OVSCLR: Event Counter 25 Overflow Clear Position */
#define PMU_OVSCLR_CNT25_STATUS_Msk          (1UL << PMU_OVSCLR_CNT25_STATUS_Pos)          /*!< PMU OVSCLR: Event Counter 25 Overflow Clear Mask */

#define PMU_OVSCLR_CNT26_STATUS_Pos           26U                                          /*!< PMU OVSCLR: Event Counter 26 Overflow Clear Position */
#define PMU_OVSCLR_CNT26_STATUS_Msk          (1UL << PMU_OVSCLR_CNT26_STATUS_Pos)          /*!< PMU OVSCLR: Event Counter 26 Overflow Clear Mask */

#define PMU_OVSCLR_CNT27_STATUS_Pos           27U                                          /*!< PMU OVSCLR: Event Counter 27 Overflow Clear Position */
#define PMU_OVSCLR_CNT27_STATUS_Msk          (1UL << PMU_OVSCLR_CNT27_STATUS_Pos)          /*!< PMU OVSCLR: Event Counter 27 Overflow Clear Mask */

#define PMU_OVSCLR_CNT28_STATUS_Pos           28U                                          /*!< PMU OVSCLR: Event Counter 28 Overflow Clear Position */
#define PMU_OVSCLR_CNT28_STATUS_Msk          (1UL << PMU_OVSCLR_CNT28_STATUS_Pos)          /*!< PMU OVSCLR: Event Counter 28 Overflow Clear Mask */

#define PMU_OVSCLR_CNT29_STATUS_Pos           29U                                          /*!< PMU OVSCLR: Event Counter 29 Overflow Clear Position */
#define PMU_OVSCLR_CNT29_STATUS_Msk          (1UL << PMU_OVSCLR_CNT29_STATUS_Pos)          /*!< PMU OVSCLR: Event Counter 29 Overflow Clear Mask */

#define PMU_OVSCLR_CNT30_STATUS_Pos           30U                                          /*!< PMU OVSCLR: Event Counter 30 Overflow Clear Position */
#define PMU_OVSCLR_CNT30_STATUS_Msk          (1UL << PMU_OVSCLR_CNT30_STATUS_Pos)          /*!< PMU OVSCLR: Event Counter 30 Overflow Clear Mask */

#define PMU_OVSCLR_CYCCNT_STATUS_Pos          31U                                          /*!< PMU OVSCLR: Cycle Counter Overflow Clear Position */
#define PMU_OVSCLR_CYCCNT_STATUS_Msk         (1UL << PMU_OVSCLR_CYCCNT_STATUS_Pos)         /*!< PMU OVSCLR: Cycle Counter Overflow Clear Mask */

/** \brief PMU Software Increment Counter */

#define PMU_SWINC_CNT0_Pos                    0U                                           /*!< PMU SWINC: Event Counter 0 Software Increment Position */
#define PMU_SWINC_CNT0_Msk                   (1UL /*<< PMU_SWINC_CNT0_Pos */)              /*!< PMU SWINC: Event Counter 0 Software Increment Mask */

#define PMU_SWINC_CNT1_Pos                    1U                                           /*!< PMU SWINC: Event Counter 1 Software Increment Position */
#define PMU_SWINC_CNT1_Msk                   (1UL << PMU_SWINC_CNT1_Pos)                   /*!< PMU SWINC: Event Counter 1 Software Increment Mask */

#define PMU_SWINC_CNT2_Pos                    2U                                           /*!< PMU SWINC: Event Counter 2 Software Increment Position */
#define PMU_SWINC_CNT2_Msk                   (1UL << PMU_SWINC_CNT2_Pos)                   /*!< PMU SWINC: Event Counter 2 Software Increment Mask */

#define PMU_SWINC_CNT3_Pos                    3U                                           /*!< PMU SWINC: Event Counter 3 Software Increment Position */
#define PMU_SWINC_CNT3_Msk                   (1UL << PMU_SWINC_CNT3_Pos)                   /*!< PMU SWINC: Event Counter 3 Software Increment Mask */

#define PMU_SWINC_CNT4_Pos                    4U                                           /*!< PMU SWINC: Event Counter 4 Software Increment Position */
#define PMU_SWINC_CNT4_Msk                   (1UL << PMU_SWINC_CNT4_Pos)                   /*!< PMU SWINC: Event Counter 4 Software Increment Mask */

#define PMU_SWINC_CNT5_Pos                    5U                                           /*!< PMU SWINC: Event Counter 5 Software Increment Position */
#define PMU_SWINC_CNT5_Msk                   (1UL << PMU_SWINC_CNT5_Pos)                   /*!< PMU SWINC: Event Counter 5 Software Increment Mask */

#define PMU_SWINC_CNT6_Pos                    6U                                           /*!< PMU SWINC: Event Counter 6 Software Increment Position */
#define PMU_SWINC_CNT6_Msk                   (1UL << PMU_SWINC_CNT6_Pos)                   /*!< PMU SWINC: Event Counter 6 Software Increment Mask */

#define PMU_SWINC_CNT7_Pos                    7U                                           /*!< PMU SWINC: Event Counter 7 Software Increment Position */
#define PMU_SWINC_CNT7_Msk                   (1UL << PMU_SWINC_CNT7_Pos)                   /*!< PMU SWINC: Event Counter 7 Software Increment Mask */

#define PMU_SWINC_CNT8_Pos                    8U                                           /*!< PMU SWINC: Event Counter 8 Software Increment Position */
#define PMU_SWINC_CNT8_Msk                   (1UL << PMU_SWINC_CNT8_Pos)                   /*!< PMU SWINC: Event Counter 8 Software Increment Mask */

#define PMU_SWINC_CNT9_Pos                    9U                                           /*!< PMU SWINC: Event Counter 9 Software Increment Position */
#define PMU_SWINC_CNT9_Msk                   (1UL << PMU_SWINC_CNT9_Pos)                   /*!< PMU SWINC: Event Counter 9 Software Increment Mask */

#define PMU_SWINC_CNT10_Pos                   10U                                          /*!< PMU SWINC: Event Counter 10 Software Increment Position */
#define PMU_SWINC_CNT10_Msk                  (1UL << PMU_SWINC_CNT10_Pos)                  /*!< PMU SWINC: Event Counter 10 Software Increment Mask */

#define PMU_SWINC_CNT11_Pos                   11U                                          /*!< PMU SWINC: Event Counter 11 Software Increment Position */
#define PMU_SWINC_CNT11_Msk                  (1UL << PMU_SWINC_CNT11_Pos)                  /*!< PMU SWINC: Event Counter 11 Software Increment Mask */

#define PMU_SWINC_CNT12_Pos                   12U                                          /*!< PMU SWINC: Event Counter 12 Software Increment Position */
#define PMU_SWINC_CNT12_Msk                  (1UL << PMU_SWINC_CNT12_Pos)                  /*!< PMU SWINC: Event Counter 12 Software Increment Mask */

#define PMU_SWINC_CNT13_Pos                   13U                                          /*!< PMU SWINC: Event Counter 13 Software Increment Position */
#define PMU_SWINC_CNT13_Msk                  (1UL << PMU_SWINC_CNT13_Pos)                  /*!< PMU SWINC: Event Counter 13 Software Increment Mask */

#define PMU_SWINC_CNT14_Pos                   14U                                          /*!< PMU SWINC: Event Counter 14 Software Increment Position */
#define PMU_SWINC_CNT14_Msk                  (1UL << PMU_SWINC_CNT14_Pos)                  /*!< PMU SWINC: Event Counter 14 Software Increment Mask */

#define PMU_SWINC_CNT15_Pos                   15U                                          /*!< PMU SWINC: Event Counter 15 Software Increment Position */
#define PMU_SWINC_CNT15_Msk                  (1UL << PMU_SWINC_CNT15_Pos)                  /*!< PMU SWINC: Event Counter 15 Software Increment Mask */

#define PMU_SWINC_CNT16_Pos                   16U                                          /*!< PMU SWINC: Event Counter 16 Software Increment Position */
#define PMU_SWINC_CNT16_Msk                  (1UL << PMU_SWINC_CNT16_Pos)                  /*!< PMU SWINC: Event Counter 16 Software Increment Mask */

#define PMU_SWINC_CNT17_Pos                   17U                                          /*!< PMU SWINC: Event Counter 17 Software Increment Position */
#define PMU_SWINC_CNT17_Msk                  (1UL << PMU_SWINC_CNT17_Pos)                  /*!< PMU SWINC: Event Counter 17 Software Increment Mask */

#define PMU_SWINC_CNT18_Pos                   18U                                          /*!< PMU SWINC: Event Counter 18 Software Increment Position */
#define PMU_SWINC_CNT18_Msk                  (1UL << PMU_SWINC_CNT18_Pos)                  /*!< PMU SWINC: Event Counter 18 Software Increment Mask */

#define PMU_SWINC_CNT19_Pos                   19U                                          /*!< PMU SWINC: Event Counter 19 Software Increment Position */
#define PMU_SWINC_CNT19_Msk                  (1UL << PMU_SWINC_CNT19_Pos)                  /*!< PMU SWINC: Event Counter 19 Software Increment Mask */

#define PMU_SWINC_CNT20_Pos                   20U                                          /*!< PMU SWINC: Event Counter 20 Software Increment Position */
#define PMU_SWINC_CNT20_Msk                  (1UL << PMU_SWINC_CNT20_Pos)                  /*!< PMU SWINC: Event Counter 20 Software Increment Mask */

#define PMU_SWINC_CNT21_Pos                   21U                                          /*!< PMU SWINC: Event Counter 21 Software Increment Position */
#define PMU_SWINC_CNT21_Msk                  (1UL << PMU_SWINC_CNT21_Pos)                  /*!< PMU SWINC: Event Counter 21 Software Increment Mask */

#define PMU_SWINC_CNT22_Pos                   22U                                          /*!< PMU SWINC: Event Counter 22 Software Increment Position */
#define PMU_SWINC_CNT22_Msk                  (1UL << PMU_SWINC_CNT22_Pos)                  /*!< PMU SWINC: Event Counter 22 Software Increment Mask */

#define PMU_SWINC_CNT23_Pos                   23U                                          /*!< PMU SWINC: Event Counter 23 Software Increment Position */
#define PMU_SWINC_CNT23_Msk                  (1UL << PMU_SWINC_CNT23_Pos)                  /*!< PMU SWINC: Event Counter 23 Software Increment Mask */

#define PMU_SWINC_CNT24_Pos                   24U                                          /*!< PMU SWINC: Event Counter 24 Software Increment Position */
#define PMU_SWINC_CNT24_Msk                  (1UL << PMU_SWINC_CNT24_Pos)                  /*!< PMU SWINC: Event Counter 24 Software Increment Mask */

#define PMU_SWINC_CNT25_Pos                   25U                                          /*!< PMU SWINC: Event Counter 25 Software Increment Position */
#define PMU_SWINC_CNT25_Msk                  (1UL << PMU_SWINC_CNT25_Pos)                  /*!< PMU SWINC: Event Counter 25 Software Increment Mask */

#define PMU_SWINC_CNT26_Pos                   26U                                          /*!< PMU SWINC: Event Counter 26 Software Increment Position */
#define PMU_SWINC_CNT26_Msk                  (1UL << PMU_SWINC_CNT26_Pos)                  /*!< PMU SWINC: Event Counter 26 Software Increment Mask */

#define PMU_SWINC_CNT27_Pos                   27U                                          /*!< PMU SWINC: Event Counter 27 Software Increment Position */
#define PMU_SWINC_CNT27_Msk                  (1UL << PMU_SWINC_CNT27_Pos)                  /*!< PMU SWINC: Event Counter 27 Software Increment Mask */

#define PMU_SWINC_CNT28_Pos                   28U                                          /*!< PMU SWINC: Event Counter 28 Software Increment Position */
#define PMU_SWINC_CNT28_Msk                  (1UL << PMU_SWINC_CNT28_Pos)                  /*!< PMU SWINC: Event Counter 28 Software Increment Mask */

#define PMU_SWINC_CNT29_Pos                   29U                                          /*!< PMU SWINC: Event Counter 29 Software Increment Position */
#define PMU_SWINC_CNT29_Msk                  (1UL << PMU_SWINC_CNT29_Pos)                  /*!< PMU SWINC: Event Counter 29 Software Increment Mask */

#define PMU_SWINC_CNT30_Pos                   30U                                          /*!< PMU SWINC: Event Counter 30 Software Increment Position */
#define PMU_SWINC_CNT30_Msk                  (1UL << PMU_SWINC_CNT30_Pos)                  /*!< PMU SWINC: Event Counter 30 Software Increment Mask */

/** \brief PMU Control Register Definitions */

#define PMU_CTRL_ENABLE_Pos                   0U                                           /*!< PMU CTRL: ENABLE Position */
#define PMU_CTRL_ENABLE_Msk                  (1UL /*<< PMU_CTRL_ENABLE_Pos*/)              /*!< PMU CTRL: ENABLE Mask */

#define PMU_CTRL_EVENTCNT_RESET_Pos           1U                                           /*!< PMU CTRL: Event Counter Reset Position */
#define PMU_CTRL_EVENTCNT_RESET_Msk          (1UL << PMU_CTRL_EVENTCNT_RESET_Pos)          /*!< PMU CTRL: Event Counter Reset Mask */

#define PMU_CTRL_CYCCNT_RESET_Pos             2U                                           /*!< PMU CTRL: Cycle Counter Reset Position */
#define PMU_CTRL_CYCCNT_RESET_Msk            (1UL << PMU_CTRL_CYCCNT_RESET_Pos)            /*!< PMU CTRL: Cycle Counter Reset Mask */

#define PMU_CTRL_CYCCNT_DISABLE_Pos           5U                                           /*!< PMU CTRL: Disable Cycle Counter Position */
#define PMU_CTRL_CYCCNT_DISABLE_Msk          (1UL << PMU_CTRL_CYCCNT_DISABLE_Pos)          /*!< PMU CTRL: Disable Cycle Counter Mask */

#define PMU_CTRL_FRZ_ON_OV_Pos                9U                                           /*!< PMU CTRL: Freeze-on-overflow Position */
#define PMU_CTRL_FRZ_ON_OV_Msk               (1UL << PMU_CTRL_FRZ_ON_OVERFLOW_Pos)         /*!< PMU CTRL: Freeze-on-overflow Mask */

#define PMU_CTRL_TRACE_ON_OV_Pos              11U                                          /*!< PMU CTRL: Trace-on-overflow Position */
#define PMU_CTRL_TRACE_ON_OV_Msk             (1UL << PMU_CTRL_TRACE_ON_OVERFLOW_Pos)       /*!< PMU CTRL: Trace-on-overflow Mask */

/** \brief PMU Type Register Definitions */

#define PMU_TYPE_NUM_CNTS_Pos                 0U                                           /*!< PMU TYPE: Number of Counters Position */
#define PMU_TYPE_NUM_CNTS_Msk                (0xFFUL /*<< PMU_TYPE_NUM_CNTS_Pos*/)         /*!< PMU TYPE: Number of Counters Mask */

#define PMU_TYPE_SIZE_CNTS_Pos                8U                                           /*!< PMU TYPE: Size of Counters Position */
#define PMU_TYPE_SIZE_CNTS_Msk               (0x3FUL << PMU_TYPE_SIZE_CNTS_Pos)            /*!< PMU TYPE: Size of Counters Mask */

#define PMU_TYPE_CYCCNT_PRESENT_Pos           14U                                          /*!< PMU TYPE: Cycle Counter Present Position */
#define PMU_TYPE_CYCCNT_PRESENT_Msk          (1UL << PMU_TYPE_CYCCNT_PRESENT_Pos)          /*!< PMU TYPE: Cycle Counter Present Mask */

#define PMU_TYPE_FRZ_OV_SUPPORT_Pos           21U                                          /*!< PMU TYPE: Freeze-on-overflow Support Position */
#define PMU_TYPE_FRZ_OV_SUPPORT_Msk          (1UL << PMU_TYPE_FRZ_OV_SUPPORT_Pos)          /*!< PMU TYPE: Freeze-on-overflow Support Mask */

#define PMU_TYPE_TRACE_ON_OV_SUPPORT_Pos      23U                                          /*!< PMU TYPE: Trace-on-overflow Support Position */
#define PMU_TYPE_TRACE_ON_OV_SUPPORT_Msk     (1UL << PMU_TYPE_FRZ_OV_SUPPORT_Pos)          /*!< PMU TYPE: Trace-on-overflow Support Mask */

/** \brief PMU Authentication Status Register Definitions */

#define PMU_AUTHSTATUS_NSID_Pos               0U                                           /*!< PMU AUTHSTATUS: Non-secure Invasive Debug Position */
#define PMU_AUTHSTATUS_NSID_Msk              (0x3UL /*<< PMU_AUTHSTATUS_NSID_Pos*/)        /*!< PMU AUTHSTATUS: Non-secure Invasive Debug Mask */

#define PMU_AUTHSTATUS_NSNID_Pos              2U                                           /*!< PMU AUTHSTATUS: Non-secure Non-invasive Debug Position */
#define PMU_AUTHSTATUS_NSNID_Msk             (0x3UL << PMU_AUTHSTATUS_NSNID_Pos)           /*!< PMU AUTHSTATUS: Non-secure Non-invasive Debug Mask */

#define PMU_AUTHSTATUS_SID_Pos                4U                                           /*!< PMU AUTHSTATUS: Secure Invasive Debug Position */
#define PMU_AUTHSTATUS_SID_Msk               (0x3UL << PMU_AUTHSTATUS_SID_Pos)             /*!< PMU AUTHSTATUS: Secure Invasive Debug Mask */

#define PMU_AUTHSTATUS_SNID_Pos               6U                                           /*!< PMU AUTHSTATUS: Secure Non-invasive Debug Position */
#define PMU_AUTHSTATUS_SNID_Msk              (0x3UL << PMU_AUTHSTATUS_SNID_Pos)            /*!< PMU AUTHSTATUS: Secure Non-invasive Debug Mask */

#define PMU_AUTHSTATUS_NSUID_Pos              16U                                          /*!< PMU AUTHSTATUS: Non-secure Unprivileged Invasive Debug Position */
#define PMU_AUTHSTATUS_NSUID_Msk             (0x3UL << PMU_AUTHSTATUS_NSUID_Pos)           /*!< PMU AUTHSTATUS: Non-secure Unprivileged Invasive Debug Mask */

#define PMU_AUTHSTATUS_NSUNID_Pos             18U                                          /*!< PMU AUTHSTATUS: Non-secure Unprivileged Non-invasive Debug Position */
#define PMU_AUTHSTATUS_NSUNID_Msk            (0x3UL << PMU_AUTHSTATUS_NSUNID_Pos)          /*!< PMU AUTHSTATUS: Non-secure Unprivileged Non-invasive Debug Mask */

#define PMU_AUTHSTATUS_SUID_Pos               20U                                          /*!< PMU AUTHSTATUS: Secure Unprivileged Invasive Debug Position */
#define PMU_AUTHSTATUS_SUID_Msk              (0x3UL << PMU_AUTHSTATUS_SUID_Pos)            /*!< PMU AUTHSTATUS: Secure Unprivileged Invasive Debug Mask */

#define PMU_AUTHSTATUS_SUNID_Pos              22U                                          /*!< PMU AUTHSTATUS: Secure Unprivileged Non-invasive Debug Position */
#define PMU_AUTHSTATUS_SUNID_Msk             (0x3UL << PMU_AUTHSTATUS_SUNID_Pos)           /*!< PMU AUTHSTATUS: Secure Unprivileged Non-invasive Debug Mask */


/*@} end of group CMSIS_PMU */
#endif

#if defined (__MPU_PRESENT) && (__MPU_PRESENT == 1U)
/**
  \ingroup  CMSIS_core_register
  \defgroup CMSIS_MPU     Memory Protection Unit (MPU)
  \brief    Type definitions for the Memory Protection Unit (MPU)
  @{
 */

/**
  \brief  Structure type to access the Memory Protection Unit (MPU).
 */
typedef struct
{
  __IM  uint32_t TYPE;                   /*!< Offset: 0x000 (R/ )  MPU Type Register */
  __IOM uint32_t CTRL;                   /*!< Offset: 0x004 (R/W)  MPU Control Register */
  __IOM uint32_t RNR;                    /*!< Offset: 0x008 (R/W)  MPU Region Number Register */
  __IOM uint32_t RBAR;                   /*!< Offset: 0x00C (R/W)  MPU Region Base Address Register */
  __IOM uint32_t RLAR;                   /*!< Offset: 0x010 (R/W)  MPU Region Limit Address Register */
  __IOM uint32_t RBAR_A1;                /*!< Offset: 0x014 (R/W)  MPU Region Base Address Register Alias 1 */
  __IOM uint32_t RLAR_A1;                /*!< Offset: 0x018 (R/W)  MPU Region Limit Address Register Alias 1 */
  __IOM uint32_t RBAR_A2;                /*!< Offset: 0x01C (R/W)  MPU Region Base Address Register Alias 2 */
  __IOM uint32_t RLAR_A2;                /*!< Offset: 0x020 (R/W)  MPU Region Limit Address Register Alias 2 */
  __IOM uint32_t RBAR_A3;                /*!< Offset: 0x024 (R/W)  MPU Region Base Address Register Alias 3 */
  __IOM uint32_t RLAR_A3;                /*!< Offset: 0x028 (R/W)  MPU Region Limit Address Register Alias 3 */
        uint32_t RESERVED0[1];
  union {
  __IOM uint32_t MAIR[2];
  struct {
  __IOM uint32_t MAIR0;                  /*!< Offset: 0x030 (R/W)  MPU Memory Attribute Indirection Register 0 */
  __IOM uint32_t MAIR1;                  /*!< Offset: 0x034 (R/W)  MPU Memory Attribute Indirection Register 1 */
  };
  };
} MPU_Type;

#define MPU_TYPE_RALIASES                  4U

/* MPU Type Register Definitions */
#define MPU_TYPE_IREGION_Pos               16U                                            /*!< MPU TYPE: IREGION Position */
#define MPU_TYPE_IREGION_Msk               (0xFFUL << MPU_TYPE_IREGION_Pos)               /*!< MPU TYPE: IREGION Mask */

#define MPU_TYPE_DREGION_Pos                8U                                            /*!< MPU TYPE: DREGION Position */
#define MPU_TYPE_DREGION_Msk               (0xFFUL << MPU_TYPE_DREGION_Pos)               /*!< MPU TYPE: DREGION Mask */

#define MPU_TYPE_SEPARATE_Pos               0U                                            /*!< MPU TYPE: SEPARATE Position */
#define MPU_TYPE_SEPARATE_Msk              (1UL /*<< MPU_TYPE_SEPARATE_Pos*/)             /*!< MPU TYPE: SEPARATE Mask */

/* MPU Control Register Definitions */
#define MPU_CTRL_PRIVDEFENA_Pos             2U                                            /*!< MPU CTRL: PRIVDEFENA Position */
#define MPU_CTRL_PRIVDEFENA_Msk            (1UL << MPU_CTRL_PRIVDEFENA_Pos)               /*!< MPU CTRL: PRIVDEFENA Mask */

#define MPU_CTRL_HFNMIENA_Pos               1U                                            /*!< MPU CTRL: HFNMIENA Position */
#define MPU_CTRL_HFNMIENA_Msk              (1UL << MPU_CTRL_HFNMIENA_Pos)                 /*!< MPU CTRL: HFNMIENA Mask */

#define MPU_CTRL_ENABLE_Pos                 0U                                            /*!< MPU CTRL: ENABLE Position */
#define MPU_CTRL_ENABLE_Msk                (1UL /*<< MPU_CTRL_ENABLE_Pos*/)               /*!< MPU CTRL: ENABLE Mask */

/* MPU Region Number Register Definitions */
#define MPU_RNR_REGION_Pos                  0U                                            /*!< MPU RNR: REGION Position */
#define MPU_RNR_REGION_Msk                 (0xFFUL /*<< MPU_RNR_REGION_Pos*/)             /*!< MPU RNR: REGION Mask */

/* MPU Region Base Address Register Definitions */
#define MPU_RBAR_BASE_Pos                   5U                                            /*!< MPU RBAR: BASE Position */
#define MPU_RBAR_BASE_Msk                  (0x7FFFFFFUL << MPU_RBAR_BASE_Pos)             /*!< MPU RBAR: BASE Mask */

#define MPU_RBAR_SH_Pos                     3U                                            /*!< MPU RBAR: SH Position */
#define MPU_RBAR_SH_Msk                    (0x3UL << MPU_RBAR_SH_Pos)                     /*!< MPU RBAR: SH Mask */

#define MPU_RBAR_AP_Pos                     1U                                            /*!< MPU RBAR: AP Position */
#define MPU_RBAR_AP_Msk                    (0x3UL << MPU_RBAR_AP_Pos)                     /*!< MPU RBAR: AP Mask */

#define MPU_RBAR_XN_Pos                     0U                                            /*!< MPU RBAR: XN Position */
#define MPU_RBAR_XN_Msk                    (01UL /*<< MPU_RBAR_XN_Pos*/)                  /*!< MPU RBAR: XN Mask */

/* MPU Region Limit Address Register Definitions */
#define MPU_RLAR_LIMIT_Pos                  5U                                            /*!< MPU RLAR: LIMIT Position */
#define MPU_RLAR_LIMIT_Msk                 (0x7FFFFFFUL << MPU_RLAR_LIMIT_Pos)            /*!< MPU RLAR: LIMIT Mask */

#define MPU_RLAR_PXN_Pos                    4U                                            /*!< MPU RLAR: PXN Position */
#define MPU_RLAR_PXN_Msk                   (1UL << MPU_RLAR_PXN_Pos)                      /*!< MPU RLAR: PXN Mask */

#define MPU_RLAR_AttrIndx_Pos               1U                                            /*!< MPU RLAR: AttrIndx Position */
#define MPU_RLAR_AttrIndx_Msk              (7UL << MPU_RLAR_AttrIndx_Pos)                 /*!< MPU RLAR: AttrIndx Mask */

#define MPU_RLAR_EN_Pos                     0U                                            /*!< MPU RLAR: Region enable bit Position */
#define MPU_RLAR_EN_Msk                    (1UL /*<< MPU_RLAR_EN_Pos*/)                   /*!< MPU RLAR: Region enable bit Disable Mask */

/* MPU Memory Attribute Indirection Register 0 Definitions */
#define MPU_MAIR0_Attr3_Pos                24U                                            /*!< MPU MAIR0: Attr3 Position */
#define MPU_MAIR0_Attr3_Msk                (0xFFUL << MPU_MAIR0_Attr3_Pos)                /*!< MPU MAIR0: Attr3 Mask */

#define MPU_MAIR0_Attr2_Pos                16U                                            /*!< MPU MAIR0: Attr2 Position */
#define MPU_MAIR0_Attr2_Msk                (0xFFUL << MPU_MAIR0_Attr2_Pos)                /*!< MPU MAIR0: Attr2 Mask */

#define MPU_MAIR0_Attr1_Pos                 8U                                            /*!< MPU MAIR0: Attr1 Position */
#define MPU_MAIR0_Attr1_Msk                (0xFFUL << MPU_MAIR0_Attr1_Pos)                /*!< MPU MAIR0: Attr1 Mask */

#define MPU_MAIR0_Attr0_Pos                 0U                                            /*!< MPU MAIR0: Attr0 Position */
#define MPU_MAIR0_Attr0_Msk                (0xFFUL /*<< MPU_MAIR0_Attr0_Pos*/)            /*!< MPU MAIR0: Attr0 Mask */

/* MPU Memory Attribute Indirection Register 1 Definitions */
#define MPU_MAIR1_Attr7_Pos                24U                                            /*!< MPU MAIR1: Attr7 Position */
#define MPU_MAIR1_Attr7_Msk                (0xFFUL << MPU_MAIR1_Attr7_Pos)                /*!< MPU MAIR1: Attr7 Mask */

#define MPU_MAIR1_Attr6_Pos                16U                                            /*!< MPU MAIR1: Attr6 Position */
#define MPU_MAIR1_Attr6_Msk                (0xFFUL << MPU_MAIR1_Attr6_Pos)                /*!< MPU MAIR1: Attr6 Mask */

#define MPU_MAIR1_Attr5_Pos                 8U                                            /*!< MPU MAIR1: Attr5 Position */
#define MPU_MAIR1_Attr5_Msk                (0xFFUL << MPU_MAIR1_Attr5_Pos)                /*!< MPU MAIR1: Attr5 Mask */

#define MPU_MAIR1_Attr4_Pos                 0U                                            /*!< MPU MAIR1: Attr4 Position */
#define MPU_MAIR1_Attr4_Msk                (0xFFUL /*<< MPU_MAIR1_Attr4_Pos*/)            /*!< MPU MAIR1: Attr4 Mask */

/*@} end of group CMSIS_MPU */
#endif


#if defined (__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3U)
/**
  \ingroup  CMSIS_core_register
  \defgroup CMSIS_SAU     Security Attribution Unit (SAU)
  \brief    Type definitions for the Security Attribution Unit (SAU)
  @{
 */

/**
  \brief  Structure type to access the Security Attribution Unit (SAU).
 */
typedef struct
{
  __IOM uint32_t CTRL;                   /*!< Offset: 0x000 (R/W)  SAU Control Register */
  __IM  uint32_t TYPE;                   /*!< Offset: 0x004 (R/ )  SAU Type Register */
#if defined (__SAUREGION_PRESENT) && (__SAUREGION_PRESENT == 1U)
  __IOM uint32_t RNR;                    /*!< Offset: 0x008 (R/W)  SAU Region Number Register */
  __IOM uint32_t RBAR;                   /*!< Offset: 0x00C (R/W)  SAU Region Base Address Register */
  __IOM uint32_t RLAR;                   /*!< Offset: 0x010 (R/W)  SAU Region Limit Address Register */
#else
        uint32_t RESERVED0[3];
#endif
  __IOM uint32_t SFSR;                   /*!< Offset: 0x014 (R/W)  Secure Fault Status Register */
  __IOM uint32_t SFAR;                   /*!< Offset: 0x018 (R/W)  Secure Fault Address Register */
} SAU_Type;

/* SAU Control Register Definitions */
#define SAU_CTRL_ALLNS_Pos                  1U                                            /*!< SAU CTRL: ALLNS Position */
#define SAU_CTRL_ALLNS_Msk                 (1UL << SAU_CTRL_ALLNS_Pos)                    /*!< SAU CTRL: ALLNS Mask */

#define SAU_CTRL_ENABLE_Pos                 0U                                            /*!< SAU CTRL: ENABLE Position */
#define SAU_CTRL_ENABLE_Msk                (1UL /*<< SAU_CTRL_ENABLE_Pos*/)               /*!< SAU CTRL: ENABLE Mask */

/* SAU Type Register Definitions */
#define SAU_TYPE_SREGION_Pos                0U                                            /*!< SAU TYPE: SREGION Position */
#define SAU_TYPE_SREGION_Msk               (0xFFUL /*<< SAU_TYPE_SREGION_Pos*/)           /*!< SAU TYPE: SREGION Mask */

#if defined (__SAUREGION_PRESENT) && (__SAUREGION_PRESENT == 1U)
/* SAU Region Number Register Definitions */
#define SAU_RNR_REGION_Pos                  0U                                            /*!< SAU RNR: REGION Position */
#define SAU_RNR_REGION_Msk                 (0xFFUL /*<< SAU_RNR_REGION_Pos*/)             /*!< SAU RNR: REGION Mask */

/* SAU Region Base Address Register Definitions */
#define SAU_RBAR_BADDR_Pos                  5U                                            /*!< SAU RBAR: BADDR Position */
#define SAU_RBAR_BADDR_Msk                 (0x7FFFFFFUL << SAU_RBAR_BADDR_Pos)            /*!< SAU RBAR: BADDR Mask */

/* SAU Region Limit Address Register Definitions */
#define SAU_RLAR_LADDR_Pos                  5U                                            /*!< SAU RLAR: LADDR Position */
#define SAU_RLAR_LADDR_Msk                 (0x7FFFFFFUL << SAU_RLAR_LADDR_Pos)            /*!< SAU RLAR: LADDR Mask */

#define SAU_RLAR_NSC_Pos                    1U                                            /*!< SAU RLAR: NSC Position */
#define SAU_RLAR_NSC_Msk                   (1UL << SAU_RLAR_NSC_Pos)                      /*!< SAU RLAR: NSC Mask */

#define SAU_RLAR_ENABLE_Pos                 0U                                            /*!< SAU RLAR: ENABLE Position */
#define SAU_RLAR_ENABLE_Msk                (1UL /*<< SAU_RLAR_ENABLE_Pos*/)               /*!< SAU RLAR: ENABLE Mask */

#endif /* defined (__SAUREGION_PRESENT) && (__SAUREGION_PRESENT == 1U) */

/* Secure Fault Status Register Definitions */
#define SAU_SFSR_LSERR_Pos                  7U                                            /*!< SAU SFSR: LSERR Position */
#define SAU_SFSR_LSERR_Msk                 (1UL << SAU_SFSR_LSERR_Pos)                    /*!< SAU SFSR: LSERR Mask */

#define SAU_SFSR_SFARVALID_Pos              6U                                            /*!< SAU SFSR: SFARVALID Position */
#define SAU_SFSR_SFARVALID_Msk             (1UL << SAU_SFSR_SFARVALID_Pos)                /*!< SAU SFSR: SFARVALID Mask */

#define SAU_SFSR_LSPERR_Pos                 5U                                            /*!< SAU SFSR: LSPERR Position */
#define SAU_SFSR_LSPERR_Msk                (1UL << SAU_SFSR_LSPERR_Pos)                   /*!< SAU SFSR: LSPERR Mask */

#define SAU_SFSR_INVTRAN_Pos                4U                                            /*!< SAU SFSR: INVTRAN Position */
#define SAU_SFSR_INVTRAN_Msk               (1UL << SAU_SFSR_INVTRAN_Pos)                  /*!< SAU SFSR: INVTRAN Mask */

#define SAU_SFSR_AUVIOL_Pos                 3U                                            /*!< SAU SFSR: AUVIOL Position */
#define SAU_SFSR_AUVIOL_Msk                (1UL << SAU_SFSR_AUVIOL_Pos)                   /*!< SAU SFSR: AUVIOL Mask */

#define SAU_SFSR_INVER_Pos                  2U                                            /*!< SAU SFSR: INVER Position */
#define SAU_SFSR_INVER_Msk                 (1UL << SAU_SFSR_INVER_Pos)                    /*!< SAU SFSR: INVER Mask */

#define SAU_SFSR_INVIS_Pos                  1U                                            /*!< SAU SFSR: INVIS Position */
#define SAU_SFSR_INVIS_Msk                 (1UL << SAU_SFSR_INVIS_Pos)                    /*!< SAU SFSR: INVIS Mask */

#define SAU_SFSR_INVEP_Pos                  0U                                            /*!< SAU SFSR: INVEP Position */
#define SAU_SFSR_INVEP_Msk                 (1UL /*<< SAU_SFSR_INVEP_Pos*/)                /*!< SAU SFSR: INVEP Mask */

/*@} end of group CMSIS_SAU */
#endif /* defined (__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3U) */


/**
  \ingroup  CMSIS_core_register
  \defgroup CMSIS_FPU     Floating Point Unit (FPU)
  \brief    Type definitions for the Floating Point Unit (FPU)
  @{
 */

/**
  \brief  Structure type to access the Floating Point Unit (FPU).
 */
typedef struct
{
        uint32_t RESERVED0[1U];
  __IOM uint32_t FPCCR;                  /*!< Offset: 0x004 (R/W)  Floating-Point Context Control Register */
  __IOM uint32_t FPCAR;                  /*!< Offset: 0x008 (R/W)  Floating-Point Context Address Register */
  __IOM uint32_t FPDSCR;                 /*!< Offset: 0x00C (R/W)  Floating-Point Default Status Control Register */
  __IM  uint32_t MVFR0;                  /*!< Offset: 0x010 (R/ )  Media and VFP Feature Register 0 */
  __IM  uint32_t MVFR1;                  /*!< Offset: 0x014 (R/ )  Media and VFP Feature Register 1 */
  __IM  uint32_t MVFR2;                  /*!< Offset: 0x018 (R/ )  Media and VFP Feature Register 2 */
} FPU_Type;

/* Floating-Point Context Control Register Definitions */
#define FPU_FPCCR_ASPEN_Pos                31U                                            /*!< FPCCR: ASPEN bit Position */
#define FPU_FPCCR_ASPEN_Msk                (1UL << FPU_FPCCR_ASPEN_Pos)                   /*!< FPCCR: ASPEN bit Mask */

#define FPU_FPCCR_LSPEN_Pos                30U                                            /*!< FPCCR: LSPEN Position */
#define FPU_FPCCR_LSPEN_Msk                (1UL << FPU_FPCCR_LSPEN_Pos)                   /*!< FPCCR: LSPEN bit Mask */

#define FPU_FPCCR_LSPENS_Pos               29U                                            /*!< FPCCR: LSPENS Position */
#define FPU_FPCCR_LSPENS_Msk               (1UL << FPU_FPCCR_LSPENS_Pos)                  /*!< FPCCR: LSPENS bit Mask */

#define FPU_FPCCR_CLRONRET_Pos             28U                                            /*!< FPCCR: CLRONRET Position */
#define FPU_FPCCR_CLRONRET_Msk             (1UL << FPU_FPCCR_CLRONRET_Pos)                /*!< FPCCR: CLRONRET bit Mask */

#define FPU_FPCCR_CLRONRETS_Pos            27U                                            /*!< FPCCR: CLRONRETS Position */
#define FPU_FPCCR_CLRONRETS_Msk            (1UL << FPU_FPCCR_CLRONRETS_Pos)               /*!< FPCCR: CLRONRETS bit Mask */

#define FPU_FPCCR_TS_Pos                   26U                                            /*!< FPCCR: TS Position */
#define FPU_FPCCR_TS_Msk                   (1UL << FPU_FPCCR_TS_Pos)                      /*!< FPCCR: TS bit Mask */

#define FPU_FPCCR_UFRDY_Pos                10U                                            /*!< FPCCR: UFRDY Position */
#define FPU_FPCCR_UFRDY_Msk                (1UL << FPU_FPCCR_UFRDY_Pos)                   /*!< FPCCR: UFRDY bit Mask */

#define FPU_FPCCR_SPLIMVIOL_Pos             9U                                            /*!< FPCCR: SPLIMVIOL Position */
#define FPU_FPCCR_SPLIMVIOL_Msk            (1UL << FPU_FPCCR_SPLIMVIOL_Pos)               /*!< FPCCR: SPLIMVIOL bit Mask */

#define FPU_FPCCR_MONRDY_Pos                8U                                            /*!< FPCCR: MONRDY Position */
#define FPU_FPCCR_MONRDY_Msk               (1UL << FPU_FPCCR_MONRDY_Pos)                  /*!< FPCCR: MONRDY bit Mask */

#define FPU_FPCCR_SFRDY_Pos                 7U                                            /*!< FPCCR: SFRDY Position */
#define FPU_FPCCR_SFRDY_Msk                (1UL << FPU_FPCCR_SFRDY_Pos)                   /*!< FPCCR: SFRDY bit Mask */

#define FPU_FPCCR_BFRDY_Pos                 6U                                            /*!< FPCCR: BFRDY Position */
#define FPU_FPCCR_BFRDY_Msk                (1UL << FPU_FPCCR_BFRDY_Pos)                   /*!< FPCCR: BFRDY bit Mask */

#define FPU_FPCCR_MMRDY_Pos                 5U                                            /*!< FPCCR: MMRDY Position */
#define FPU_FPCCR_MMRDY_Msk                (1UL << FPU_FPCCR_MMRDY_Pos)                   /*!< FPCCR: MMRDY bit Mask */

#define FPU_FPCCR_HFRDY_Pos                 4U                                            /*!< FPCCR: HFRDY Position */
#define FPU_FPCCR_HFRDY_Msk                (1UL << FPU_FPCCR_HFRDY_Pos)                   /*!< FPCCR: HFRDY bit Mask */

#define FPU_FPCCR_THREAD_Pos                3U                                            /*!< FPCCR: processor mode bit Position */
#define FPU_FPCCR_THREAD_Msk               (1UL << FPU_FPCCR_THREAD_Pos)                  /*!< FPCCR: processor mode active bit Mask */

#define FPU_FPCCR_S_Pos                     2U                                            /*!< FPCCR: Security status of the FP context bit Position */
#define FPU_FPCCR_S_Msk                    (1UL << FPU_FPCCR_S_Pos)                       /*!< FPCCR: Security status of the FP context bit Mask */

#define FPU_FPCCR_USER_Pos                  1U                                            /*!< FPCCR: privilege level bit Position */
#define FPU_FPCCR_USER_Msk                 (1UL << FPU_FPCCR_USER_Pos)                    /*!< FPCCR: privilege level bit Mask */

#define FPU_FPCCR_LSPACT_Pos                0U                                            /*!< FPCCR: Lazy state preservation active bit Position */
#define FPU_FPCCR_LSPACT_Msk               (1UL /*<< FPU_FPCCR_LSPACT_Pos*/)              /*!< FPCCR: Lazy state preservation active bit Mask */

/* Floating-Point Context Address Register Definitions */
#define FPU_FPCAR_ADDRESS_Pos               3U                                            /*!< FPCAR: ADDRESS bit Position */
#define FPU_FPCAR_ADDRESS_Msk              (0x1FFFFFFFUL << FPU_FPCAR_ADDRESS_Pos)        /*!< FPCAR: ADDRESS bit Mask */

/* Floating-Point Default Status Control Register Definitions */
#define FPU_FPDSCR_AHP_Pos                 26U                                            /*!< FPDSCR: AHP bit Position */
#define FPU_FPDSCR_AHP_Msk                 (1UL << FPU_FPDSCR_AHP_Pos)                    /*!< FPDSCR: AHP bit Mask */

#define FPU_FPDSCR_DN_Pos                  25U                                            /*!< FPDSCR: DN bit Position */
#define FPU_FPDSCR_DN_Msk                  (1UL << FPU_FPDSCR_DN_Pos)                     /*!< FPDSCR: DN bit Mask */

#define FPU_FPDSCR_FZ_Pos                  24U                                            /*!< FPDSCR: FZ bit Position */
#define FPU_FPDSCR_FZ_Msk                  (1UL << FPU_FPDSCR_FZ_Pos)                     /*!< FPDSCR: FZ bit Mask */

#define FPU_FPDSCR_RMode_Pos               22U                                            /*!< FPDSCR: RMode bit Position */
#define FPU_FPDSCR_RMode_Msk               (3UL << FPU_FPDSCR_RMode_Pos)                  /*!< FPDSCR: RMode bit Mask */

#define FPU_FPDSCR_FZ16_Pos                19U                                            /*!< FPDSCR: FZ16 bit Position */
#define FPU_FPDSCR_FZ16_Msk                (1UL << FPU_FPDSCR_FZ16_Pos)                   /*!< FPDSCR: FZ16 bit Mask */

#define FPU_FPDSCR_LTPSIZE_Pos             16U                                            /*!< FPDSCR: LTPSIZE bit Position */
#define FPU_FPDSCR_LTPSIZE_Msk             (7UL << FPU_FPDSCR_LTPSIZE_Pos)                /*!< FPDSCR: LTPSIZE bit Mask */

/* Media and VFP Feature Register 0 Definitions */
#define FPU_MVFR0_FPRound_Pos              28U                                            /*!< MVFR0: FPRound bits Position */
#define FPU_MVFR0_FPRound_Msk              (0xFUL << FPU_MVFR0_FPRound_Pos)               /*!< MVFR0: FPRound bits Mask */

#define FPU_MVFR0_FPSqrt_Pos               20U                                            /*!< MVFR0: FPSqrt bits Position */
#define FPU_MVFR0_FPSqrt_Msk               (0xFUL << FPU_MVFR0_FPSqrt_Pos)                 /*!< MVFR0: FPSqrt bits Mask */

#define FPU_MVFR0_FPDivide_Pos             16U                                            /*!< MVFR0: FPDivide bits Position */
#define FPU_MVFR0_FPDivide_Msk             (0xFUL << FPU_MVFR0_FPDivide_Pos)              /*!< MVFR0: Divide bits Mask */

#define FPU_MVFR0_FPDP_Pos                  8U                                            /*!< MVFR0: FPDP bits Position */
#define FPU_MVFR0_FPDP_Msk                 (0xFUL << FPU_MVFR0_FPDP_Pos)                  /*!< MVFR0: FPDP bits Mask */

#define FPU_MVFR0_FPSP_Pos                  4U                                            /*!< MVFR0: FPSP bits Position */
#define FPU_MVFR0_FPSP_Msk                 (0xFUL << FPU_MVFR0_FPSP_Pos)                  /*!< MVFR0: FPSP bits Mask */

#define FPU_MVFR0_SIMDReg_Pos               0U                                            /*!< MVFR0: SIMDReg bits Position */
#define FPU_MVFR0_SIMDReg_Msk              (0xFUL /*<< FPU_MVFR0_SIMDReg_Pos*/)           /*!< MVFR0: SIMDReg bits Mask */

/* Media and VFP Feature Register 1 Definitions */
#define FPU_MVFR1_FMAC_Pos                 28U                                            /*!< MVFR1: FMAC bits Position */
#define FPU_MVFR1_FMAC_Msk                 (0xFUL << FPU_MVFR1_FMAC_Pos)                  /*!< MVFR1: FMAC bits Mask */

#define FPU_MVFR1_FPHP_Pos                 24U                                            /*!< MVFR1: FPHP bits Position */
#define FPU_MVFR1_FPHP_Msk                 (0xFUL << FPU_MVFR1_FPHP_Pos)                  /*!< MVFR1: FPHP bits Mask */

#define FPU_MVFR1_FP16_Pos                 20U                                            /*!< MVFR1: FP16 bits Position */
#define FPU_MVFR1_FP16_Msk                 (0xFUL << FPU_MVFR1_FP16_Pos)                  /*!< MVFR1: FP16 bits Mask */

#define FPU_MVFR1_MVE_Pos                   8U                                            /*!< MVFR1: MVE bits Position */
#define FPU_MVFR1_MVE_Msk                  (0xFUL << FPU_MVFR1_MVE_Pos)                   /*!< MVFR1: MVE bits Mask */

#define FPU_MVFR1_FPDNaN_Pos                4U                                            /*!< MVFR1: FPDNaN bits Position */
#define FPU_MVFR1_FPDNaN_Msk               (0xFUL << FPU_MVFR1_FPDNaN_Pos)                /*!< MVFR1: FPDNaN bits Mask */

#define FPU_MVFR1_FPFtZ_Pos                 0U                                            /*!< MVFR1: FPFtZ bits Position */
#define FPU_MVFR1_FPFtZ_Msk                (0xFUL /*<< FPU_MVFR1_FPFtZ_Pos*/)             /*!< MVFR1: FPFtZ bits Mask */

/* Media and VFP Feature Register 2 Definitions */
#define FPU_MVFR2_FPMisc_Pos                4U                                            /*!< MVFR2: FPMisc bits Position */
#define FPU_MVFR2_FPMisc_Msk               (0xFUL << FPU_MVFR2_FPMisc_Pos)                /*!< MVFR2: FPMisc bits Mask */

/*@} end of group CMSIS_FPU */

/* CoreDebug is deprecated. replaced by DCB (Debug Control Block) */
/**
  \ingroup  CMSIS_core_register
  \defgroup CMSIS_CoreDebug       Core Debug Registers (CoreDebug)
  \brief    Type definitions for the Core Debug Registers
  @{
 */

/**
  \brief  \deprecated Structure type to access the Core Debug Register (CoreDebug).
 */
typedef struct
{
  __IOM uint32_t DHCSR;                  /*!< Offset: 0x000 (R/W)  Debug Halting Control and Status Register */
  __OM  uint32_t DCRSR;                  /*!< Offset: 0x004 ( /W)  Debug Core Register Selector Register */
  __IOM uint32_t DCRDR;                  /*!< Offset: 0x008 (R/W)  Debug Core Register Data Register */
  __IOM uint32_t DEMCR;                  /*!< Offset: 0x00C (R/W)  Debug Exception and Monitor Control Register */
  __OM  uint32_t DSCEMCR;                /*!< Offset: 0x010 ( /W)  Debug Set Clear Exception and Monitor Control Register */
  __IOM uint32_t DAUTHCTRL;              /*!< Offset: 0x014 (R/W)  Debug Authentication Control Register */
  __IOM uint32_t DSCSR;                  /*!< Offset: 0x018 (R/W)  Debug Security Control and Status Register */
} CoreDebug_Type;

/* Debug Halting Control and Status Register Definitions */
#define CoreDebug_DHCSR_DBGKEY_Pos         16U                                            /*!< \deprecated CoreDebug DHCSR: DBGKEY Position */
#define CoreDebug_DHCSR_DBGKEY_Msk         (0xFFFFUL << CoreDebug_DHCSR_DBGKEY_Pos)       /*!< \deprecated CoreDebug DHCSR: DBGKEY Mask */

#define CoreDebug_DHCSR_S_RESTART_ST_Pos   26U                                            /*!< \deprecated CoreDebug DHCSR: S_RESTART_ST Position */
#define CoreDebug_DHCSR_S_RESTART_ST_Msk   (1UL << CoreDebug_DHCSR_S_RESTART_ST_Pos)      /*!< \deprecated CoreDebug DHCSR: S_RESTART_ST Mask */

#define CoreDebug_DHCSR_S_RESET_ST_Pos     25U                                            /*!< \deprecated CoreDebug DHCSR: S_RESET_ST Position */
#define CoreDebug_DHCSR_S_RESET_ST_Msk     (1UL << CoreDebug_DHCSR_S_RESET_ST_Pos)        /*!< \deprecated CoreDebug DHCSR: S_RESET_ST Mask */

#define CoreDebug_DHCSR_S_RETIRE_ST_Pos    24U                                            /*!< \deprecated CoreDebug DHCSR: S_RETIRE_ST Position */
#define CoreDebug_DHCSR_S_RETIRE_ST_Msk    (1UL << CoreDebug_DHCSR_S_RETIRE_ST_Pos)       /*!< \deprecated CoreDebug DHCSR: S_RETIRE_ST Mask */

#define CoreDebug_DHCSR_S_FPD_Pos          23U                                            /*!< \deprecated CoreDebug DHCSR: S_FPD Position */
#define CoreDebug_DHCSR_S_FPD_Msk          (1UL << CoreDebug_DHCSR_S_FPD_Pos)             /*!< \deprecated CoreDebug DHCSR: S_FPD Mask */

#define CoreDebug_DHCSR_S_SUIDE_Pos        22U                                            /*!< \deprecated CoreDebug DHCSR: S_SUIDE Position */
#define CoreDebug_DHCSR_S_SUIDE_Msk        (1UL << CoreDebug_DHCSR_S_SUIDE_Pos)           /*!< \deprecated CoreDebug DHCSR: S_SUIDE Mask */

#define CoreDebug_DHCSR_S_NSUIDE_Pos       21U                                            /*!< \deprecated CoreDebug DHCSR: S_NSUIDE Position */
#define CoreDebug_DHCSR_S_NSUIDE_Msk       (1UL << CoreDebug_DHCSR_S_NSUIDE_Pos)          /*!< \deprecated CoreDebug DHCSR: S_NSUIDE Mask */

#define CoreDebug_DHCSR_S_SDE_Pos          20U                                            /*!< \deprecated CoreDebug DHCSR: S_SDE Position */
#define CoreDebug_DHCSR_S_SDE_Msk          (1UL << CoreDebug_DHCSR_S_SDE_Pos)             /*!< \deprecated CoreDebug DHCSR: S_SDE Mask */

#define CoreDebug_DHCSR_S_LOCKUP_Pos       19U                                            /*!< \deprecated CoreDebug DHCSR: S_LOCKUP Position */
#define CoreDebug_DHCSR_S_LOCKUP_Msk       (1UL << CoreDebug_DHCSR_S_LOCKUP_Pos)          /*!< \deprecated CoreDebug DHCSR: S_LOCKUP Mask */

#define CoreDebug_DHCSR_S_SLEEP_Pos        18U                                            /*!< \deprecated CoreDebug DHCSR: S_SLEEP Position */
#define CoreDebug_DHCSR_S_SLEEP_Msk        (1UL << CoreDebug_DHCSR_S_SLEEP_Pos)           /*!< \deprecated CoreDebug DHCSR: S_SLEEP Mask */

#define CoreDebug_DHCSR_S_HALT_Pos         17U                                            /*!< \deprecated CoreDebug DHCSR: S_HALT Position */
#define CoreDebug_DHCSR_S_HALT_Msk         (1UL << CoreDebug_DHCSR_S_HALT_Pos)            /*!< \deprecated CoreDebug DHCSR: S_HALT Mask */

#define CoreDebug_DHCSR_S_REGRDY_Pos       16U                                            /*!< \deprecated CoreDebug DHCSR: S_REGRDY Position */
#define CoreDebug_DHCSR_S_REGRDY_Msk       (1UL << CoreDebug_DHCSR_S_REGRDY_Pos)          /*!< \deprecated CoreDebug DHCSR: S_REGRDY Mask */

#define CoreDebug_DHCSR_C_PMOV_Pos          6U                                            /*!< \deprecated CoreDebug DHCSR: C_PMOV Position */
#define CoreDebug_DHCSR_C_PMOV_Msk         (1UL << CoreDebug_DHCSR_C_PMOV_Pos)            /*!< \deprecated CoreDebug DHCSR: C_PMOV Mask */

#define CoreDebug_DHCSR_C_SNAPSTALL_Pos     5U                                            /*!< \deprecated CoreDebug DHCSR: C_SNAPSTALL Position */
#define CoreDebug_DHCSR_C_SNAPSTALL_Msk    (1UL << CoreDebug_DHCSR_C_SNAPSTALL_Pos)       /*!< \deprecated CoreDebug DHCSR: C_SNAPSTALL Mask */

#define CoreDebug_DHCSR_C_MASKINTS_Pos      3U                                            /*!< \deprecated CoreDebug DHCSR: C_MASKINTS Position */
#define CoreDebug_DHCSR_C_MASKINTS_Msk     (1UL << CoreDebug_DHCSR_C_MASKINTS_Pos)        /*!< \deprecated CoreDebug DHCSR: C_MASKINTS Mask */

#define CoreDebug_DHCSR_C_STEP_Pos          2U                                            /*!< \deprecated CoreDebug DHCSR: C_STEP Position */
#define CoreDebug_DHCSR_C_STEP_Msk         (1UL << CoreDebug_DHCSR_C_STEP_Pos)            /*!< \deprecated CoreDebug DHCSR: C_STEP Mask */

#define CoreDebug_DHCSR_C_HALT_Pos          1U                                            /*!< \deprecated CoreDebug DHCSR: C_HALT Position */
#define CoreDebug_DHCSR_C_HALT_Msk         (1UL << CoreDebug_DHCSR_C_HALT_Pos)            /*!< \deprecated CoreDebug DHCSR: C_HALT Mask */

#define CoreDebug_DHCSR_C_DEBUGEN_Pos       0U                                            /*!< \deprecated CoreDebug DHCSR: C_DEBUGEN Position */
#define CoreDebug_DHCSR_C_DEBUGEN_Msk      (1UL /*<< CoreDebug_DHCSR_C_DEBUGEN_Pos*/)     /*!< \deprecated CoreDebug DHCSR: C_DEBUGEN Mask */

/* Debug Core Register Selector Register Definitions */
#define CoreDebug_DCRSR_REGWnR_Pos         16U                                            /*!< \deprecated CoreDebug DCRSR: REGWnR Position */
#define CoreDebug_DCRSR_REGWnR_Msk         (1UL << CoreDebug_DCRSR_REGWnR_Pos)            /*!< \deprecated CoreDebug DCRSR: REGWnR Mask */

#define CoreDebug_DCRSR_REGSEL_Pos          0U                                            /*!< \deprecated CoreDebug DCRSR: REGSEL Position */
#define CoreDebug_DCRSR_REGSEL_Msk         (0x1FUL /*<< CoreDebug_DCRSR_REGSEL_Pos*/)     /*!< \deprecated CoreDebug DCRSR: REGSEL Mask */

/* Debug Exception and Monitor Control Register Definitions */
#define CoreDebug_DEMCR_TRCENA_Pos         24U                                            /*!< \deprecated CoreDebug DEMCR: TRCENA Position */
#define CoreDebug_DEMCR_TRCENA_Msk         (1UL << CoreDebug_DEMCR_TRCENA_Pos)            /*!< \deprecated CoreDebug DEMCR: TRCENA Mask */

#define CoreDebug_DEMCR_MON_REQ_Pos        19U                                            /*!< \deprecated CoreDebug DEMCR: MON_REQ Position */
#define CoreDebug_DEMCR_MON_REQ_Msk        (1UL << CoreDebug_DEMCR_MON_REQ_Pos)           /*!< \deprecated CoreDebug DEMCR: MON_REQ Mask */

#define CoreDebug_DEMCR_MON_STEP_Pos       18U                                            /*!< \deprecated CoreDebug DEMCR: MON_STEP Position */
#define CoreDebug_DEMCR_MON_STEP_Msk       (1UL << CoreDebug_DEMCR_MON_STEP_Pos)          /*!< \deprecated CoreDebug DEMCR: MON_STEP Mask */

#define CoreDebug_DEMCR_MON_PEND_Pos       17U                                            /*!< \deprecated CoreDebug DEMCR: MON_PEND Position */
#define CoreDebug_DEMCR_MON_PEND_Msk       (1UL << CoreDebug_DEMCR_MON_PEND_Pos)          /*!< \deprecated CoreDebug DEMCR: MON_PEND Mask */

#define CoreDebug_DEMCR_MON_EN_Pos         16U                                            /*!< \deprecated CoreDebug DEMCR: MON_EN Position */
#define CoreDebug_DEMCR_MON_EN_Msk         (1UL << CoreDebug_DEMCR_MON_EN_Pos)            /*!< \deprecated CoreDebug DEMCR: MON_EN Mask */

#define CoreDebug_DEMCR_VC_HARDERR_Pos     10U                                            /*!< \deprecated CoreDebug DEMCR: VC_HARDERR Position */
#define CoreDebug_DEMCR_VC_HARDERR_Msk     (1UL << CoreDebug_DEMCR_VC_HARDERR_Pos)        /*!< \deprecated CoreDebug DEMCR: VC_HARDERR Mask */

#define CoreDebug_DEMCR_VC_INTERR_Pos       9U                                            /*!< \deprecated CoreDebug DEMCR: VC_INTERR Position */
#define CoreDebug_DEMCR_VC_INTERR_Msk      (1UL << CoreDebug_DEMCR_VC_INTERR_Pos)         /*!< \deprecated CoreDebug DEMCR: VC_INTERR Mask */

#define CoreDebug_DEMCR_VC_BUSERR_Pos       8U                                            /*!< \deprecated CoreDebug DEMCR: VC_BUSERR Position */
#define CoreDebug_DEMCR_VC_BUSERR_Msk      (1UL << CoreDebug_DEMCR_VC_BUSERR_Pos)         /*!< \deprecated CoreDebug DEMCR: VC_BUSERR Mask */

#define CoreDebug_DEMCR_VC_STATERR_Pos      7U                                            /*!< \deprecated CoreDebug DEMCR: VC_STATERR Position */
#define CoreDebug_DEMCR_VC_STATERR_Msk     (1UL << CoreDebug_DEMCR_VC_STATERR_Pos)        /*!< \deprecated CoreDebug DEMCR: VC_STATERR Mask */

#define CoreDebug_DEMCR_VC_CHKERR_Pos       6U                                            /*!< \deprecated CoreDebug DEMCR: VC_CHKERR Position */
#define CoreDebug_DEMCR_VC_CHKERR_Msk      (1UL << CoreDebug_DEMCR_VC_CHKERR_Pos)         /*!< \deprecated CoreDebug DEMCR: VC_CHKERR Mask */

#define CoreDebug_DEMCR_VC_NOCPERR_Pos      5U                                            /*!< \deprecated CoreDebug DEMCR: VC_NOCPERR Position */
#define CoreDebug_DEMCR_VC_NOCPERR_Msk     (1UL << CoreDebug_DEMCR_VC_NOCPERR_Pos)        /*!< \deprecated CoreDebug DEMCR: VC_NOCPERR Mask */

#define CoreDebug_DEMCR_VC_MMERR_Pos        4U                                            /*!< \deprecated CoreDebug DEMCR: VC_MMERR Position */
#define CoreDebug_DEMCR_VC_MMERR_Msk       (1UL << CoreDebug_DEMCR_VC_MMERR_Pos)          /*!< \deprecated CoreDebug DEMCR: VC_MMERR Mask */

#define CoreDebug_DEMCR_VC_CORERESET_Pos    0U                                            /*!< \deprecated CoreDebug DEMCR: VC_CORERESET Position */
#define CoreDebug_DEMCR_VC_CORERESET_Msk   (1UL /*<< CoreDebug_DEMCR_VC_CORERESET_Pos*/)  /*!< \deprecated CoreDebug DEMCR: VC_CORERESET Mask */

/* Debug Set Clear Exception and Monitor Control Register Definitions */
#define CoreDebug_DSCEMCR_CLR_MON_REQ_Pos  19U                                            /*!< \deprecated CoreDebug DSCEMCR: CLR_MON_REQ, Position */
#define CoreDebug_DSCEMCR_CLR_MON_REQ_Msk  (1UL << CoreDebug_DSCEMCR_CLR_MON_REQ_Pos)     /*!< \deprecated CoreDebug DSCEMCR: CLR_MON_REQ, Mask */

#define CoreDebug_DSCEMCR_CLR_MON_PEND_Pos 17U                                            /*!< \deprecated CoreDebug DSCEMCR: CLR_MON_PEND, Position */
#define CoreDebug_DSCEMCR_CLR_MON_PEND_Msk (1UL << CoreDebug_DSCEMCR_CLR_MON_PEND_Pos)    /*!< \deprecated CoreDebug DSCEMCR: CLR_MON_PEND, Mask */

#define CoreDebug_DSCEMCR_SET_MON_REQ_Pos   3U                                            /*!< \deprecated CoreDebug DSCEMCR: SET_MON_REQ, Position */
#define CoreDebug_DSCEMCR_SET_MON_REQ_Msk  (1UL << CoreDebug_DSCEMCR_SET_MON_REQ_Pos)     /*!< \deprecated CoreDebug DSCEMCR: SET_MON_REQ, Mask */

#define CoreDebug_DSCEMCR_SET_MON_PEND_Pos  1U                                            /*!< \deprecated CoreDebug DSCEMCR: SET_MON_PEND, Position */
#define CoreDebug_DSCEMCR_SET_MON_PEND_Msk (1UL << CoreDebug_DSCEMCR_SET_MON_PEND_Pos)    /*!< \deprecated CoreDebug DSCEMCR: SET_MON_PEND, Mask */

/* Debug Authentication Control Register Definitions */
#define CoreDebug_DAUTHCTRL_UIDEN_Pos      10U                                            /*!< \deprecated CoreDebug DAUTHCTRL: UIDEN, Position */
#define CoreDebug_DAUTHCTRL_UIDEN_Msk      (1UL << CoreDebug_DAUTHCTRL_UIDEN_Pos)         /*!< \deprecated CoreDebug DAUTHCTRL: UIDEN, Mask */

#define CoreDebug_DAUTHCTRL_UIDAPEN_Pos     9U                                            /*!< \deprecated CoreDebug DAUTHCTRL: UIDAPEN, Position */
#define CoreDebug_DAUTHCTRL_UIDAPEN_Msk    (1UL << CoreDebug_DAUTHCTRL_UIDAPEN_Pos)       /*!< \deprecated CoreDebug DAUTHCTRL: UIDAPEN, Mask */

#define CoreDebug_DAUTHCTRL_FSDMA_Pos       8U                                            /*!< \deprecated CoreDebug DAUTHCTRL: FSDMA, Position */
#define CoreDebug_DAUTHCTRL_FSDMA_Msk      (1UL << CoreDebug_DAUTHCTRL_FSDMA_Pos)         /*!< \deprecated CoreDebug DAUTHCTRL: FSDMA, Mask */

#define CoreDebug_DAUTHCTRL_INTSPNIDEN_Pos  3U                                            /*!< \deprecated CoreDebug DAUTHCTRL: INTSPNIDEN, Position */
#define CoreDebug_DAUTHCTRL_INTSPNIDEN_Msk (1UL << CoreDebug_DAUTHCTRL_INTSPNIDEN_Pos)    /*!< \deprecated CoreDebug DAUTHCTRL: INTSPNIDEN, Mask */

#define CoreDebug_DAUTHCTRL_SPNIDENSEL_Pos  2U                                            /*!< \deprecated CoreDebug DAUTHCTRL: SPNIDENSEL Position */
#define CoreDebug_DAUTHCTRL_SPNIDENSEL_Msk (1UL << CoreDebug_DAUTHCTRL_SPNIDENSEL_Pos)    /*!< \deprecated CoreDebug DAUTHCTRL: SPNIDENSEL Mask */

#define CoreDebug_DAUTHCTRL_INTSPIDEN_Pos   1U                                            /*!< \deprecated CoreDebug DAUTHCTRL: INTSPIDEN Position */
#define CoreDebug_DAUTHCTRL_INTSPIDEN_Msk  (1UL << CoreDebug_DAUTHCTRL_INTSPIDEN_Pos)     /*!< \deprecated CoreDebug DAUTHCTRL: INTSPIDEN Mask */

#define CoreDebug_DAUTHCTRL_SPIDENSEL_Pos   0U                                            /*!< \deprecated CoreDebug DAUTHCTRL: SPIDENSEL Position */
#define CoreDebug_DAUTHCTRL_SPIDENSEL_Msk  (1UL /*<< CoreDebug_DAUTHCTRL_SPIDENSEL_Pos*/) /*!< \deprecated CoreDebug DAUTHCTRL: SPIDENSEL Mask */

/* Debug Security Control and Status Register Definitions */
#define CoreDebug_DSCSR_CDS_Pos            16U                                            /*!< \deprecated CoreDebug DSCSR: CDS Position */
#define CoreDebug_DSCSR_CDS_Msk            (1UL << CoreDebug_DSCSR_CDS_Pos)               /*!< \deprecated CoreDebug DSCSR: CDS Mask */

#define CoreDebug_DSCSR_SBRSEL_Pos          1U                                            /*!< \deprecated CoreDebug DSCSR: SBRSEL Position */
#define CoreDebug_DSCSR_SBRSEL_Msk         (1UL << CoreDebug_DSCSR_SBRSEL_Pos)            /*!< \deprecated CoreDebug DSCSR: SBRSEL Mask */

#define CoreDebug_DSCSR_SBRSELEN_Pos        0U                                            /*!< \deprecated CoreDebug DSCSR: SBRSELEN Position */
#define CoreDebug_DSCSR_SBRSELEN_Msk       (1UL /*<< CoreDebug_DSCSR_SBRSELEN_Pos*/)      /*!< \deprecated CoreDebug DSCSR: SBRSELEN Mask */

/*@} end of group CMSIS_CoreDebug */


/**
  \ingroup  CMSIS_core_register
  \defgroup CMSIS_DCB       Debug Control Block
  \brief    Type definitions for the Debug Control Block Registers
  @{
 */

/**
  \brief  Structure type to access the Debug Control Block Registers (DCB).
 */
typedef struct
{
  __IOM uint32_t DHCSR;                  /*!< Offset: 0x000 (R/W)  Debug Halting Control and Status Register */
  __OM  uint32_t DCRSR;                  /*!< Offset: 0x004 ( /W)  Debug Core Register Selector Register */
  __IOM uint32_t DCRDR;                  /*!< Offset: 0x008 (R/W)  Debug Core Register Data Register */
  __IOM uint32_t DEMCR;                  /*!< Offset: 0x00C (R/W)  Debug Exception and Monitor Control Register */
  __OM  uint32_t DSCEMCR;                /*!< Offset: 0x010 ( /W)  Debug Set Clear Exception and Monitor Control Register */
  __IOM uint32_t DAUTHCTRL;              /*!< Offset: 0x014 (R/W)  Debug Authentication Control Register */
  __IOM uint32_t DSCSR;                  /*!< Offset: 0x018 (R/W)  Debug Security Control and Status Register */
} DCB_Type;

/* DHCSR, Debug Halting Control and Status Register Definitions */
#define DCB_DHCSR_DBGKEY_Pos               16U                                            /*!< DCB DHCSR: Debug key Position */
#define DCB_DHCSR_DBGKEY_Msk               (0xFFFFUL << DCB_DHCSR_DBGKEY_Pos)             /*!< DCB DHCSR: Debug key Mask */

#define DCB_DHCSR_S_RESTART_ST_Pos         26U                                            /*!< DCB DHCSR: Restart sticky status Position */
#define DCB_DHCSR_S_RESTART_ST_Msk         (0x1UL << DCB_DHCSR_S_RESTART_ST_Pos)          /*!< DCB DHCSR: Restart sticky status Mask */

#define DCB_DHCSR_S_RESET_ST_Pos           25U                                            /*!< DCB DHCSR: Reset sticky status Position */
#define DCB_DHCSR_S_RESET_ST_Msk           (0x1UL << DCB_DHCSR_S_RESET_ST_Pos)            /*!< DCB DHCSR: Reset sticky status Mask */

#define DCB_DHCSR_S_RETIRE_ST_Pos          24U                                            /*!< DCB DHCSR: Retire sticky status Position */
#define DCB_DHCSR_S_RETIRE_ST_Msk          (0x1UL << DCB_DHCSR_S_RETIRE_ST_Pos)           /*!< DCB DHCSR: Retire sticky status Mask */

#define DCB_DHCSR_S_FPD_Pos                23U                                            /*!< DCB DHCSR: Floating-point registers Debuggable Position */
#define DCB_DHCSR_S_FPD_Msk                (0x1UL << DCB_DHCSR_S_FPD_Pos)                 /*!< DCB DHCSR: Floating-point registers Debuggable Mask */

#define DCB_DHCSR_S_SUIDE_Pos              22U                                            /*!< DCB DHCSR: Secure unprivileged halting debug enabled Position */
#define DCB_DHCSR_S_SUIDE_Msk              (0x1UL << DCB_DHCSR_S_SUIDE_Pos)               /*!< DCB DHCSR: Secure unprivileged halting debug enabled Mask */

#define DCB_DHCSR_S_NSUIDE_Pos             21U                                            /*!< DCB DHCSR: Non-secure unprivileged halting debug enabled Position */
#define DCB_DHCSR_S_NSUIDE_Msk             (0x1UL << DCB_DHCSR_S_NSUIDE_Pos)              /*!< DCB DHCSR: Non-secure unprivileged halting debug enabled Mask */

#define DCB_DHCSR_S_SDE_Pos                20U                                            /*!< DCB DHCSR: Secure debug enabled Position */
#define DCB_DHCSR_S_SDE_Msk                (0x1UL << DCB_DHCSR_S_SDE_Pos)                 /*!< DCB DHCSR: Secure debug enabled Mask */

#define DCB_DHCSR_S_LOCKUP_Pos             19U                                            /*!< DCB DHCSR: Lockup status Position */
#define DCB_DHCSR_S_LOCKUP_Msk             (0x1UL << DCB_DHCSR_S_LOCKUP_Pos)              /*!< DCB DHCSR: Lockup status Mask */

#define DCB_DHCSR_S_SLEEP_Pos              18U                                            /*!< DCB DHCSR: Sleeping status Position */
#define DCB_DHCSR_S_SLEEP_Msk              (0x1UL << DCB_DHCSR_S_SLEEP_Pos)               /*!< DCB DHCSR: Sleeping status Mask */

#define DCB_DHCSR_S_HALT_Pos               17U                                            /*!< DCB DHCSR: Halted status Position */
#define DCB_DHCSR_S_HALT_Msk               (0x1UL << DCB_DHCSR_S_HALT_Pos)                /*!< DCB DHCSR: Halted status Mask */

#define DCB_DHCSR_S_REGRDY_Pos             16U                                            /*!< DCB DHCSR: Register ready status Position */
#define DCB_DHCSR_S_REGRDY_Msk             (0x1UL << DCB_DHCSR_S_REGRDY_Pos)              /*!< DCB DHCSR: Register ready status Mask */

#define DCB_DHCSR_C_PMOV_Pos                6U                                            /*!< DCB DHCSR: Halt on PMU overflow control Position */
#define DCB_DHCSR_C_PMOV_Msk               (0x1UL << DCB_DHCSR_C_PMOV_Pos)                /*!< DCB DHCSR: Halt on PMU overflow control Mask */

#define DCB_DHCSR_C_SNAPSTALL_Pos           5U                                            /*!< DCB DHCSR: Snap stall control Position */
#define DCB_DHCSR_C_SNAPSTALL_Msk          (0x1UL << DCB_DHCSR_C_SNAPSTALL_Pos)           /*!< DCB DHCSR: Snap stall control Mask */

#define DCB_DHCSR_C_MASKINTS_Pos            3U                                            /*!< DCB DHCSR: Mask interrupts control Position */
#define DCB_DHCSR_C_MASKINTS_Msk           (0x1UL << DCB_DHCSR_C_MASKINTS_Pos)            /*!< DCB DHCSR: Mask interrupts control Mask */

#define DCB_DHCSR_C_STEP_Pos                2U                                            /*!< DCB DHCSR: Step control Position */
#define DCB_DHCSR_C_STEP_Msk               (0x1UL << DCB_DHCSR_C_STEP_Pos)                /*!< DCB DHCSR: Step control Mask */

#define DCB_DHCSR_C_HALT_Pos                1U                                            /*!< DCB DHCSR: Halt control Position */
#define DCB_DHCSR_C_HALT_Msk               (0x1UL << DCB_DHCSR_C_HALT_Pos)                /*!< DCB DHCSR: Halt control Mask */

#define DCB_DHCSR_C_DEBUGEN_Pos             0U                                            /*!< DCB DHCSR: Debug enable control Position */
#define DCB_DHCSR_C_DEBUGEN_Msk            (0x1UL /*<< DCB_DHCSR_C_DEBUGEN_Pos*/)         /*!< DCB DHCSR: Debug enable control Mask */

/* DCRSR, Debug Core Register Select Register Definitions */
#define DCB_DCRSR_REGWnR_Pos               16U                                            /*!< DCB DCRSR: Register write/not-read Position */
#define DCB_DCRSR_REGWnR_Msk               (0x1UL << DCB_DCRSR_REGWnR_Pos)                /*!< DCB DCRSR: Register write/not-read Mask */

#define DCB_DCRSR_REGSEL_Pos                0U                                            /*!< DCB DCRSR: Register selector Position */
#define DCB_DCRSR_REGSEL_Msk               (0x7FUL /*<< DCB_DCRSR_REGSEL_Pos*/)           /*!< DCB DCRSR: Register selector Mask */

/* DCRDR, Debug Core Register Data Register Definitions */
#define DCB_DCRDR_DBGTMP_Pos                0U                                            /*!< DCB DCRDR: Data temporary buffer Position */
#define DCB_DCRDR_DBGTMP_Msk               (0xFFFFFFFFUL /*<< DCB_DCRDR_DBGTMP_Pos*/)     /*!< DCB DCRDR: Data temporary buffer Mask */

/* DEMCR, Debug Exception and Monitor Control Register Definitions */
#define DCB_DEMCR_TRCENA_Pos               24U                                            /*!< DCB DEMCR: Trace enable Position */
#define DCB_DEMCR_TRCENA_Msk               (0x1UL << DCB_DEMCR_TRCENA_Pos)                /*!< DCB DEMCR: Trace enable Mask */

#define DCB_DEMCR_MONPRKEY_Pos             23U                                            /*!< DCB DEMCR: Monitor pend req key Position */
#define DCB_DEMCR_MONPRKEY_Msk             (0x1UL << DCB_DEMCR_MONPRKEY_Pos)              /*!< DCB DEMCR: Monitor pend req key Mask */

#define DCB_DEMCR_UMON_EN_Pos              21U                                            /*!< DCB DEMCR: Unprivileged monitor enable Position */
#define DCB_DEMCR_UMON_EN_Msk              (0x1UL << DCB_DEMCR_UMON_EN_Pos)               /*!< DCB DEMCR: Unprivileged monitor enable Mask */

#define DCB_DEMCR_SDME_Pos                 20U                                            /*!< DCB DEMCR: Secure DebugMonitor enable Position */
#define DCB_DEMCR_SDME_Msk                 (0x1UL << DCB_DEMCR_SDME_Pos)                  /*!< DCB DEMCR: Secure DebugMonitor enable Mask */

#define DCB_DEMCR_MON_REQ_Pos              19U                                            /*!< DCB DEMCR: Monitor request Position */
#define DCB_DEMCR_MON_REQ_Msk              (0x1UL << DCB_DEMCR_MON_REQ_Pos)               /*!< DCB DEMCR: Monitor request Mask */

#define DCB_DEMCR_MON_STEP_Pos             18U                                            /*!< DCB DEMCR: Monitor step Position */
#define DCB_DEMCR_MON_STEP_Msk             (0x1UL << DCB_DEMCR_MON_STEP_Pos)              /*!< DCB DEMCR: Monitor step Mask */

#define DCB_DEMCR_MON_PEND_Pos             17U                                            /*!< DCB DEMCR: Monitor pend Position */
#define DCB_DEMCR_MON_PEND_Msk             (0x1UL << DCB_DEMCR_MON_PEND_Pos)              /*!< DCB DEMCR: Monitor pend Mask */

#define DCB_DEMCR_MON_EN_Pos               16U                                            /*!< DCB DEMCR: Monitor enable Position */
#define DCB_DEMCR_MON_EN_Msk               (0x1UL << DCB_DEMCR_MON_EN_Pos)                /*!< DCB DEMCR: Monitor enable Mask */

#define DCB_DEMCR_VC_SFERR_Pos             11U                                            /*!< DCB DEMCR: Vector Catch SecureFault Position */
#define DCB_DEMCR_VC_SFERR_Msk             (0x1UL << DCB_DEMCR_VC_SFERR_Pos)              /*!< DCB DEMCR: Vector Catch SecureFault Mask */

#define DCB_DEMCR_VC_HARDERR_Pos           10U                                            /*!< DCB DEMCR: Vector Catch HardFault errors Position */
#define DCB_DEMCR_VC_HARDERR_Msk           (0x1UL << DCB_DEMCR_VC_HARDERR_Pos)            /*!< DCB DEMCR: Vector Catch HardFault errors Mask */

#define DCB_DEMCR_VC_INTERR_Pos             9U                                            /*!< DCB DEMCR: Vector Catch interrupt errors Position */
#define DCB_DEMCR_VC_INTERR_Msk            (0x1UL << DCB_DEMCR_VC_INTERR_Pos)             /*!< DCB DEMCR: Vector Catch interrupt errors Mask */

#define DCB_DEMCR_VC_BUSERR_Pos             8U                                            /*!< DCB DEMCR: Vector Catch BusFault errors Position */
#define DCB_DEMCR_VC_BUSERR_Msk            (0x1UL << DCB_DEMCR_VC_BUSERR_Pos)             /*!< DCB DEMCR: Vector Catch BusFault errors Mask */

#define DCB_DEMCR_VC_STATERR_Pos            7U                                            /*!< DCB DEMCR: Vector Catch state errors Position */
#define DCB_DEMCR_VC_STATERR_Msk           (0x1UL << DCB_DEMCR_VC_STATERR_Pos)            /*!< DCB DEMCR: Vector Catch state errors Mask */

#define DCB_DEMCR_VC_CHKERR_Pos             6U                                            /*!< DCB DEMCR: Vector Catch check errors Position */
#define DCB_DEMCR_VC_CHKERR_Msk            (0x1UL << DCB_DEMCR_VC_CHKERR_Pos)             /*!< DCB DEMCR: Vector Catch check errors Mask */

#define DCB_DEMCR_VC_NOCPERR_Pos            5U                                            /*!< DCB DEMCR: Vector Catch NOCP errors Position */
#define DCB_DEMCR_VC_NOCPERR_Msk           (0x1UL << DCB_DEMCR_VC_NOCPERR_Pos)            /*!< DCB DEMCR: Vector Catch NOCP errors Mask */

#define DCB_DEMCR_VC_MMERR_Pos              4U                                            /*!< DCB DEMCR: Vector Catch MemManage errors Position */
#define DCB_DEMCR_VC_MMERR_Msk             (0x1UL << DCB_DEMCR_VC_MMERR_Pos)              /*!< DCB DEMCR: Vector Catch MemManage errors Mask */

#define DCB_DEMCR_VC_CORERESET_Pos          0U                                            /*!< DCB DEMCR: Vector Catch Core reset Position */
#define DCB_DEMCR_VC_CORERESET_Msk         (0x1UL /*<< DCB_DEMCR_VC_CORERESET_Pos*/)      /*!< DCB DEMCR: Vector Catch Core reset Mask */

/* DSCEMCR, Debug Set Clear Exception and Monitor Control Register Definitions */
#define DCB_DSCEMCR_CLR_MON_REQ_Pos        19U                                            /*!< DCB DSCEMCR: Clear monitor request Position */
#define DCB_DSCEMCR_CLR_MON_REQ_Msk        (0x1UL << DCB_DSCEMCR_CLR_MON_REQ_Pos)         /*!< DCB DSCEMCR: Clear monitor request Mask */

#define DCB_DSCEMCR_CLR_MON_PEND_Pos       17U                                            /*!< DCB DSCEMCR: Clear monitor pend Position */
#define DCB_DSCEMCR_CLR_MON_PEND_Msk       (0x1UL << DCB_DSCEMCR_CLR_MON_PEND_Pos)        /*!< DCB DSCEMCR: Clear monitor pend Mask */

#define DCB_DSCEMCR_SET_MON_REQ_Pos         3U                                            /*!< DCB DSCEMCR: Set monitor request Position */
#define DCB_DSCEMCR_SET_MON_REQ_Msk        (0x1UL << DCB_DSCEMCR_SET_MON_REQ_Pos)         /*!< DCB DSCEMCR: Set monitor request Mask */

#define DCB_DSCEMCR_SET_MON_PEND_Pos        1U                                            /*!< DCB DSCEMCR: Set monitor pend Position */
#define DCB_DSCEMCR_SET_MON_PEND_Msk       (0x1UL << DCB_DSCEMCR_SET_MON_PEND_Pos)        /*!< DCB DSCEMCR: Set monitor pend Mask */

/* DAUTHCTRL, Debug Authentication Control Register Definitions */
#define DCB_DAUTHCTRL_UIDEN_Pos            10U                                            /*!< DCB DAUTHCTRL: Unprivileged Invasive Debug Enable Position */
#define DCB_DAUTHCTRL_UIDEN_Msk            (0x1UL << DCB_DAUTHCTRL_UIDEN_Pos)             /*!< DCB DAUTHCTRL: Unprivileged Invasive Debug Enable Mask */

#define DCB_DAUTHCTRL_UIDAPEN_Pos           9U                                            /*!< DCB DAUTHCTRL: Unprivileged Invasive DAP Access Enable Position */
#define DCB_DAUTHCTRL_UIDAPEN_Msk          (0x1UL << DCB_DAUTHCTRL_UIDAPEN_Pos)           /*!< DCB DAUTHCTRL: Unprivileged Invasive DAP Access Enable Mask */

#define DCB_DAUTHCTRL_FSDMA_Pos             8U                                            /*!< DCB DAUTHCTRL: Force Secure DebugMonitor Allowed Position */
#define DCB_DAUTHCTRL_FSDMA_Msk            (0x1UL << DCB_DAUTHCTRL_FSDMA_Pos)             /*!< DCB DAUTHCTRL: Force Secure DebugMonitor Allowed Mask */

#define DCB_DAUTHCTRL_INTSPNIDEN_Pos        3U                                            /*!< DCB DAUTHCTRL: Internal Secure non-invasive debug enable Position */
#define DCB_DAUTHCTRL_INTSPNIDEN_Msk       (0x1UL << DCB_DAUTHCTRL_INTSPNIDEN_Pos)        /*!< DCB DAUTHCTRL: Internal Secure non-invasive debug enable Mask */

#define DCB_DAUTHCTRL_SPNIDENSEL_Pos        2U                                            /*!< DCB DAUTHCTRL: Secure non-invasive debug enable select Position */
#define DCB_DAUTHCTRL_SPNIDENSEL_Msk       (0x1UL << DCB_DAUTHCTRL_SPNIDENSEL_Pos)        /*!< DCB DAUTHCTRL: Secure non-invasive debug enable select Mask */

#define DCB_DAUTHCTRL_INTSPIDEN_Pos         1U                                            /*!< DCB DAUTHCTRL: Internal Secure invasive debug enable Position */
#define DCB_DAUTHCTRL_INTSPIDEN_Msk        (0x1UL << DCB_DAUTHCTRL_INTSPIDEN_Pos)         /*!< DCB DAUTHCTRL: Internal Secure invasive debug enable Mask */

#define DCB_DAUTHCTRL_SPIDENSEL_Pos         0U                                            /*!< DCB DAUTHCTRL: Secure invasive debug enable select Position */
#define DCB_DAUTHCTRL_SPIDENSEL_Msk        (0x1UL /*<< DCB_DAUTHCTRL_SPIDENSEL_Pos*/)     /*!< DCB DAUTHCTRL: Secure invasive debug enable select Mask */

/* DSCSR, Debug Security Control and Status Register Definitions */
#define DCB_DSCSR_CDSKEY_Pos               17U                                            /*!< DCB DSCSR: CDS write-enable key Position */
#define DCB_DSCSR_CDSKEY_Msk               (0x1UL << DCB_DSCSR_CDSKEY_Pos)                /*!< DCB DSCSR: CDS write-enable key Mask */

#define DCB_DSCSR_CDS_Pos                  16U                                            /*!< DCB DSCSR: Current domain Secure Position */
#define DCB_DSCSR_CDS_Msk                  (0x1UL << DCB_DSCSR_CDS_Pos)                   /*!< DCB DSCSR: Current domain Secure Mask */

#define DCB_DSCSR_SBRSEL_Pos                1U                                            /*!< DCB DSCSR: Secure banked register select Position */
#define DCB_DSCSR_SBRSEL_Msk               (0x1UL << DCB_DSCSR_SBRSEL_Pos)                /*!< DCB DSCSR: Secure banked register select Mask */

#define DCB_DSCSR_SBRSELEN_Pos              0U                                            /*!< DCB DSCSR: Secure banked register select enable Position */
#define DCB_DSCSR_SBRSELEN_Msk             (0x1UL /*<< DCB_DSCSR_SBRSELEN_Pos*/)          /*!< DCB DSCSR: Secure banked register select enable Mask */

/*@} end of group CMSIS_DCB */



/**
  \ingroup  CMSIS_core_register
  \defgroup CMSIS_DIB       Debug Identification Block
  \brief    Type definitions for the Debug Identification Block Registers
  @{
 */

/**
  \brief  Structure type to access the Debug Identification Block Registers (DIB).
 */
typedef struct
{
  __OM  uint32_t DLAR;                   /*!< Offset: 0x000 ( /W)  SCS Software Lock Access Register */
  __IM  uint32_t DLSR;                   /*!< Offset: 0x004 (R/ )  SCS Software Lock Status Register */
  __IM  uint32_t DAUTHSTATUS;            /*!< Offset: 0x008 (R/ )  Debug Authentication Status Register */
  __IM  uint32_t DDEVARCH;               /*!< Offset: 0x00C (R/ )  SCS Device Architecture Register */
  __IM  uint32_t DDEVTYPE;               /*!< Offset: 0x010 (R/ )  SCS Device Type Register */
} DIB_Type;

/* DLAR, SCS Software Lock Access Register Definitions */
#define DIB_DLAR_KEY_Pos                    0U                                            /*!< DIB DLAR: KEY Position */
#define DIB_DLAR_KEY_Msk                   (0xFFFFFFFFUL /*<< DIB_DLAR_KEY_Pos */)        /*!< DIB DLAR: KEY Mask */

/* DLSR, SCS Software Lock Status Register Definitions */
#define DIB_DLSR_nTT_Pos                    2U                                            /*!< DIB DLSR: Not thirty-two bit Position */
#define DIB_DLSR_nTT_Msk                   (0x1UL << DIB_DLSR_nTT_Pos )                   /*!< DIB DLSR: Not thirty-two bit Mask */

#define DIB_DLSR_SLK_Pos                    1U                                            /*!< DIB DLSR: Software Lock status Position */
#define DIB_DLSR_SLK_Msk                   (0x1UL << DIB_DLSR_SLK_Pos )                   /*!< DIB DLSR: Software Lock status Mask */

#define DIB_DLSR_SLI_Pos                    0U                                            /*!< DIB DLSR: Software Lock implemented Position */
#define DIB_DLSR_SLI_Msk                   (0x1UL /*<< DIB_DLSR_SLI_Pos*/)                /*!< DIB DLSR: Software Lock implemented Mask */

/* DAUTHSTATUS, Debug Authentication Status Register Definitions */
#define DIB_DAUTHSTATUS_SUNID_Pos          22U                                            /*!< DIB DAUTHSTATUS: Secure Unprivileged Non-invasive Debug Allowed Position */
#define DIB_DAUTHSTD`�HF��] P&7DS|��"�a�PCTWX	cY���Ә�_N 䪮Sd�] P#+V�r%�c#Q��pk\K��~��R�d�O�*��|�a����I\H���?�צ�0aZ��(��9S�6��S�I��C9��I���i������􀌬
��C6���2S0��B�4j�xxn�TB�I���e��Ie��6���4}�Y��y���+=�i��͖�����6���{{[[CLC����;AuTPS^�SuQ�!��UR}8_��UK���O�:dI|�l�l�fŰ�*_S�������;�iE�?
'%��H��Gn��`lQK�@6��F��f�^|Q���M�͒핵��&����tcCXdk�F�˪sHI�JtR]���d*�Hi��z��u���0�,<L��M��������'G�2��Lϰ*BPo���E�=�s�%mN���<)��㪎��3^�q&�<�W{ITc�M��0����izGV���m��r � Vs_�Nn��N�8^��G
���H���F�
�2�-�������͸m���6�.�dlU��?*�'��ˊ4_�p�Q��u�K/�"jܲ��� ���C���*N��N^�6�s���E+�J�d��΀�~]�NѶ�ʛn+�_��^�GN����6ᯮX��P�,_��\�K�i�a��B�	�~ʈ1�㹖�
�Ҕ����L0��=$0��N�oA�<�/$2�m�՗m����uRЛ��4�rs �EE�rQ�z����RT��H۱���\_�S�x��֎Mo�H����R��b�P�J�@&)��O����	���3us���M'��EV�
V��&�_�6�.�q�J,���i�����f���%�#
�� x8�����	�~�X`�9�*�g&��ԝ�|+Љ����	�d6 s�`�uk}n>���5FR̋ڹVa9b�c�D�
.�^"~������_A�9�e��a��C�7�C����d��Y���⳯��@u�� ��ecJL`㒸bʑ�D7Kn�D4��g�:>z��]�&�1и|A
�s�M�����N�v��ɠn�i+V��Z�"B=���=4�b���Y��F̾�Fo�f����T�?�Ś@���{��>����c2��3H��~��^�-�P�x�.���<��傷������u�@ 8>��G���U?`(&�"�a�E�?h��͎�u��`7P.�$[cCQ��n�TxM_��;qb2�L���pdX�_�}8a���]_�]F���)K�vD�5��X�t+���D�����|#�ۉP��nl��*|��l��N�Z�O�����M�M�����ѕ�G[U̦��34@Fu�� Td�NCTI<��A��S��+;gkR�E���w��Y,}���UΝ�«��lP�bs��QEy@�}��s�
�V*0Xs����Ӌ��nF(���̀`N.�0s(���c�#��dn�TU�G*���L�*��!��M��=������2������3q��u�� `8*�'m���:�"xl{��`>���(���kq��e�C��bh�h&��Ù֐)�[�|��
�0klS���������Av�D5Xr��Rq�d��DXB�I�޼hY��Psu�օϒ=�h̊��[�^d����(G�u��P1�,ke�S)���1�#լ[��&�&m����x�<�Sџ�(�4�ܩ�����_����t:1ww+�[�7b�ػ/J:$a����b�Z-�) w{1�XH���J��$l1 ��0j���>�hoV��z�2�m�Ճ'��]����Pv��plV��4�b�ϑ�@k�`�-	��E,�Y ����P�9$�VB��;�<)@���J��o�+�߷A�O
Ȋㄅ�2d���V�ű8~�ַ�F���A�I���\z��;a䱔}�P2�d��.E����U�̎���5��v�*��p\�r����\�/gL���RRؑ߱�C	p���G�p�H7�%L� _˳�ڡ
&	����E�D-ey�^[�?��-Ȭ�	`��U�Ø4Ʒ!]�@"���Ln��]�A��Y���Y��y����*��<r�݆��S<��6�n�T{G[J�O�2����� =-��ޛATr�GaY�V��eN�u+_��AH���o�y��eB�L�k�(��ˀ���Ŕ/�^}3����dvB�ݧe� D+�"S0�_]�A��H^��\X	��z����*�.�M�>��H����������m�:�98,*��2���?tAV&��e�3>>�|<\棋�߼Fjp��yRo�-O!�Ix�e�RY��7 ���ޘ�X�	�$E���-1�����Pz���v�zC�4*�R�=���X��.h[����9�C�5K�:;���L���*ԵlB�Sl�ʎ�3��R����n~,��y��͓-��3��ž�w~��PR��{[u�_)�&�"����Gb��2��I?�c.�
��l@mj�+��)�۴�I0��
�;��"���Ii�����a��t��(4���ٷ"Ρ�@E�v�.f"��S-�LϜa՞z�1�#�����	�(V���ȣ(A�rޒ�P�<R �-�+��~�������"�3�S>�5�c8�����1ĥz��b}�8`|˽9��E�\��s+]�A��M���M��xV�����3=�鵶��<u��6�.�$|#Y���i������xU����\TO�Fw��+�:#�uKR��P�*�ԓZ��Ff�b���:~�\�;�X���.$)�(߬���Mp�RX�ѩ KWOF�
��!� B�WZME�����5��JjEC���R���cWf�=�)�f�R�ų$�a��������v7�e���oQIw~��ۿ;�F1t"��ZQ�@ײZ>w?7�?=�g���)�0^iS��Pބ�b���C 	�>��{��[%�c)���r�]����^ƀj�"����\hָfʒ��7n�I�D��x9��|%�����H�w�-K�0�ճhz��ͲY�����s�g�!w77�?Lfؠ���Aa[��<e:�sj�	����xf��5b)��"�	t�R�K� �5��^?�ﭴ��y���tv�^� d8j��:�F���� }8�!��̎^&-�Wr��ɚ�.Z�m2��)L�~'�.��3�'�)��]J���@���։��pe�!��n"m,2��5���5�o>�hN��V����G<
���:��;kz�[�|����(U��Ƕ�=C��9	�B��?�m[�#�C���*�N��k�����9����%��1��+�s�P<��v�� �p����[	�~��v���$2l�r���u��#+%�~���(k��r�V\Ҕ
�K�}����=�i���g�����Wm�ۜ�M��P"J�7�7V�+��vu>����a�ɠ��i�BO�?�?F!m���A����7$w,���m�"�\P�8kj�W�����,q��s!��a�L4�}��r���|X²鍶ݎ�pQ�n�QgD"O9�dۙ7��"[T{��!ȍT�D}$~�/��-���[eDO˶S�1��=z�_M]�1����e�
��G\ ���g�����8~�r#�>R�gV
�������#�w�6|��*��Z��9�T��y�Ƴ%��1��������4ˤb�VZm��ݙ���MR�W�xf�o��'���]�Ϡ�_�����;J��Jr��,��&�g�h�4��x� ן�������ڏ�&jQ#��0uq C�odT��Gd7Sn��['Cb�ў�PsPh�q�\D�xoZ�y����8e�7�4N�LD�}�Y��y�Z��'c����枼�V��JDw^9rS��Ž�`)`$,Qb�%_�t66c_��jw��NrTtb��4R�E�sXʡ��3����0 Ei�7]s��Gn�!*x>��K�q�E��<oi��v���e�~�&�����53(��ݸ�x�C5�n}�$Ilc�͠)5��!��%n�Z|�Fv�E��@�U��F�����M�̀m���`��N��Lm�խ����r�]��8X*����ٍ�݁��@R���Au_i{�V���q�zApns���#o���4+X+�E"�n��F6Iٚk����r+[���z��~�XcB�ɔn�Tf�B���w�raƙ�{���c�N��@�{��f���P���Y�B�I���k��h�ҭ�˵p�3�F�� iL�����֑��~}mI��e~d,oc�dbn�pu�rCY_SA��3;��_��* )�o�e�n�J�<_D�������<�ȚTif�jb,#��v"�\`��?4KB�a�6�C�x�zn,����V�����7i+�؏]Ud1�e0��3ml�w�p&!��;�S�u-�嵳`�~����q�(%u�"���8C@���13���T �Zq<�Vy�}�,5�ηC9��Q�潥:��|�F�ԏ2S ��x�������V�F��D0}bm嵻{����Vx0c�fy��f5`J �Usg�e�.�*xRZ�H`�r�����hK;��C���;D{+�O��Է�il�owa��4�Z?�~��0 �Z̢�yi���&�+���Z`�Of�rO����t�h�� ��}(|�f���q��	�x+���S��6��oB�g�"S�s�n��r���Ј�քݨ���v3�qE��:_�.y�&�o� �L~��]����X?����V�4�sq�l�bx"��ʥ�)3����~x�&V��v�U\I�c��Cu�,/���f4��v��<�K�A�!q�۾�x��*٨��=�\K����*6㧆x-�U���P3�A�w�������G��F���^h;+i��9��Q�9>���t�m�OdC,x���w����u�H w�`?��Ub5���I<��7��{3�f
�6H�y���Ǖ��w�����nOn-����ʇV�OKw���j��x���QV���6���<��Cw��g��9!Wݶ���F�K��x�{O�?J;9�䲂`���Q��
"�"��)n��j�wN��UC���\����}�ْ�ձ�a���ޅ./PD�:�`En�mR"?����Ay`��:_pD�1:_�vq�#�$��i��C{����}�Y�J�A��Xxo��������Ȫ�Gp��@`8S+;J)�Q�Q�>O@O	����|��Pi,j+�|3�g�zP$[NȤ%�@���FPzr,��L��u5�!2'r������E��|�vOOj�t���t�e�Q9喷r{����J�V�\P�U�(o���ǐ_��6���(�[HF#��s��|y>ĕ�+M�O��.�$z�?D�1'�ªY���8��z��b�l��uo��DѴY�̖V�a�-�k£I��}>����������8��u!O5HEK����㯔$"@I�:��r`b�mPv4��Z�GI��MUq4�o¡���7n��J�O2�%����]�]���1���XT�y���%��?1�,v��� i��6�!��f�(d�=�ƅ���o��5�ˣ0bmk`��sE��'�E&4=�-�P�8���CzH���������%�c�d#:KPX)޺�+<ʯ/$;ckQ�Df�R�E� ?x(������ϊ7�WмQtV*�QdB���^�C'W��)O1�sR���fy�Z�:��
�6�p7����	�b���a��Z�'y��<U��6��~�XKB�I���xy�-��Qh���ӻI�x�����+0#(����{ܛ���[ F�qa�5�o1�|&��G3㱊�&D�2�m�U�z�[�}�ن���<A��v��QcDuN?�����B|����d�i��U+�\D|<����f�NL��8.�5rmΘ�H�e�S��k:�{N�fƏ��b�h�x��]���hX��َ��q��@a�n�OwL��\����{zZ<�Æ����Z�g���lv��"����J�G;�a��U�H��T��Y���!�>���tӀŉ0Ҫ�3����	�~�XoB�	�~֘fҒ啼�Ά�+��C�Y�%w.���Dbv�څ�-E�|մ�r�Y98�YbԦ�"� ֭C.��^�@}�����]�\������<V������7>��DF�J�O  =�)�&�"�Sie�������[5n�6p�z$jvT�,q��-E��(3
��z�f��W�h_+w�s�����Ԍ��w���wr`m�0Q�u�W+NX7_ǛQ��=��D[KCOI���u��
�?0(,&���1��F���.$$Z�(B��6��v�yzg����w�C��d`�,���������}�d.��i3I�6K�;Q��2;�fy�Z��9����*��`'���{ sy}��y�Z�.��J�O=�)��Ų��}���R�ūaKy�լ�`d��45� �5K�`�m���;��^,�������)�"���{g{�������џ$P#D!�hq'o~�DC2)Ë/d0l5��'4"�a�[t_y��36��ʾjWv����v�cj�=X	�I#���?��Rq��9d�%$�U�W�g�9�j֗.���Ѣ�(�램�h��TdSz��#sa��a�Ct	�~�p2�-��o!���!���Q�8�W{�_�?O�v�o��:<����=���6ɪ[��l8v��J��B҉���i��v���k�?�������j�W7F���_+@'H"���@Q�n�TKG@@��R��(��z,Ia��w������0`h�,����Fǋ�R)��B�I�ʼo�f�xb��𳆑:�F����:�++ggR����v��� l �70!�%Y�/X�lT��'_����	މ/s��}�l��k��$�-Ё���װk��v����{8h@*��Fϯ:�3GnV�fۿ%�#��@|���M����q�U-�3ƣ^e�����-E��b�'�B�>CM��8ː�����1 =���tdSp�;!�h3m�R:fU��:��'~��'Ov���a�I��|xڲ���x�ڿ�M"�*0��l0��5n�E��3�r�Y�w�"ձ�K�Ĭ񈙵���7�4Kg<7x����u�O\_h;/�J�����{$csQ��a�PGD
�pY,�`־�z#�5^��� R�4��n>�IL�-��v����+��{��l�{r�]�A��W.��Z�C)��� �G޹�J�(6/����$J�ȎtQ:�k{v�^�@j�2��`h����K٩R�q��\-l�������P]��Jy�޳$	VD�x��t޶�N�|=������Hv���3�d2$R_v��0b���[?Cj�������Q�D\A�Ax��O	���84%��Y�T�.�T{�<i�|�2濶���4�����)(J��z�4SoE�'b����_A΋U(xک����{6���a��zA�m�՝���S�^�z���1+Q��Y������>J��VѩNx'kW�k��A����EhV`����w	޾�Hp�<Q��~�(�#�A��M������ְf�d����n{Y>��F����1`%�&"���\N�>#��AW῀p�{!��q�-�*�cҶ�;��´�̆�Z�6����o?T(f���|�{j� \A�y���S:��^�0�!B�I�N�d=�i���:鋂���7�ӫg��b{io1�C�7�L� �?0,h���������,{e�S#E��0ol�Z��/2�Z�#X! i��6���/&�#������lU�+���Cxdm�3I�C�|_�>Su̫J� �����rB�M����"��<����K}�Y����:�c^�i����r�݌a��Q�C{=�'�.��m�Հg�����0�� �_i�i^k����	�Nܸ��JY�3"�幰N��d�`��zl�AI�򵽏	�>��d~�3��qA��ΰm�b���<�o�3z��Ǥk���4��Z�JF�O���F�'��11����5��ز�9�e�an%������Ό�c1�?�/6�ju-c�e��qG��:��<wiޖ�V��:��_�9\�#)�߆���2��u���U^~K��^��v���`|�8�q�p��P\�^g�������S�x���Bd1gF�����:�� w|Z,�YI6͙���7kU��;�s]���;%�0�E��J�s��ὰI�y��ݳ>�pQ^�����C���l &�Ng�� ��[+�q��y�(��7����M�ǘ�y����������6?��dz��? ( &�"�!� F�
�7M_O�ݶ���FƊ��7*'�$@#HD��?w�DK��DCKI�N�z��%�O�NK"��̆�W�����a*�sQ7����P@-�a��M�H<�/5*���-�e����?>� 6�ɡ�n�h�]�x��N�h=֩������Q���;�,J@$*~��A~�a��
_D/�:�L���_�~!h��ܧ{ĺ�{-o�J!2��\�9,M�]����~��?W�<��CD.��B�I����j��?&�"Ωٴ+W~��G�9���G+@�O�9����[,�?$��m���WF�J��,�%��]e�-/���g�����X��h�o�
o\����2������wF��Sx�����	��q2��M�7X.��Y�B������-+�S����`l\�9���Hi?�E=���q�h��ve����sȣ�D�8>���*��������C�7�W��p/HK���}�֠��2�il�f.Ň"sHα�LE��-�e�u�WY�K�R����(N��B�����Dho5�9�k��4H/N�C}�Ю��p�\#"��,Q�v+Sǻ�ri�b���OJ�@'�=����@�|��b�ѰdLM�Ϳ-�%��I�l�l�����q��](Ơj�L_�Ѡ��V�F��>�!L)������"]�N���^yB�=��c�z��Yv���:Fݏ��k�gi�o�o�Jx���xG��O�ƺ�u�V��4˰C6T�_��W�ّ����̜3��
r%���*�'5�1�$(�,K���2�m���Gz��/�
-���=�i�ʶ��<i�����1<���:6Ἓ "�J�;g�<�_�-��L߷��e��~�w8��G
��	�> v�E�2v�b֑��B�C��͜m���h0����cq��da�Ye�*i�2w�b�|��GΣ]=��܍���E��Y�F�	��{qn �����l�iuf�)��+]vI���R\ڤ�C�K0
�rX�U�Fz���xa��g�߹�
�<j�b���EY���T��B�ɫ.�dj�GM��:޴<R���'��b�Q��I�N�Lj������Y��ũ��z�6nŹ:'.cޤ���V��BDd�L\¹�?�ԟ�SH��0q�u��<`)�&�"�iͽ<�z�,,������G�͊�5�Ba[U��TgGR���3{'�y�cm@�lu�mJ ,�۱�|[��a��K>�guV��t��w�FZ��^o�⻄�'����	�l�y{ņ~���ѯȯ.B&��0�i?]-J:�>'��ݴ�yFSP0��.�dt0�Vfu&�C���Q4	
��86���b�4x9�+�j.�$I��}������M����1�,WeƓ*��'=�����Q�'ЋK�T�{^m�=���f����	���G9��YN>�Xp�tr�]��8Wj��ֳ.ő����M�1��E��%u$�r����?�esxڱ�A��y���O+6�g�$�L��@*���Ҍ����J���.��1wk菩������l-%�8Nk֚89��>��K�	�h^���2�}����
��4�靶ю�\sA��a��DTGJ��5�Q�d6"_��t���3���o9[�e��BS��~�2Yʂ��2�m���>�hE��֨��������{RCX	�ؙ�	���5p;q�\wA׈oT�T^K%��BX�v�Q�� Lj�Y�7�ô��8�\64ԓ��H(7g�ף-D�bp�TR�F�L·S#D��W������=*��d�I��Y���l�������7�tt��[|Y����&��DfbOj�nt:r�݅�� Sxڻ#a�Px�s]�A�J��	@>�"����-�E�;�:�O�yg�Š��R������s`�*^  ��yjg%V�'ЌD e�o�aa#\���j�Ss�瀁l�B���F�Z!Y�XpѰA� M��HX���G��u�_�2�-����i���f����3m�U���wLz.�]�ވ�-�j�f��+�3�ؕ��P����_5�/($&�bɟ�*K���ui"��b�Q���{�n'�nM�I�͓i 윆%̀d���\��W1Ƭj��+&�r����T�.��G�$��s򱽌I��ѬdE�K%�c,��s?]���N���p�)B��|��5�R�Ŷ��|~��Rⅱ�[3���q�"��K�;\�o��֪Y�z>��{��M�	%���	�7�s���a}�<I����~��oz�1��0�eOޢ�}������?�9����#*�� r����Z<�)��R���^�8	��񾁵�O�5���4OoL�u���_���RFxx��Bb��(U�8fRa�_C��2���=Xi��mı�����;�n���_3_{V�ǋ'����poLL�in��Z���?h5֯&�"�a�PN�Za�*𑲠��\��2���a\0|��b�Q�J�OL7�C��ˊ�<��%# ���+@74��,�)�`}�]{�(���~v��_��(���/&�"�a��%�u�N�V�����Y���ȼr���$GcJ��l7U�4Z�C	�t�D{,� �N��4h����Q\�����9�j�W �:�+('f�܎ۥC�#n%�B6'N�"�]t�,d��C'J�˷�2ٷ�.�%=�31>��=�����"��@Mό���a`|>��2�1�pXeb�<Oi������x>��S���:?k�2�_��U�w$*��Y���췏;�4&�T§Tw	D��\�g���OJz!���ۏ$��-m���RM?��i�k$�O�<]����N��~��i��#�|.ØZ҃%�����	�M����)��J�V���j��5�.����RW��A��?����Zu�~�`��_��L6�~�߉���� �F@���{Xzt��r���b�iZ�>9�i��~���i��Hc���I�N�a��y��{!���̈́�H�����z�.sb�ё�7!8+���cB�l`�7-.�n@�H��,��uc�W��D�3�
�P�z�j)�����麸����0��B4Ek?6^  E��`�%M{_hR�p��b	TI���Da�PoDwwV�Y�L��`)o(�{ʝ��G<7J���w���O7r䘨R��L�08X%�D�����g��cP��9�[7Cn��V�<�N����9���A���_@�}p��9>��s�g��a�&$�r��Y������=���Ґ�Q��.A��`OP7�i�V�F�<�l@Ij�͜�������l~�ti9Sot+�p��` [c,��R�E�Kt<'�X�e����>?,G����gum�mDD_�_/D`�jt��c8*[��YB��J�j�W/F�
�wJ�t�ƴ��6�׹�v���`sP�~�)#��he�Rp����=�l���U�I<�\����L���6U���1Qq����+���,�Wu���ߔ`��UD��?N�:@5��h�c�b�^��Lh	��������;����/dr�݇)�C3�p)k+A�Hr���X��TkG3)��x�@D���0=� ���3�-ц�3_b���4\����?�\E�a:�Z�?y���>Ŧ"��ݒ_3��$�W�P;*��7��Q܀~�\'����]�(��8���Z�<9����>��pN��`Q�sy!��v��:p���?Y�K�į��ۥ�&�"��0G&��'P�ꭸz8�P�-��wm�?��<f�����=���H�)�aQ*y�^�!wξ���T	?��ҞK�P_MG�<>�V�ƀSU��Ss����r}�&�o�L�Z��Y��]��Ă󬸈�A��F��B5%C;Ү$ޮz�{s[]�Iګ5��,Z?F{&���Q�� G�53A �<J���:��?7h.��Y�G�ʛ�s�z&f�@����WUv��9p���G0
�?�;6�f�$�_��H��l����M��>�Tʲ��wf��������n�d�0��lX·)���B����a���Z��Y���Y�����b��9%�i�������:���K?XH�����ʪaip #`!� d `5�'jkmn2��16VS4��yet3"�q�ci�5�+'d'����}=F����j)k�#BQ��oN�e��'%�1��+��K�U�]���M��m�������������//d$cu��+j"L!�l$!Cx�.�+*ű%bgF��՟'"�!�`\	�w1g�RQ����](u��4f��u�`^h�Zr��q����]Z��x������&�b���DOCDi=�rͤڼ�:�+$'ch�єdWS ���MBD��$5�o8�#mdΓC٬Ve��9Ũ$k�5���R�z�=�i���&�����ȃ�� t��d1Wg��{tu�#�T�4I`c��26Л�I@����.�2&h�O�f к�FoBx�w��+�"�q~�+���x��Ӯ,^�ĳ'�ϒ�q��W�k�"Q��bX��ΐ���v��S�w��(sE,�Ļ��5�ש?�ww�ZA��HI��/��z%a�q�d��@�m�;f����ĦA���؍�.�k)s5��"h^7/���)&W��oog�*l�'b����.��7��\�T?>�]O�3��b�6��W7��|֢�w���1�� ��͉1������&�-�b�U�AJ|U}���������A5�;$+lmP΄�1$��vc!eD�v���4#�[�B"+[ơ�m�9���w,-�1�k���dJ�,��c��� �HJ�Y���/6�7֓�0ᅸ�'�$ќ�'����!h=�޻��� vn�-.�Cz(�3��l��k��(��փ��R� �U�?�[6.T��������.o�y��gH"�7-��""�h�53��gp�G�Rɪ��=�@�D�����0=�q���ήsEM�lF���ok�N}?��f�b�9�f�?��3��Cp��"���X^��0�`�����mV�O�A��Α3U}�i�����x.�Z�ʼLbWF�P���;L�GA^��!��1��Z��+)�������f��B�Q���0 $������oF�aT@� ��;Vy���A�e�S���:�k-� D�.=��
������z�.8��f����1r���U���Puӟ��q���"��w�ղ1�o���˳"�a2��-[�[(l��*�R+���?2�*���:���C�DH��`�(��lt���=�!�Ž}�Z�F�XI����e-����ϴ�G�,=8�v�Z]9}-��� e����P'04�a�7�4n$�?G_>��I��Ƶ��P~�
�}F�X�@�fޥ�T�7���Ԓ�6�4a���i�SVWFļ��`cۅ�hg+e�S2���V�t�CK��7�&"��?��]:����z�n3���멠AV����b܇�+[�Y�II�ѱ�;[V�Fѣ��N�O;	�lH���GjX���<y��Q�9P�_@�2s�i�5�����]�������Sl��$�-��'B�	�W����:�~����`ڒh�ɇ&i�b6=r��8��!b�2۷4I9�)��[�B����=9������\u���X�N:��GC�q�d���=�h�I����	�i>��迊;�+���!�c`���||�ZX�Ȓ+J9�pk�q��&��96��[�ºq���[��K���c8��5@��B����x��X��!��k�֏�ܚ����t�D�Dt_@aQ���&��b ilG{�\a��E�<~<-L��S�E���'49l6�{�����v{W���W �:ʣFdE�a�Y�ˣ/�i.��!��:�6x���-T�4��!����5
�:���j��["���b���LQ��m�U�.y��~n�Q�V���p�M���L\It��̵��^Ww�Wj�;z{�?e��g´x7Z��Q�!�6�-� 2�}���������H�Ӊ��}ie%'#A�R�,1f�WY>!����lS:��0`,%�#?i�s�� nj��OŹ6����5z*�c�+���+=�.&�������s�\Y��J�|Dd<i���&�*���*{6�|m����J�@|�)�bbk�$��;*q�껴WJӝ��s*к�]�v�c���z�SO� ����-�7)���1��W#)���&�&G��!z�8[�K�ǃ6�{�Z�����'��ٕ����B���|�Q{'��W��gቫtk�{�����j�����⠣Ƞl
�aEŤ\7|.��R�E��`��y�/����:���j���sr(��W8%�9�m^�C�h�WU^��ԁq��2����/1�,sm��Cځ De�A��Z�L������y�/�ؙV����uF�Ɋ�J��tz�ɻA�|t�r��1�,U���W,��ޯ����=�{"��a5JUs�������|(\s�\��XoC�oOcL��m�U���)sn�uˍ���U�U�4h[��/]Q;�؎��*"�&�0[)�tW�z��q����l2��=��t�W��&�1)��u������ ڻg�h��Ksl6Oyˌg����E�\;j�
��thb�рdXB���L�E�<�x�3
䢧����T~��S�6*�_��/�,�y����)��J�����LI����u��rCQL�Q�Q|���wW�����O��]�[7D����f>�!��06��� �u���6�&��8ƶ�[ւ�����]�Ǐ(��`
Qy>��V��)�`^`1�\�r�����_J!����ueJl�*t�D}Dt�NH��x9��`W��_ [��f�\�Jz�K��ʦ��婳����\�m+YiV��rD�� $au��I/�	�U|b��֎>Ц>��Z9\/�eΓQ��A�H['�$D�۬o���RS��#���	���b�䷏�h9��e�T���t|Y��9����/>�(Cf����st�S��HA�=eU � ^���T� ��w^��N�b�щ�^�@i���N�%o��� ���rǑ��m1��U�0�!(�k��S��s����#���8�����r;Цi��.qiQ[�Nc�\�L� �u2 ��������ߪm
�?��5:;l+U�G2����;+w���UW���\K.qZ�{���Y�v[�Clj���8���E��,�_|�,�T��A�(+ЙA���� �^ʻ�6���sA�������z�m�!;]�І@��:v��˷0C�o
��9�/�Blv���=�Y��ώ�}���'ɣ� �A�K��k_�Y[b���G�_�M��%�6Ic�����x�W���U �5"8���ϓ����0�!mv�����= ��B1D�FMG����;�fYZ��! �g���̐Mi�r�B��,�top/�f�瀎�$ʛ�iG'��g#'��3:�[�D?�_+�Z�l��1��T��Ux���Â��?��,��a8i��տe����p^���I��8(q�\���a���S^5���f��!��3N����:�)��'q�^�kQ�����u���Jv��33����@W�9����I�{`�R��� @@�0��,���XP�w=��� �X�jf�M���(J�����_�Ů����?{��[N��	^���B�i��O�<�[>�������������\�:XpFhD��#�ʨ::__6^�a���Ky%G5���T�B��D@kN�l�ɀ�j�bk :"���!J%޼�\aG�HΑ��o�g� s�X��7�/[�&�'���\���0�����H��G������Dx��!��lk��2�h�����,�#C� ����3F[����܋I!}�df�c&��`8�vq��!/��3�W�MSIS_NVIC_VIRTUAL_HEADER_FILE
#else
  #define NVIC_SetPriorityGrouping    __NVIC_SetPriorityGrouping
  #define NVIC_GetPriorityGrouping    __NVIC_GetPriorityGrouping
  #define NVIC_EnableIRQ              __NVIC_EnableIRQ
  #define NVIC_GetEnableIRQ           __NVIC_GetEnableIRQ
  #define NVIC_DisableIRQ             __NVIC_DisableIRQ
  #define NVIC_GetPendingIRQ          __NVIC_GetPendingIRQ
  #define NVIC_SetPendingIRQ          __NVIC_SetPendingIRQ
  #define NVIC_ClearPendingIRQ        __NVIC_ClearPendingIRQ
  #define NVIC_GetActive              __NVIC_GetActive
  #define NVIC_SetPriority            __NVIC_SetPriority
  #define NVIC_GetPriority            __NVIC_GetPriority
  #define NVIC_SystemReset            __NVIC_SystemReset
#endif /* CMSIS_NVIC_VIRTUAL */

#ifdef CMSIS_VECTAB_VIRTUAL
  #ifndef CMSIS_VECTAB_VIRTUAL_HEADER_FILE
    #define CMSIS_VECTAB_VIRTUAL_HEADER_FILE "cmsis_vectab_virtual.h"
  #endif
  #include CMSIS_VECTAB_VIRTUAL_HEADER_FILE
#else
  #define NVIC_SetVector              __NVIC_SetVector
  #define NVIC_GetVector              __NVIC_GetVector
#endif  /* (CMSIS_VECTAB_VIRTUAL) */

#define NVIC_USER_IRQ_OFFSET          16


/* Special LR values for Secure/Non-Secure call handling and exception handling                                               */

/* Function Return Payload (from ARMv8-M Architecture Reference Manual) LR value on entry from Secure BLXNS                   */
#define FNC_RETURN                 (0xFEFFFFFFUL)     /* bit [0] ignored when processing a branch                             */

/* The following EXC_RETURN mask values are used to evaluate the LR on exception entry */
#define EXC_RETURN_PREFIX          (0xFF000000UL)     /* bits [31:24] set to indicate an EXC_RETURN value                     */
#define EXC_RETURN_S               (0x00000040UL)     /* bit [6] stack used to push registers: 0=Non-secure 1=Secure          */
#define EXC_RETURN_DCRS            (0x00000020UL)     /* bit [5] stacking rules for called registers: 0=skipped 1=saved       */
#define EXC_RETURN_FTYPE           (0x00000010UL)     /* bit [4] allocate stack for floating-point context: 0=done 1=skipped  */
#define EXC_RETURN_MODE            (0x00000008UL)     /* bit [3] processor mode for return: 0=Handler mode 1=Thread mode      */
#define EXC_RETURN_SPSEL           (0x00000004UL)     /* bit [2] stack pointer used to restore context: 0=MSP 1=PSP           */
#define EXC_RETURN_ES              (0x00000001UL)     /* bit [0] security state exception was taken to: 0=Non-secure 1=Secure */

/* Integrity Signature (from ARMv8-M Architecture Reference Manual) for exception context stacking                            */
#if defined (__FPU_PRESENT) && (__FPU_PRESENT == 1U)  /* Value for processors with floating-point extension:                  */
#define EXC_INTEGRITY_SIGNATURE     (0xFEFA125AUL)     /* bit [0] SFTC must match LR bit[4] EXC_RETURN_FTYPE                   */
#else
#define EXC_INTEGRITY_SIGNATURE     (0xFEFA125BUL)     /* Value for processors without floating-point extension                */
#endif


/**
  \brief   Set Priority Grouping
  \details Sets the priority grouping field using the required unlock sequence.
           The parameter PriorityGroup is assigned to the field SCB->AIRCR [10:8] PRIGROUP field.
           Only values from 0..7 are used.
           In case of a conflict between priority grouping and available
           priority bits (__NVIC_PRIO_BITS), the smallest possible priority group is set.
  \param [in]      PriorityGroup  Priority grouping field.
 */
__STATIC_INLINE void __NVIC_SetPriorityGrouping(uint32_t PriorityGroup)
{
  uint32_t reg_value;
  uint32_t PriorityGroupTmp = (PriorityGroup & (uint32_t)0x07UL);             /* only values 0..7 are used          */

  reg_value  =  SCB->AIRCR;                                                   /* read old register configuration    */
  reg_value &= ~((uint32_t)(SCB_AIRCR_VECTKEY_Msk | SCB_AIRCR_PRIGROUP_Msk)); /* clear bits to change               */
  reg_value  =  (reg_value                                   |
                ((uint32_t)0x5FAUL << SCB_AIRCR_VECTKEY_Pos) |
                (PriorityGroupTmp << SCB_AIRCR_PRIGROUP_Pos)  );              /* Insert write key and priority group */
  SCB->AIRCR =  reg_value;
}


/**
  \brief   Get Priority Grouping
  \details Reads the priority grouping field from the NVIC Interrupt Controller.
  \return                Priority grouping field (SCB->AIRCR [10:8] PRIGROUP field).
 */
__STATIC_INLINE uint32_t __NVIC_GetPriorityGrouping(void)
{
  return ((uint32_t)((SCB->AIRCR & SCB_AIRCR_PRIGROUP_Msk) >> SCB_AIRCR_PRIGROUP_Pos));
}


/**
  \brief   Enable Interrupt
  \details Enables a device specific interrupt in the NVIC interrupt controller.
  \param [in]      IRQn  Device specific interrupt number.
  \note    IRQn must not be negative.
 */
__STATIC_INLINE void __NVIC_EnableIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    __COMPILER_BARRIER();
    NVIC->ISER[(((uint32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)IRQn) & 0x1FUL));
    __COMPILER_BARRIER();
  }
}


/**
  \brief   Get Interrupt Enable status
  \details Returns a device specific interrupt enable status from the NVIC interrupt controller.
  \param [in]      IRQn  Device specific interrupt number.
  \return             0  Interrupt is not enabled.
  \return             1  Interrupt is enabled.
  \note    IRQn must not be negative.
 */
__STATIC_INLINE uint32_t __NVIC_GetEnableIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    return((uint32_t)(((NVIC->ISER[(((uint32_t)IRQn) >> 5UL)] & (1UL << (((uint32_t)IRQn) & 0x1FUL))) != 0UL) ? 1UL : 0UL));
  }
  else
  {
    return(0U);
  }
}


/**
  \brief   Disable Interrupt
  \details Disables a device specific interrupt in the NVIC interrupt controller.
  \param [in]      IRQn  Device specific interrupt number.
  \note    IRQn must not be negative.
 */
__STATIC_INLINE void __NVIC_DisableIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    NVIC->ICER[(((uint32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)IRQn) & 0x1FUL));
    __DSB();
    __ISB();
  }
}


/**
  \brief   Get Pending Interrupt
  \details Reads the NVIC pending register and returns the pending bit for the specified device specific interrupt.
  \param [in]      IRQn  Device specific interrupt number.
  \return             0  Interrupt status is not pending.
  \return             1  Interrupt status is pending.
  \note    IRQn must not be negative.
 */
__STATIC_INLINE uint32_t __NVIC_GetPendingIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    return((uint32_t)(((NVIC->ISPR[(((uint32_t)IRQn) >> 5UL)] & (1UL << (((uint32_t)IRQn) & 0x1FUL))) != 0UL) ? 1UL : 0UL));
  }
  else
  {
    return(0U);
  }
}


/**
  \brief   Set Pending Interrupt
  \details Sets the pending bit of a device specific interrupt in the NVIC pending register.
  \param [in]      IRQn  Device specific interrupt number.
  \note    IRQn must not be negative.
 */
__STATIC_INLINE void __NVIC_SetPendingIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    NVIC->ISPR[(((uint32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)IRQn) & 0x1FUL));
  }
}


/**
  \brief   Clear Pending Interrupt
  \details Clears the pending bit of a device specific interrupt in the NVIC pending register.
  \param [in]      IRQn  Device specific interrupt number.
  \note    IRQn must not be negative.
 */
__STATIC_INLINE void __NVIC_ClearPendingIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    NVIC->ICPR[(((uint32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)IRQn) & 0x1FUL));
  }
}


/**
  \brief   Get Active Interrupt
  \details Reads the active register in the NVIC and returns the active bit for the device specific interrupt.
  \param [in]      IRQn  Device specific interrupt number.
  \return             0  Interrupt status is not active.
  \return             1  Interrupt status is active.
  \note    IRQn must not be negative.
 */
__STATIC_INLINE uint32_t __NVIC_GetActive(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    return((uint32_t)(((NVIC->IABR[(((uint32_t)IRQn) >> 5UL)] & (1UL << (((uint32_t)IRQn) & 0x1FUL))) != 0UL) ? 1UL : 0UL));
  }
  else
  {
    return(0U);
  }
}


#if defined (__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3U)
/**
  \brief   Get Interrupt Target State
  \details Reads the interrupt target field in the NVIC and returns the interrupt target bit for the device specific interrupt.
  \param [in]      IRQn  Device specific interrupt number.
  \return             0  if interrupt is assigned to Secure
  \return             1  if interrupt is assigned to Non Secure
  \note    IRQn must not be negative.
 */
__STATIC_INLINE uint32_t NVIC_GetTargetState(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    return((uint32_t)(((NVIC->ITNS[(((uint32_t)IRQn) >> 5UL)] & (1UL << (((uint32_t)IRQn) & 0x1FUL))) != 0UL) ? 1UL : 0UL));
  }
  else
  {
    return(0U);
  }
}


/**
  \brief   Set Interrupt Target State
  \details Sets the interrupt target field in the NVIC and returns the interrupt target bit for the device specific interrupt.
  \param [in]      IRQn  Device specific interrupt number.
  \return             0  if interrupt is assigned to Secure
                      1  if interrupt is assigned to Non Secure
  \note    IRQn must not be negative.
 */
__STATIC_INLINE uint32_t NVIC_SetTargetState(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    NVIC->ITNS[(((uint32_t)IRQn) >> 5UL)] |=  ((uint32_t)(1UL << (((uint32_t)IRQn) & 0x1FUL)));
    return((uint32_t)(((NVIC->ITNS[(((uint32_t)IRQn) >> 5UL)] & (1UL << (((uint32_t)IRQn) & 0x1FUL))) != 0UL) ? 1UL : 0UL));
  }
  else
  {
    return(0U);
  }
}


/**
  \brief   Clear Interrupt Target State
  \details Clears the interrupt target field in the NVIC and returns the interrupt target bit for the device specific interrupt.
  \param [in]      IRQn  Device specific interrupt number.
  \return             0  if interrupt is assigned to Secure
                      1  if interrupt is assigned to Non Secure
  \note    IRQn must not be negative.
 */
__STATIC_INLINE uint32_t NVIC_ClearTargetState(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    NVIC->ITNS[(((uint32_t)IRQn) >> 5UL)] &= ~((uint32_t)(1UL << (((uint32_t)IRQn) & 0x1FUL)));
    return((uint32_t)(((NVIC->ITNS[(((uint32_t)IRQn) >> 5UL)] & (1UL << (((uint32_t)IRQn) & 0x1FUL))) != 0UL) ? 1UL : 0UL));
  }
  else
  {
    return(0U);
  }
}
#endif /* defined (__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3U) */


/**
  \brief   Set Interrupt Priority
  \details Sets the priority of a device specific interrupt or a processor exception.
           The interrupt number can be positive to specify a device specific interrupt,
           or negative to specify a processor exception.
  \param [in]      IRQn  Interrupt number.
  \param [in]  priority  Priority to set.
  \note    The priority cannot be set for every processor exception.
 */
__STATIC_INLINE void __NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority)
{
  if ((int32_t)(IRQn) >= 0)
  {
    NVIC->IPR[((uint32_t)IRQn)]               = (uint8_t)((priority << (8U - __NVIC_PRIO_BITS)) & (uint32_t)0xFFUL);
  }
  else
  {
    SCB->SHPR[(((uint32_t)IRQn) & 0xFUL)-4UL] = (uint8_t)((priority << (8U - __NVIC_PRIO_BITS)) & (uint32_t)0xFFUL);
  }
}


/**
  \brief   Get Interrupt Priority
  \details Reads the priority of a device specific interrupt or a processor exception.
           The interrupt number can be positive to specify a device specific interrupt,
           or negative to specify a processor exception.
  \param [in]   IRQn  Interrupt number.
  \return             Interrupt Priority.
                      Value is aligned automatically to the implemented priority bits of the microcontroller.
 */
__STATIC_INLINE uint32_t __NVIC_GetPriority(IRQn_Type IRQn)
{

  if ((int32_t)(IRQn) >= 0)
  {
    return(((uint32_t)NVIC->IPR[((uint32_t)IRQn)]               >> (8U - __NVIC_PRIO_BITS)));
  }
  else
  {
    return(((uint32_t)SCB->SHPR[(((uint32_t)IRQn) & 0xFUL)-4UL] >> (8U - __NVIC_PRIO_BITS)));
  }
}


/**
  \brief   Encode Priority
  \details Encodes the priority for an interrupt with the given priority group,
           preemptive priority value, and subpriority value.
           In case of a conflict between priority grouping and available
           priority bits (__NVIC_PRIO_BITS), the smallest possible priority group is set.
  \param [in]     PriorityGroup  Used priority group.
  \param [in]   PreemptPriority  Preemptive priority value (starting from 0).
  \param [in]       SubPriority  Subpriority value (starting from 0).
  \return                        Encoded priority. Value can be used in the function \ref NVIC_SetPriority().
 */
__STATIC_INLINE uint32_t NVIC_EncodePriority (uint32_t PriorityGroup, uint32_t PreemptPriority, uint32_t SubPriority)
{
  uint32_t PriorityGroupTmp = (PriorityGroup & (uint32_t)0x07UL);   /* only values 0..7 are used          */
  uint32_t PreemptPriorityBits;
  uint32_t SubPriorityBits;

  PreemptPriorityBits = ((7UL - PriorityGroupTmp) > (uint32_t)(__NVIC_PRIO_BITS)) ? (uint32_t)(__NVIC_PRIO_BITS) : (uint32_t)(7UL - PriorityGroupTmp);
  SubPriorityBits     = ((PriorityGroupTmp + (uint32_t)(__NVIC_PRIO_BITS)) < (uint32_t)7UL) ? (uint32_t)0UL : (uint32_t)((PriorityGroupTmp - 7UL) + (uint32_t)(__NVIC_PRIO_BITS));

  return (
           ((PreemptPriority & (uint32_t)((1UL << (PreemptPriorityBits)) - 1UL)) << SubPriorityBits) |
           ((SubPriority     & (uint32_t)((1UL << (SubPriorityBits    )) - 1UL)))
         );
}


/**
  \brief   Decode Priority
  \details Decodes an interrupt priority value with a given priority group to
           preemptive priority value and subpriority value.
           In case of a conflict between priority grouping and available
           priority bits (__NVIC_PRIO_BITS) the smallest possible priority group is set.
  \param [in]         Priority   Priority value, which can be retrieved with the function \ref NVIC_GetPriority().
  \param [in]     PriorityGroup  Used priority group.
  \param [out] pPreemptPriority  Preemptive priority value (starting from 0).
  \param [out]     pSubPriority  Subpriority value (starting from 0).
 */
__STATIC_INLINE void NVIC_DecodePriority (uint32_t Priority, uint32_t PriorityGroup, uint32_t* const pPreemptPriority, uint32_t* const pSubPriority)
{
  uint32_t PriorityGroupTmp = (PriorityGroup & (uint32_t)0x07UL);   /* only values 0..7 are used          */
  uint32_t PreemptPriorityBits;
  uint32_t SubPriorityBits;

  PreemptPriorityBits = ((7UL - PriorityGroupTmp) > (uint32_t)(__NVIC_PRIO_BITS)) ? (uint32_t)(__NVIC_PRIO_BITS) : (uint32_t)(7UL - PriorityGroupTmp);
  SubPriorityBits     = ((PriorityGroupTmp + (uint32_t)(__NVIC_PRIO_BITS)) < (uint32_t)7UL) ? (uint32_t)0UL : (uint32_t)((PriorityGroupTmp - 7UL) + (uint32_t)(__NVIC_PRIO_BITS));

  *pPreemptPriority = (Priority >> SubPriorityBits) & (uint32_t)((1UL << (PreemptPriorityBits)) - 1UL);
  *pSubPriority     = (Priority                   ) & (uint32_t)((1UL << (SubPriorityBits    )) - 1UL);
}


/**
  \brief   Set Interrupt Vector
  \details Sets an interrupt vector in SRAM based interrupt vector table.
           The interrupt number can be positive to specify a device specific interrupt,
           or negative to specify a processor exception.
           VTOR must been relocated to SRAM before.
  \param [in]   IRQn      Interrupt number
  \param [in]   vector    Address of interrupt handler function
 */
__STATIC_INLINE void __NVIC_SetVector(IRQn_Type IRQn, uint32_t vector)
{
  uint32_t *vectors = (uint32_t *)SCB->VTOR;
  vectors[(int32_t)IRQn + NVIC_USER_IRQ_OFFSET] = vector;
  __DSB();
}


/**
  \brief   Get Interrupt Vector
  \details Reads an interrupt vector from interrupt vector table.
           The interrupt number can be positive to specify a device specific interrupt,
           or negative to specify a processor exception.
  \param [in]   IRQn      Interrupt number.
  \return                 Address of interrupt handler function
 */
__STATIC_INLINE uint32_t __NVIC_GetVector(IRQn_Type IRQn)
{
  uint32_t *vectors = (uint32_t *)SCB->VTOR;
  return vectors[(int32_t)IRQn + NVIC_USER_IRQ_OFFSET];
}


/**
  \brief   System Reset
  \details Initiates a system reset request to reset the MCU.
 */
__NO_RETURN __STATIC_INLINE void __NVIC_SystemReset(void)
{
  __DSB();                                                          /* Ensure all outstanding memory accesses included
                                                                       buffered write are completed before reset */
  SCB->AIRCR  = (uint32_t)((0x5FAUL << SCB_AIRCR_VECTKEY_Pos)    |
                           (SCB->AIRCR & SCB_AIRCR_PRIGROUP_Msk) |
                            SCB_AIRCR_SYSRESETREQ_Msk    );         /* Keep priority group unchanged */
  __DSB();                                                          /* Ensure completion of memory access */

  for(;;)                                                           /* wait until reset */
  {
    __NOP();
  }
}

#if defined (__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3U)
/**
  \brief   Set Priority Grouping (non-secure)
  \details Sets the non-secure priority grouping field when in secure state using the required unlock sequence.
           The parameter PriorityGroup is assigned to the field SCB->AIRCR [10:8] PRIGROUP field.
           Only values from 0..7 are used.
           In case of a conflict between priority grouping and available
           priority bits (__NVIC_PRIO_BITS), the smallest possible priority group is set.
  \param [in]      PriorityGroup  Priority grouping field.
 */
__STATIC_INLINE void TZ_NVIC_SetPriorityGrouping_NS(uint32_t PriorityGroup)
{
  uint32_t reg_value;
  uint32_t PriorityGroupTmp = (PriorityGroup & (uint32_t)0x07UL);             /* only values 0..7 are used          */

  reg_value  =  SCB_NS->AIRCR;                                                   /* read old register configuration    */
  reg_value &= ~((uint32_t)(SCB_AIRCR_VECTKEY_Msk | SCB_AIRCR_PRIGROUP_Msk));             /* clear bits to change               */
  reg_value  =  (reg_value                                   |
                ((uint32_t)0x5FAUL << SCB_AIRCR_VECTKEY_Pos) |
                (PriorityGroupTmp << SCB_AIRCR_PRIGROUP_Pos)                      );              /* Insert write key and priority group */
  SCB_NS->AIRCR =  reg_value;
}


/**
  \brief   Get Priority Grouping (non-secure)
  \details Reads the priority grouping field from the non-secure NVIC when in secure state.
  \return                Priority grouping field (SCB->AIRCR [10:8] PRIGROUP field).
 */
__STATIC_INLINE uint32_t TZ_NVIC_GetPriorityGrouping_NS(void)
{
  return ((uint32_t)((SCB_NS->AIRCR & SCB_AIRCR_PRIGROUP_Msk) >> SCB_AIRCR_PRIGROUP_Pos));
}


/**
  \brief   Enable Interrupt (non-secure)
  \details Enables a device specific interrupt in the non-secure NVIC interrupt controller when in secure state.
  \param [in]      IRQn  Device specific interrupt number.
  \note    IRQn must not be negative.
 */
__STATIC_INLINE void TZ_NVIC_EnableIRQ_NS(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    NVIC_NS->ISER[(((uint32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)IRQn) & 0x1FUL));
  }
}


/**
  \brief   Get Interrupt Enable status (non-secure)
  \details Returns a device specific interrupt enable status from the non-secure NVIC interrupt controller when in secure state.
  \param [in]      IRQn  Device specific interrupt number.
  \return             0  Interrupt is not enabled.
  \return             1  Interrupt is enabled.
  \note    IRQn must not be negative.
 */
__STATIC_INLINE uint32_t TZ_NVIC_GetEnableIRQ_NS(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    return((uint32_t)(((NVIC_NS->ISER[(((uint32_t)IRQn) >> 5UL)] & (1UL << (((uint32_t)IRQn) & 0x1FUL))) != 0UL) ? 1UL : 0UL));
  }
  else
  {
    return(0U);
  }
}


/**
  \brief   Disable Interrupt (non-secure)
  \details Disables a device specific interrupt in the non-secure NVIC interrupt controller when in secure state.
  \param [in]      IRQn  Device specific interrupt number.
  \note    IRQn must not be negative.
 */
__STATIC_INLINE void TZ_NVIC_DisableIRQ_NS(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    NVIC_NS->ICER[(((uint32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)IRQn) & 0x1FUL));
  }
}


/**
  \brief   Get Pending Interrupt (non-secure)
  \details Reads the NVIC pending register in the non-secure NVIC when in secure state and returns the pending bit for the specified device specific interrupt.
  \param [in]      IRQn  Device specific interrupt number.
  \return             0  Interrupt status is not pending.
  \return             1  Interrupt status is pending.
  \note    IRQn must not be negative.
 */
__STATIC_INLINE uint32_t TZ_NVIC_GetPendingIRQ_NS(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    return((uint32_t)(((NVIC_NS->ISPR[(((uint32_t)IRQn) >> 5UL)] & (1UL << (((uint32_t)IRQn) & 0x1FUL))) != 0UL) ? 1UL : 0UL));
  }
  else
  {
    return(0U);
  }
}


/**
  \brief   Set Pending Interrupt (non-secure)
  \details Sets the pending bit of a device specific interrupt in the non-secure NVIC pending register when in secure state.
  \param [in]      IRQn  Device specific interrupt number.
  \note    IRQn must not be negative.
 */
__STATIC_INLINE void TZ_NVIC_SetPendingIRQ_NS(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    NVIC_NS->ISPR[(((uint32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)IRQn) & 0x1FUL));
  }
}


/**
  \brief   Clear Pending Interrupt (non-secure)
  \details Clears the pending bit of a device specific interrupt in the non-secure NVIC pending register when in secure state.
  \param [in]      IRQn  Device specific interrupt number.
  \note    IRQn must not be negative.
 */
__STATIC_INLINE void TZ_NVIC_ClearPendingIRQ_NS(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    NVIC_NS->ICPR[(((uint32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)IRQn) & 0x1FUL));
  }
}


/**
  \brief   Get Active Interrupt (non-secure)
  \details Reads the active register in non-secure NVIC when in secure state and returns the active bit for the device specific interrupt.
  \param [in]      IRQn  Device specific interrupt number.
  \return             0  Interrupt status is not active.
  \return             1  Interrupt status is active.
  \note    IRQn must not be negative.
 */
__STATIC_INLINE uint32_t TZ_NVIC_GetActive_NS(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    return((uint32_t)(((NVIC_NS->IABR[(((uint32_t)IRQn) >> 5UL)] & (1UL << (((uint32_t)IRQn) & 0x1FUL))) != 0UL) ? 1UL : 0UL));
  }
  else
  {
    return(0U);
  }
}


/**
  \brief   Set Interrupt Priority (non-secure)
  \details Sets the priority of a non-secure device specific interrupt or a non-secure processor exception when in secure state.
           The interrupt number can be positive to specify a device specific interrupt,
           or negative to specify a processor exception.
  \param [in]      IRQn  Interrupt number.
  \param [in]  priority  Priority to set.
  \note    The priority cannot be set for every non-secure processor exception.
 */
__STATIC_INLINE void TZ_NVIC_SetPriority_NS(IRQn_Type IRQn, uint32_t priority)
{
  if ((int32_t)(IRQn) >= 0)
  {
    NVIC_NS->IPR[((uint32_t)IRQn)]               = (uint8_t)((priority << (8U - __NVIC_PRIO_BITS)) & (uint32_t)0xFFUL);
  }
  else
  {
    SCB_NS->SHPR[(((uint32_t)IRQn) & 0xFUL)-4UL] = (uint8_t)((priority << (8U - __NVIC_PRIO_BITS)) & (uint32_t)0xFFUL);
  }
}


/**
  \brief   Get Interrupt Priority (non-secure)
  \details Reads the priority of a non-secure device specific interrupt or a non-secure processor exception when in secure state.
           The interrupt number can be positive to specify a device specific interrupt,
           or negative to specify a processor exception.
  \param [in]   IRQn  Interrupt number.
  \return             Interrupt Priority. Value is aligned automatically to the implemented priority bits of the microcontroller.
 */
__STATIC_INLINE uint32_t TZ_NVIC_GetPriority_NS(IRQn_Type IRQn)
{

  if ((int32_t)(IRQn) >= 0)
  {
    return(((uint32_t)NVIC_NS->IPR[((uint32_t)IRQn)]               >> (8U - __NVIC_PRIO_BITS)));
  }
  else
  {
    return(((uint32_t)SCB_NS->SHPR[(((uint32_t)IRQn) & 0xFUL)-4UL] >> (8U - __NVIC_PRIO_BITS)));
  }
}
#endif /*  defined (__ARM_FEATURE_CMSE) &&(__ARM_FEATURE_CMSE == 3U) */

/*@} end of CMSIS_Core_NVICFunctions */

/* ##########################  MPU functions  #################################### */

#if defined (__MPU_PRESENT) && (__MPU_PRESENT == 1U)

#include "mpu_armv8.h"

#endif

/* ##########################  PMU functions and events  #################################### */

#if defined (__PMU_PRESENT) && (__PMU_PRESENT == 1U)

#include "pmu_armv8.h"

/**
  \brief   Cortex-M55 PMU events
  \note    Architectural PMU events can be found in pmu_armv8.h
*/

#define ARMCM55_PMU_ECC_ERR                          0xC000             /*!< Any ECC error */
#define ARMCM55_PMU_ECC_ERR_FATAL                    0xC001             /*!< Any fatal ECC error */
#define ARMCM55_PMU_ECC_ERR_DCACHE                   0xC010             /*!< Any ECC error in the data cache */
#define ARMCM55_PMU_ECC_ERR_ICACHE                   0xC011             /*!< Any ECC error in the instruction cache */
#define ARMCM55_PMU_ECC_ERR_FATAL_DCACHE             0xC012             /*!< Any fatal ECC error in the data cache */
#define ARMCM55_PMU_ECC_ERR_FATAL_ICACHE             0xC013             /*!< Any fatal ECC error in the instruction cache*/
#define ARMCM55_PMU_ECC_ERR_DTCM                     0xC020             /*!< Any ECC error in the DTCM */
#define ARMCM55_PMU_ECC_ERR_ITCM                     0xC021             /*!< Any ECC error in the ITCM */
#define ARMCM55_PMU_ECC_ERR_FATAL_DTCM               0xC022             /*!< Any fatal ECC error in the DTCM */
#define ARMCM55_PMU_ECC_ERR_FATAL_ITCM               0xC023             /*!< Any fatal ECC error in the ITCM */
#define ARMCM55_PMU_PF_LINEFILL                      0xC100             /*!< A prefetcher starts a line-fill */
#define ARMCM55_PMU_PF_CANCEL                        0xC101             /*!< A prefetcher stops prefetching */
#define ARMCM55_PMU_PF_DROP_LINEFILL                 0xC102             /*!< A linefill triggered by a prefetcher has been dropped because of lack of buffering */
#define ARMCM55_PMU_NWAMODE_ENTER                    0xC200             /*!< No write-allocate mode entry */
#define ARMCM55_PMU_NWAMODE                          0xC201             /*!< Write-allocate store is not allocated into the data cache due to no-write-allocate mode */
#define ARMCM55_PMU_SAHB_ACCESS                      0xC300             /*!< Read or write access on the S-AHB interface to the TCM */
#define ARMCM55_PMU_PAHB_ACCESS                      0xC301             /*!< Read or write access to the P-AHB write interface */
#define ARMCM55_PMU_AXI_WRITE_ACCESS                 0xC302             /*!< Any beat access to M-AXI write interface */
#define ARMCM55_PMU_AXI_READ_ACCESS                  0xC303             /*!< Any beat access to M-AXI read interface */
#define ARMCM55_PMU_DOSTIMEOUT_DOUBLE                0xC400             /*!< Denial of Service timeout has fired twice and caused buffers to drain to allow forward progress */
#define ARMCM55_PMU_DOSTIMEOUT_TRIPLE                0xC401             /*!< Denial of Service timeout has fired three times and blocked the LSU to force forward progress */

#endif

/* ##########################  FPU functions  #################################### */
/**
  \ingroup  CMSIS_Core_FunctionInterface
  \defgroup CMSIS_Core_FpuFunctions FPU Functions
  \brief    Function that provides FPU type.
  @{
 */

/**
  \brief   get FPU type
  \details returns the FPU type
  \returns
   - \b  0: No FPU
   - \b  1: Single precision FPU
   - \b  2: Double + Single precision FPU
 */
__STATIC_INLINE uint32_t SCB_GetFPUType(void)
{
  uint32_t mvfr0;

  mvfr0 = FPU->MVFR0;
  if      ((mvfr0 & (FPU_MVFR0_FPSP_Msk | FPU_MVFR0_FPDP_Msk)) == 0x220U)
  {
    return 2U;           /* Double + Single precision FPU */
  }
  else if ((mvfr0 & (FPU_MVFR0_FPSP_Msk | FPU_MVFR0_FPDP_Msk)) == 0x020U)
  {
    return 1U;           /* Single precision FPU */
  }
  else
  {
    return 0U;           /* No FPU */
  }
}


/*@} end of CMSIS_Core_FpuFunctions */

/* ##########################  MVE functions  #################################### */
/**
  \ingroup  CMSIS_Core_FunctionInterface
  \defgroup CMSIS_Core_MveFunctions MVE Functions
  \brief    Function that provides MVE type.
  @{
 */

/**
  \brief   get MVE type
  \details returns the MVE type
  \returns
   - \b  0: No Vector Extension (MVE)
   - \b  1: Integer Vector Extension (MVE-I)
   - \b  2: Floating-point Vector Extension (MVE-F)
 */
__STATIC_INLINE uint32_t SCB_GetMVEType(void)
{
  const uint32_t mvfr1 = FPU->MVFR1;
  if      ((mvfr1 & FPU_MVFR1_MVE_Msk) == (0x2U << FPU_MVFR1_MVE_Pos))
  {
    return 2U;
  }
  else if ((mvfr1 & FPU_MVFR1_MVE_Msk) == (0x1U << FPU_MVFR1_MVE_Pos))
  {
    return 1U;
  }
  else
  {
    return 0U;
  }
}


/*@} end of CMSIS_Core_MveFunctions */


/* ##########################  Cache functions  #################################### */

#if ((defined (__ICACHE_PRESENT) && (__ICACHE_PRESENT == 1U)) || \
     (defined (__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)))
#include "cachel1_armv7.h"
#endif


/* ##########################   SAU functions  #################################### */
/**
  \ingroup  CMSIS_Core_FunctionInterface
  \defgroup CMSIS_Core_SAUFunctions SAU Functions
  \brief    Functions that configure the SAU.
  @{
 */

#if defined (__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3U)

/**
  \brief   Enable SAU
  \details Enables the Security Attribution Unit (SAU).
 */
__STATIC_INLINE void TZ_SAU_Enable(void)
{
    SAU->CTRL |=  (SAU_CTRL_ENABLE_Msk);
}



/**
  \brief   Disable SAU
  \details Disables the Security Attribution Unit (SAU).
 */
__STATIC_INLINE void TZ_SAU_Disable(void)
{
    SAU->CTRL &= ~(SAU_CTRL_ENABLE_Msk);
}

#endif /* defined (__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3U) */

/*@} end of CMSIS_Core_SAUFunctions */




/* ##################################    Debug Control function  ############################################ */
/**
  \ingroup  CMSIS_Core_FunctionInterface
  \defgroup CMSIS_Core_DCBFunctions Debug Control Functions
  \brief    Functions that access the Debug Control Block.
  @{
 */


/**
  \brief   Set Debug Authentication Control Register
  \details writes to Debug Authentication Control register.
  \param [in]  value  value to be writen.
 */
__STATIC_INLINE void DCB_SetAuthCtrl(uint32_t value)
{
    __DSB();
    __ISB();
    DCB->DAUTHCTRL = value;
    __DSB();
    __ISB();
}


/**
  \brief   Get Debug Authentication Control Register
  \details Reads Debug Authentication Control register.
  \return             Debug Authentication Control Register.
 */
__STATIC_INLINE uint32_t DCB_GetAuthCtrl(void)
{
    return (DCB->DAUTHCTRL);
}


#if defined (__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3U)
/**
  \brief   Set Debug Authentication Control Register (non-secure)
  \details writes to non-secure Debug Authentication Control register when in secure state.
  \param [in]  value  value to be writen
 */
__STATIC_INLINE void TZ_DCB_SetAuthCtrl_NS(uint32_t value)
{
    __DSB();
    __ISB();
    DCB_NS->DAUTHCTRL = value;
    __DSB();
    __ISB();
}


/**
  \brief   Get Debug Authentication Control Register (non-secure)
  \details Reads non-secure Debug Authentication Control register when in secure state.
  \return             Debug Authentication Control Register.
 */
__STATIC_INLINE uint32_t TZ_DCB_GetAuthCtrl_NS(void)
{
    return (DCB_NS->DAUTHCTRL);
}
#endif /* defined (__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3U) */

/*@} end of CMSIS_Core_DCBFunctions */




/* ##################################    Debug Identification function  ############################################ */
/**
  \ingroup  CMSIS_Core_FunctionInterface
  \defgroup CMSIS_Core_DIBFunctions Debug Identification Functions
  \brief    Functions that access the Debug Identification Block.
  @{
 */


/**
  \brief   Get Debug Authentication Status Register
  \details Reads Debug Authentication Status register.
  \return             Debug Authentication Status Register.
 */
__STATIC_INLINE uint32_t DIB_GetAuthStatus(void)
{
    return (DIB->DAUTHSTATUS);
}


#if defined (__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3U)
/**
  \brief   Get Debug Authentication Status Register (non-secure)
  \details Reads non-secure Debug Authentication Status register when in secure state.
  \return             Debug Authentication Status Register.
 */
__STATIC_INLINE uint32_t TZ_DIB_GetAuthStatus_NS(void)
{
    return (DIB_NS->DAUTHSTATUS);
}
#endif /* defined (__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3U) */

/*@} end of CMSIS_Core_DCBFunctions */




/* ##################################    SysTick function  ############################################ */
/**
  \ingroup  CMSIS_Core_FunctionInterface
  \defgroup CMSIS_Core_SysTickFunctions SysTick Functions
  \brief    Functions that configure the System.
  @{
 */

#if defined (__Vendor_SysTickConfig) && (__Vendor_SysTickConfig == 0U)

/**
  \brief   System Tick Configuration
  \details Initializes the System Timer and its interrupt, and starts the System Tick Timer.
           Counter is in free running mode to generate periodic interrupts.
  \param [in]  ticks  Number of ticks between two interrupts.
  \return          0  Function succeeded.
  \return          1  Function failed.
  \note    When the variable <b>__Vendor_SysTickConfig</b> is set to 1, then the
           function <b>SysTick_Config</b> is not included. In this case, the file <b><i>device</i>.h</b>
           must contain a vendor-specific implementation of this function.
 */
__STATIC_INLINE uint32_t SysTick_Config(uint32_t ticks)
{
  if ((ticks - 1UL) > SysTick_LOAD_RELOAD_Msk)
  {
    return (1UL);                                                   /* Reload value impossible */
  }

  SysTick->LOAD  = (uint32_t)(ticks - 1UL);                         /* set reload register */
  NVIC_SetPriority (SysTick_IRQn, (1UL << __NVIC_PRIO_BITS) - 1UL); /* set Priority for Systick Interrupt */
  SysTick->VAL   = 0UL;                                             /* Load the SysTick Counter Value */
  SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk |
                   SysTick_CTRL_TICKINT_Msk   |
                   SysTick_CTRL_ENABLE_Msk;                         /* Enable SysTick IRQ and SysTick Timer */
  return (0UL);                                                     /* Function successful */
}

#if defined (__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3U)
/**
  \brief   System Tick Configuration (non-secure)
  \details Initializes the non-secure System Timer and its interrupt when in secure state, and starts the System Tick Timer.
           Counter is in free running mode to generate periodic interrupts.
  \param [in]  ticks  Number of ticks between two interrupts.
  \return          0  Function succeeded.
  \return          1  Function failed.
  \note    When the variable <b>__Vendor_SysTickConfig</b> is set to 1, then the
           function <b>TZ_SysTick_Config_NS</b> is not included. In this case, the file <b><i>device</i>.h</b>
           must contain a vendor-specific implementation of this function.

 */
__STATIC_INLINE uint32_t TZ_SysTick_Config_NS(uint32_t ticks)
{
  if ((ticks - 1UL) > SysTick_LOAD_RELOAD_Msk)
  {
    return (1UL);                                                         /* Reload value impossible */
  }

  SysTick_NS->LOAD  = (uint32_t)(ticks - 1UL);                            /* set reload register */
  TZ_NVIC_SetPriority_NS (SysTick_IRQn, (1UL << __NVIC_PRIO_BITS) - 1UL); /* set Priority for Systick Interrupt */
  SysTick_NS->VAL   = 0UL;                                                /* Load the SysTick Counter Value */
  SysTick_NS->CTRL  = SysTick_CTRL_CLKSOURCE_Msk |
                      SysTick_CTRL_TICKINT_Msk   |
                      SysTick_CTRL_ENABLE_Msk;                            /* Enable SysTick IRQ and SysTick Timer */
  return (0UL);                                                           /* Function successful */
}
#endif /* defined (__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3U) */

#endif

/*@} end of CMSIS_Core_SysTickFunctions */



/* ##################################### Debug In/Output function ########################################### */
/**
  \ingroup  CMSIS_Core_FunctionInterface
  \defgroup CMSIS_core_DebugFunctions ITM Functions
  \brief    Functions that access the ITM debug interface.
  @{
 */

extern volatile int32_t ITM_RxBuffer;                              /*!< External variable to receive characters. */
#define                 ITM_RXBUFFER_EMPTY  ((int32_t)0x5AA55AA5U) /*!< Value identifying \ref ITM_RxBuffer is ready for next character. */


/**
  \brief   ITM Send Character
  \details Transmits a character via the ITM channel 0, and
           \li Just returns when no debugger is connected that has booked the output.
           \li Is blocking when a debugger is connected, but the previous character sent has not been transmitted.
  \param [in]     ch  Character to transmit.
  \returns            Character to transmit.
 */
__STATIC_INLINE uint32_t ITM_SendChar (uint32_t ch)
{
  if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
      ((ITM->TER & 1UL               ) != 0UL)   )     /* ITM Port #0 enabled */
  {
    while (ITM->PORT[0U].u32 == 0UL)
    {
      __NOP();
    }
    ITM->PORT[0U].u8 = (uint8_t)ch;
  }
  return (ch);
}


/**
  \brief   ITM Receive Character
  \details Inputs a character via the external variable \ref ITM_RxBuffer.
  \return             Received character.
  \return         -1  No character pending.
 */
__STATIC_INLINE int32_t ITM_ReceiveChar (void)
{
  int32_t ch = -1;                           /* no character available */

  if (ITM_RxBuffer != ITM_RXBUFFER_EMPTY)
  {
    ch = ITM_RxBuffer;
    ITM_RxBuffer = ITM_RXBUFFER_EMPTY;       /* ready for next character */
  }

  return (ch);
}


/**
  \brief   ITM Check Character
  \details Checks whether a character is pending for reading in the variable \ref ITM_RxBuffer.
  \return          0  No character available.
  \return          1  Character available.
 */
__STATIC_INLINE int32_t ITM_CheckChar (void)
{

  if (ITM_RxBuffer == ITM_RXBUFFER_EMPTY)
  {
    return (0);                              /* no character available */
  }
  else
  {
    return (1);                              /*    character available */
  }
}

/*@} end of CMSIS_core_DebugFunctions */




#ifdef __cplusplus
}
#endif

#endif /* __CORE_CM55_H_DEPENDANT */

#endif /* __CMSIS_GENERIC */
