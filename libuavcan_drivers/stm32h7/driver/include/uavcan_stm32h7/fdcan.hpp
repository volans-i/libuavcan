/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 * Bit definitions were copied from NuttX STM32 CAN driver.
 */

#pragma once

#include <uavcan_stm32h7/build_config.hpp>

#include <uavcan/uavcan.hpp>
#include <stdint.h>

#ifndef UAVCAN_CPP_VERSION
# error UAVCAN_CPP_VERSION
#endif

#if UAVCAN_CPP_VERSION < UAVCAN_CPP11
// #undef'ed at the end of this file
# define constexpr const
#endif

namespace uavcan_stm32h7
{
namespace fdcan
{
#include "stm32h743xx.h"

typedef FDCAN_GlobalTypeDef CanType;

constexpr unsigned long IDE = (0x40000000U); // Identifier Extension
constexpr unsigned long STID_MASK = (0x1FFC0000U); // Standard Identifier Mask
constexpr unsigned long EXID_MASK = (0x1FFFFFFFU); // Extended Identifier Mask
constexpr unsigned long RTR       = (0x20000000U); // Remote Transmission Request
constexpr unsigned long DLC_MASK  = (0x000F0000U); // Data Length Code

/**
 * CANx register sets
 */
CanType* const Can[UAVCAN_STM32H7_NUM_IFACES] = {
    reinterpret_cast<CanType*>(FDCAN1_BASE)
#if UAVCAN_STM32H7_NUM_IFACES > 1
    ,
    reinterpret_cast<CanType*>(FDCAN2_BASE)
#endif
};
}
}

#if UAVCAN_CPP_VERSION < UAVCAN_CPP11
# undef constexpr
#endif
