#pragma once

// Feature toggle macros for MecaBridge
// These can be set at compile time to enable/disable specific features

#ifndef MECABRIDGE_ENABLE_SERVOS
#define MECABRIDGE_ENABLE_SERVOS 1
#endif

#ifndef MECABRIDGE_ENABLE_ESCS
#define MECABRIDGE_ENABLE_ESCS 1
#endif

#ifndef MECABRIDGE_ENABLE_DIAGNOSTICS
#define MECABRIDGE_ENABLE_DIAGNOSTICS 1
#endif

// Derived configuration
#if MECABRIDGE_ENABLE_SERVOS
constexpr bool kEnableServos = true;
constexpr int kServoInterfaces = 2;  // positional + continuous
#else
constexpr bool kEnableServos = false;
constexpr int kServoInterfaces = 0;
#endif

#if MECABRIDGE_ENABLE_ESCS
constexpr bool kEnableESCs = true;
constexpr int kESCInterfaces = 2;    // 2 ESC channels
#else
constexpr bool kEnableESCs = false;
constexpr int kESCInterfaces = 0;
#endif

#if MECABRIDGE_ENABLE_DIAGNOSTICS
constexpr bool kEnableDiagnostics = true;
#else
constexpr bool kEnableDiagnostics = false;
#endif

// Total interface counts
constexpr int kWheelInterfaces = 4;  // Always enabled
constexpr int kTotalInterfaces = kWheelInterfaces + kServoInterfaces + kESCInterfaces;

// Interface index mappings
constexpr int kWheelStartIndex = 0;
constexpr int kServoStartIndex = kWheelStartIndex + kWheelInterfaces;
constexpr int kESCStartIndex = kServoStartIndex + kServoInterfaces;