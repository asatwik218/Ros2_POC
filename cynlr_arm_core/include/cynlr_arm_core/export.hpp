#pragma once

#if defined(CYNLR_ARM_CORE_STATIC)
  #define CYNLR_ARM_CORE_EXPORT
#elif defined(_WIN32)
  #ifdef CYNLR_ARM_CORE_BUILDING
    #define CYNLR_ARM_CORE_EXPORT __declspec(dllexport)
  #else
    #define CYNLR_ARM_CORE_EXPORT __declspec(dllimport)
  #endif
#elif defined(__GNUC__)
  #ifdef CYNLR_ARM_CORE_BUILDING
    #define CYNLR_ARM_CORE_EXPORT __attribute__((visibility("default")))
  #else
    #define CYNLR_ARM_CORE_EXPORT
  #endif
#else
  #define CYNLR_ARM_CORE_EXPORT
#endif
