/*
 * Minimal lwIP compiler/architecture hooks for this bare-metal example.
 */

#pragma once

#include <stdint.h>

typedef int sys_prot_t;

#define LWIP_PROVIDE_ERRNO

#define PACK_STRUCT_BEGIN
#define PACK_STRUCT_STRUCT __attribute__((__packed__))
#define PACK_STRUCT_END
#define PACK_STRUCT_FIELD(x) x

#ifndef LWIP_PLATFORM_ASSERT
#define LWIP_PLATFORM_ASSERT(x) do { (void)(x); while (1) { } } while (0)
#endif

#define LWIP_RAND() ((uint32_t)0x12345678U)
