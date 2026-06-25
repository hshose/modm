#pragma once

#include <cstdint>

namespace network
{

struct Config
{
	uint8_t ip[4];
	uint8_t netmask[4];
	uint8_t gateway[4];
};

struct DiagnosticsData
{
	uint32_t rxFrames;
	uint32_t txFrames;
	uint32_t rxAllocFailures;
	uint32_t txErrors;
	uint32_t droppedFrames;
	uint32_t arpFrames;
	uint32_t icmpFrames;
	bool linkUp;
};

inline constexpr Config StaticConfig {
	{ 192, 168, 1, 50 },
	{ 255, 255, 255, 0 },
	{ 192, 168, 1, 1 },
};

extern DiagnosticsData Diagnostics;

void initialize();
void poll();

}
