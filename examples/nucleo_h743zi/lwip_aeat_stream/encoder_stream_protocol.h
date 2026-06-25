#pragma once

#include <stdint.h>

#define ENCODER_STREAM_MAGIC 0x45535452UL
#define ENCODER_STREAM_VERSION 1U
#define ENCODER_STREAM_CONTROL_PORT 5010U
#define ENCODER_STREAM_MAX_UDP_PAYLOAD 1472U
#define ENCODER_STREAM_MAX_SAMPLES_PER_PACKET 64U
#define ENCODER_STREAM_DEFAULT_SAMPLES_PER_PACKET 64U
#define ENCODER_STREAM_MAX_PACKETS_PER_POLL 4U

typedef struct __attribute__((packed))
{
	uint32_t sequence;
	uint32_t timestamp_us;
	uint32_t position_raw;
	uint32_t status;
} EncoderSample;

typedef struct __attribute__((packed))
{
	uint32_t magic;
	uint16_t version;
	uint16_t type;
	uint32_t sequence;
	uint32_t payload_len;
} EncoderStreamHeader;

enum
{
	ENCODER_STREAM_CMD_START = 1,
	ENCODER_STREAM_CMD_STOP = 2,
	ENCODER_STREAM_CMD_STATUS = 3,

	ENCODER_STREAM_DATA = 10,
	ENCODER_STREAM_STATUS = 11,
	ENCODER_STREAM_ERROR = 12,
};

typedef struct __attribute__((packed))
{
	EncoderStreamHeader header;
	uint32_t requested_sample_decimation;
	uint32_t max_samples_per_packet;
	uint32_t flags;
} EncoderStreamStartCommand;

typedef struct __attribute__((packed))
{
	EncoderStreamHeader header;
	uint32_t first_sample_sequence;
	uint32_t sample_count;
	uint32_t sample_drop_count;
	uint32_t ring_overrun_count;
	EncoderSample samples[];
} EncoderStreamDataPacket;

typedef struct __attribute__((packed))
{
	EncoderStreamHeader header;
	uint32_t streaming_enabled;
	uint32_t samples_produced;
	uint32_t samples_sent;
	uint32_t ring_overruns;
	uint32_t ring_fill_level;
	uint32_t ring_max_fill_level;
	uint32_t udp_packets_sent;
	uint32_t udp_send_errors;
	uint32_t last_encoder_position;
} EncoderStreamStatusPacket;

#ifdef __cplusplus
static_assert(sizeof(EncoderSample) == 16);
static_assert(sizeof(EncoderStreamHeader) == 16);
static_assert(sizeof(EncoderStreamStartCommand) == 28);
static_assert(sizeof(EncoderStreamStatusPacket) == 52);
#endif
