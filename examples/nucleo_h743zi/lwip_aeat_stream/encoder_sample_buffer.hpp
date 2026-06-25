#pragma once

#include "encoder_stream_protocol.h"

#include <atomic>
#include <cstddef>
#include <cstdint>

class EncoderSampleBuffer
{
public:
	static constexpr std::size_t Capacity = 1024;
	static constexpr std::size_t Mask = Capacity - 1;

	static_assert((Capacity & Mask) == 0, "Ring buffer capacity must be a power of two");

	bool
	pushFromInterrupt(const EncoderSample& sample)
	{
		const uint32_t head = head_.load(std::memory_order_relaxed);
		const uint32_t tail = tail_.load(std::memory_order_acquire);
		if ((head - tail) >= Capacity) {
			overrunCount_.fetch_add(1, std::memory_order_relaxed);
			return false;
		}

		samples_[head & Mask] = sample;
		head_.store(head + 1, std::memory_order_release);
		producedCount_.fetch_add(1, std::memory_order_relaxed);

		const uint32_t fill = head + 1 - tail;
		uint32_t maxFill = maxFillLevel_.load(std::memory_order_relaxed);
		while (fill > maxFill &&
				!maxFillLevel_.compare_exchange_weak(maxFill, fill,
					std::memory_order_relaxed, std::memory_order_relaxed)) {
		}
		return true;
	}

	bool
	pop(EncoderSample& sample)
	{
		const uint32_t tail = tail_.load(std::memory_order_relaxed);
		const uint32_t head = head_.load(std::memory_order_acquire);
		if (tail == head) {
			return false;
		}

		sample = samples_[tail & Mask];
		tail_.store(tail + 1, std::memory_order_release);
		poppedCount_.fetch_add(1, std::memory_order_relaxed);
		return true;
	}

	uint32_t producedCount() const { return producedCount_.load(std::memory_order_relaxed); }
	uint32_t poppedCount() const { return poppedCount_.load(std::memory_order_relaxed); }
	uint32_t overrunCount() const { return overrunCount_.load(std::memory_order_relaxed); }
	uint32_t maxFillLevel() const { return maxFillLevel_.load(std::memory_order_relaxed); }

	uint32_t
	fillLevel() const
	{
		const uint32_t head = head_.load(std::memory_order_acquire);
		const uint32_t tail = tail_.load(std::memory_order_acquire);
		return head - tail;
	}

private:
	EncoderSample samples_[Capacity] {};
	std::atomic<uint32_t> head_ {0};
	std::atomic<uint32_t> tail_ {0};
	std::atomic<uint32_t> producedCount_ {0};
	std::atomic<uint32_t> poppedCount_ {0};
	std::atomic<uint32_t> overrunCount_ {0};
	std::atomic<uint32_t> maxFillLevel_ {0};
};

extern EncoderSampleBuffer encoderSampleBuffer;
