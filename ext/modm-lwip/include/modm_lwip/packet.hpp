#pragma once

#include <cstddef>
#include <cstdint>

#include "lwip/err.h"
#include "lwip/pbuf.h"

namespace modm::lwip
{

class PacketView
{
public:
	explicit PacketView(const struct pbuf *pbuf) : pbuf_(pbuf)
	{
	}

	bool isValid() const
	{
		return pbuf_ != nullptr;
	}

	std::size_t size() const
	{
		return pbuf_ != nullptr ? pbuf_->tot_len : 0;
	}

	bool isContiguous() const
	{
		return pbuf_ != nullptr && pbuf_->len == pbuf_->tot_len;
	}

	const uint8_t *data() const
	{
		return isContiguous() ? static_cast<const uint8_t *>(pbuf_->payload) : nullptr;
	}

	bool copyTo(void *destination, std::size_t length, std::size_t offset = 0) const
	{
		if (pbuf_ == nullptr || destination == nullptr ||
				offset > size() || length > size() - offset ||
				length > 0xffffu || offset > 0xffffu) {
			return false;
		}

		return pbuf_copy_partial(pbuf_, destination, static_cast<u16_t>(length),
				static_cast<u16_t>(offset)) == length;
	}

	template<typename T>
	bool copyAs(T &value, std::size_t offset = 0) const
	{
		return copyTo(&value, sizeof(T), offset);
	}

	template<typename Callback>
	void forEachChunk(Callback &&callback) const
	{
		for (const struct pbuf *q = pbuf_; q != nullptr; q = q->next) {
			if (q->len != 0) {
				callback(static_cast<const uint8_t *>(q->payload), q->len);
			}
		}
	}

private:
	const struct pbuf *pbuf_ = nullptr;
};

class PacketBuffer
{
public:
	PacketBuffer() = default;

	explicit PacketBuffer(std::size_t size) :
		pbuf_(size <= 0xffffu ?
				pbuf_alloc(PBUF_TRANSPORT, static_cast<u16_t>(size), PBUF_RAM) :
				nullptr)
	{
	}

	~PacketBuffer()
	{
		if (pbuf_ != nullptr) {
			pbuf_free(pbuf_);
		}
	}

	PacketBuffer(PacketBuffer &&other) noexcept :
		pbuf_(other.pbuf_)
	{
		other.pbuf_ = nullptr;
	}

	PacketBuffer &operator=(PacketBuffer &&other) noexcept
	{
		if (this != &other) {
			if (pbuf_ != nullptr) {
				pbuf_free(pbuf_);
			}
			pbuf_ = other.pbuf_;
			other.pbuf_ = nullptr;
		}
		return *this;
	}

	PacketBuffer(const PacketBuffer &) = delete;
	PacketBuffer &operator=(const PacketBuffer &) = delete;

	static PacketBuffer allocateTransport(std::size_t size)
	{
		return PacketBuffer(size);
	}

	bool isValid() const
	{
		return pbuf_ != nullptr;
	}

	std::size_t size() const
	{
		return pbuf_ != nullptr ? pbuf_->tot_len : 0;
	}

	uint8_t *data()
	{
		return isContiguous() ? static_cast<uint8_t *>(pbuf_->payload) : nullptr;
	}

	const uint8_t *data() const
	{
		return isContiguous() ? static_cast<const uint8_t *>(pbuf_->payload) : nullptr;
	}

	bool write(const void *source, std::size_t length, std::size_t offset = 0)
	{
		if (pbuf_ == nullptr || source == nullptr ||
				offset > size() || length > size() - offset ||
				length > 0xffffu || offset > 0xffffu) {
			return false;
		}

		return pbuf_take_at(pbuf_, source, static_cast<u16_t>(length),
				static_cast<u16_t>(offset)) == ERR_OK;
	}

	struct pbuf *release()
	{
		struct pbuf *p = pbuf_;
		pbuf_ = nullptr;
		return p;
	}

	struct pbuf *get()
	{
		return pbuf_;
	}

	const struct pbuf *get() const
	{
		return pbuf_;
	}

private:
	bool isContiguous() const
	{
		return pbuf_ != nullptr && pbuf_->len == pbuf_->tot_len;
	}

	struct pbuf *pbuf_ = nullptr;
};

}
