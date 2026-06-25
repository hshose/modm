#pragma once

#include "packet.hpp"

#include "lwip/err.h"

#include <cstddef>
#include <cstdint>

struct tcp_pcb;

namespace modm::lwip
{

class TcpServer;

class TcpConnection
{
public:
	using ReceiveCallback = void (*)(TcpConnection &connection,
			PacketView data,
			void *userData);
	using CloseCallback = void (*)(TcpConnection &connection, void *userData);
	using ErrorCallback = void (*)(TcpConnection &connection, err_t error, void *userData);

	TcpConnection() = default;
	~TcpConnection();

	TcpConnection(const TcpConnection &) = delete;
	TcpConnection &operator=(const TcpConnection &) = delete;

	bool write(const void *data, std::size_t length);
	bool write(PacketView data);
	void output();

	void close();
	void abort();

	void onReceive(ReceiveCallback callback, void *userData = nullptr);
	void onClose(CloseCallback callback, void *userData = nullptr);
	void onError(ErrorCallback callback, void *userData = nullptr);

	bool isActive() const;
	uint32_t rxBytes() const;
	uint32_t txBytes() const;
	uint32_t sendLimitedCount() const;

private:
	friend class TcpServer;

	void attach(struct tcp_pcb *pcb);
	void detach();
	void clearCallbacks();

	static err_t receiveTrampoline(void *arg, struct tcp_pcb *pcb,
			struct pbuf *p, err_t err);
	static err_t sentTrampoline(void *arg, struct tcp_pcb *pcb, u16_t len);
	static void errorTrampoline(void *arg, err_t err);
	static err_t pollTrampoline(void *arg, struct tcp_pcb *pcb);

	struct tcp_pcb *pcb_ = nullptr;
	bool active_ = false;
	bool closing_ = false;

	ReceiveCallback receiveCallback_ = nullptr;
	void *receiveUserData_ = nullptr;
	CloseCallback closeCallback_ = nullptr;
	void *closeUserData_ = nullptr;
	ErrorCallback errorCallback_ = nullptr;
	void *errorUserData_ = nullptr;

	uint32_t rxBytes_ = 0;
	uint32_t txBytes_ = 0;
	uint32_t sendLimitedCount_ = 0;
};

}
