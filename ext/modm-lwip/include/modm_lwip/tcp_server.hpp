#pragma once

#include "tcp_connection.hpp"

#include <cstddef>
#include <cstdint>

struct tcp_pcb;

namespace modm::lwip
{

class TcpServer
{
public:
	using AcceptCallback = void (*)(TcpConnection &connection, void *userData);

	explicit TcpServer(TcpConnection *connections, std::size_t connectionCount);
	~TcpServer();

	TcpServer(const TcpServer &) = delete;
	TcpServer &operator=(const TcpServer &) = delete;

	bool listen(uint16_t port);
	void close();

	void onAccept(AcceptCallback callback, void *userData = nullptr);

	uint32_t connectionCount() const;
	uint32_t activeConnectionCount() const;
	uint32_t errorCount() const;

private:
	static err_t acceptTrampoline(void *arg, struct tcp_pcb *newPcb, err_t err);

	TcpConnection *allocateConnection();

	struct tcp_pcb *listenPcb_ = nullptr;
	TcpConnection *connections_ = nullptr;
	std::size_t connectionCapacity_ = 0;
	AcceptCallback acceptCallback_ = nullptr;
	void *acceptUserData_ = nullptr;

	uint32_t connectionCount_ = 0;
	uint32_t errorCount_ = 0;
};

}
