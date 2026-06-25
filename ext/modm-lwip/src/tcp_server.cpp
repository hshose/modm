#include "modm_lwip/tcp_server.hpp"

#include "lwip/tcp.h"

namespace modm::lwip
{

TcpServer::TcpServer(TcpConnection *connections, std::size_t connectionCount) :
	connections_(connections), connectionCapacity_(connectionCount)
{
}

TcpServer::~TcpServer()
{
	close();
}

bool
TcpServer::listen(uint16_t port)
{
	if (listenPcb_ != nullptr) {
		return true;
	}

	struct tcp_pcb *pcb = tcp_new();
	if (pcb == nullptr) {
		errorCount_++;
		return false;
	}

	if (tcp_bind(pcb, IP_ANY_TYPE, port) != ERR_OK) {
		tcp_abort(pcb);
		errorCount_++;
		return false;
	}

	listenPcb_ = tcp_listen(pcb);
	if (listenPcb_ == nullptr) {
		tcp_abort(pcb);
		errorCount_++;
		return false;
	}

	tcp_arg(listenPcb_, this);
	tcp_accept(listenPcb_, acceptTrampoline);
	return true;
}

void
TcpServer::close()
{
	if (listenPcb_ != nullptr) {
		tcp_arg(listenPcb_, nullptr);
		tcp_accept(listenPcb_, nullptr);
		tcp_close(listenPcb_);
		listenPcb_ = nullptr;
	}

	for (std::size_t index = 0; index < connectionCapacity_; ++index) {
		connections_[index].close();
	}
}

void
TcpServer::onAccept(AcceptCallback callback, void *userData)
{
	acceptCallback_ = callback;
	acceptUserData_ = userData;
}

uint32_t
TcpServer::connectionCount() const
{
	return connectionCount_;
}

uint32_t
TcpServer::activeConnectionCount() const
{
	uint32_t count = 0;
	for (std::size_t index = 0; index < connectionCapacity_; ++index) {
		if (connections_[index].isActive()) {
			count++;
		}
	}
	return count;
}

uint32_t
TcpServer::errorCount() const
{
	return errorCount_;
}

err_t
TcpServer::acceptTrampoline(void *arg, struct tcp_pcb *newPcb, err_t err)
{
	auto *server = static_cast<TcpServer *>(arg);
	if (server == nullptr || err != ERR_OK || newPcb == nullptr) {
		if (server != nullptr) {
			server->errorCount_++;
		}
		return ERR_VAL;
	}

	TcpConnection *connection = server->allocateConnection();
	if (connection == nullptr) {
		tcp_abort(newPcb);
		server->errorCount_++;
		return ERR_ABRT;
	}

	connection->attach(newPcb);
	server->connectionCount_++;

	if (server->acceptCallback_ != nullptr) {
		server->acceptCallback_(*connection, server->acceptUserData_);
	}

	return ERR_OK;
}

TcpConnection *
TcpServer::allocateConnection()
{
	for (std::size_t index = 0; index < connectionCapacity_; ++index) {
		if (!connections_[index].isActive()) {
			return &connections_[index];
		}
	}
	return nullptr;
}

}
