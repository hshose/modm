#include "modm_lwip/tcp_connection.hpp"

#include "lwip/tcp.h"

namespace modm::lwip
{

TcpConnection::~TcpConnection()
{
	close();
}

bool
TcpConnection::write(const void *data, std::size_t length)
{
	if (pcb_ == nullptr || data == nullptr || length == 0) {
		return false;
	}

	const auto *bytes = static_cast<const uint8_t *>(data);
	std::size_t offset = 0;
	while (offset < length) {
		const u16_t available = tcp_sndbuf(pcb_);
		if (available == 0) {
			sendLimitedCount_++;
			return false;
		}

		std::size_t chunk = length - offset;
		if (chunk > available) {
			chunk = available;
		}
		if (chunk > 0xffffu) {
			chunk = 0xffffu;
		}

		const err_t result = tcp_write(pcb_, bytes + offset,
				static_cast<u16_t>(chunk), TCP_WRITE_FLAG_COPY);
		if (result != ERR_OK) {
			sendLimitedCount_++;
			return false;
		}

		txBytes_ += static_cast<uint32_t>(chunk);
		offset += chunk;
	}

	return true;
}

bool
TcpConnection::write(PacketView data)
{
	bool ok = true;
	data.forEachChunk([this, &ok](const uint8_t *payload, std::size_t length) {
		if (ok) {
			ok = write(payload, length);
		}
	});
	return ok;
}

void
TcpConnection::output()
{
	if (pcb_ != nullptr) {
		(void) tcp_output(pcb_);
	}
}

void
TcpConnection::close()
{
	if (pcb_ == nullptr) {
		detach();
		return;
	}

	struct tcp_pcb *pcb = pcb_;
	clearCallbacks();
	detach();

	if (tcp_close(pcb) != ERR_OK) {
		tcp_abort(pcb);
	}
}

void
TcpConnection::abort()
{
	if (pcb_ == nullptr) {
		detach();
		return;
	}

	struct tcp_pcb *pcb = pcb_;
	clearCallbacks();
	detach();
	tcp_abort(pcb);
}

void
TcpConnection::onReceive(ReceiveCallback callback, void *userData)
{
	receiveCallback_ = callback;
	receiveUserData_ = userData;
}

void
TcpConnection::onClose(CloseCallback callback, void *userData)
{
	closeCallback_ = callback;
	closeUserData_ = userData;
}

void
TcpConnection::onError(ErrorCallback callback, void *userData)
{
	errorCallback_ = callback;
	errorUserData_ = userData;
}

bool
TcpConnection::isActive() const
{
	return active_;
}

uint32_t
TcpConnection::rxBytes() const
{
	return rxBytes_;
}

uint32_t
TcpConnection::txBytes() const
{
	return txBytes_;
}

uint32_t
TcpConnection::sendLimitedCount() const
{
	return sendLimitedCount_;
}

void
TcpConnection::attach(struct tcp_pcb *pcb)
{
	pcb_ = pcb;
	active_ = pcb != nullptr;
	closing_ = false;
	rxBytes_ = 0;
	txBytes_ = 0;
	sendLimitedCount_ = 0;

	tcp_arg(pcb_, this);
	tcp_recv(pcb_, receiveTrampoline);
	tcp_sent(pcb_, sentTrampoline);
	tcp_err(pcb_, errorTrampoline);
	tcp_poll(pcb_, pollTrampoline, 4);
}

void
TcpConnection::detach()
{
	pcb_ = nullptr;
	active_ = false;
	closing_ = false;
	receiveCallback_ = nullptr;
	receiveUserData_ = nullptr;
	closeCallback_ = nullptr;
	closeUserData_ = nullptr;
	errorCallback_ = nullptr;
	errorUserData_ = nullptr;
}

void
TcpConnection::clearCallbacks()
{
	if (pcb_ == nullptr) {
		return;
	}

	tcp_arg(pcb_, nullptr);
	tcp_recv(pcb_, nullptr);
	tcp_sent(pcb_, nullptr);
	tcp_err(pcb_, nullptr);
	tcp_poll(pcb_, nullptr, 0);
}

err_t
TcpConnection::receiveTrampoline(void *arg, struct tcp_pcb *pcb,
		struct pbuf *p, err_t err)
{
	auto *connection = static_cast<TcpConnection *>(arg);
	if (connection == nullptr) {
		if (p != nullptr) {
			pbuf_free(p);
		}
		return ERR_OK;
	}

	if (err != ERR_OK) {
		if (p != nullptr) {
			pbuf_free(p);
		}
		return err;
	}

	if (p == nullptr) {
		auto closeCallback = connection->closeCallback_;
		void *closeUserData = connection->closeUserData_;
		connection->clearCallbacks();
		connection->detach();
		if (closeCallback != nullptr) {
			closeCallback(*connection, closeUserData);
		}
		if (tcp_close(pcb) == ERR_OK) {
			return ERR_OK;
		}
		tcp_abort(pcb);
		return ERR_ABRT;
	}

	connection->rxBytes_ += p->tot_len;
	tcp_recved(pcb, p->tot_len);

	if (connection->receiveCallback_ != nullptr) {
		connection->receiveCallback_(*connection, PacketView(p),
				connection->receiveUserData_);
	}

	pbuf_free(p);
	return ERR_OK;
}

err_t
TcpConnection::sentTrampoline(void *, struct tcp_pcb *, u16_t)
{
	return ERR_OK;
}

void
TcpConnection::errorTrampoline(void *arg, err_t err)
{
	auto *connection = static_cast<TcpConnection *>(arg);
	if (connection == nullptr) {
		return;
	}

	auto errorCallback = connection->errorCallback_;
	void *errorUserData = connection->errorUserData_;
	connection->detach();
	if (errorCallback != nullptr) {
		errorCallback(*connection, err, errorUserData);
	}
}

err_t
TcpConnection::pollTrampoline(void *arg, struct tcp_pcb *pcb)
{
	auto *connection = static_cast<TcpConnection *>(arg);
	if (connection != nullptr && connection->pcb_ == pcb) {
		(void) tcp_output(pcb);
	}
	return ERR_OK;
}

}
