#include "modm_lwip/udp_socket.hpp"

#include "lwip/err.h"
#include "lwip/udp.h"

namespace modm::lwip
{

UdpSocket::~UdpSocket()
{
	close();
}

bool
UdpSocket::bind(uint16_t port)
{
	if (pcb_ != nullptr) {
		return true;
	}

	pcb_ = udp_new();
	if (pcb_ == nullptr) {
		errorCount_++;
		return false;
	}

	if (udp_bind(pcb_, IP_ANY_TYPE, port) != ERR_OK) {
		udp_remove(pcb_);
		pcb_ = nullptr;
		errorCount_++;
		return false;
	}

	udp_recv(pcb_, receiveTrampoline, this);
	return true;
}

void
UdpSocket::close()
{
	if (pcb_ == nullptr) {
		return;
	}

	udp_recv(pcb_, nullptr, nullptr);
	udp_remove(pcb_);
	pcb_ = nullptr;
}

bool
UdpSocket::isBound() const
{
	return pcb_ != nullptr;
}

void
UdpSocket::onReceive(ReceiveCallback callback, void *userData)
{
	callback_ = callback;
	userData_ = userData;
}

bool
UdpSocket::sendTo(const UdpEndpoint &endpoint, const void *data, std::size_t length)
{
	PacketBuffer packet = PacketBuffer::allocateTransport(length);
	if (!packet.isValid() || !packet.write(data, length)) {
		errorCount_++;
		return false;
	}

	return sendTo(endpoint, static_cast<PacketBuffer &&>(packet));
}

bool
UdpSocket::sendTo(const UdpEndpoint &endpoint, PacketBuffer &&packet)
{
	if (pcb_ == nullptr || !packet.isValid()) {
		errorCount_++;
		return false;
	}

	struct pbuf *p = packet.get();
	const err_t result = udp_sendto(pcb_, p, &endpoint.address(), endpoint.port());
	if (result == ERR_OK) {
		txCount_++;
		return true;
	}

	errorCount_++;
	return false;
}

uint32_t
UdpSocket::rxCount() const
{
	return rxCount_;
}

uint32_t
UdpSocket::txCount() const
{
	return txCount_;
}

uint32_t
UdpSocket::errorCount() const
{
	return errorCount_;
}

void
UdpSocket::receiveTrampoline(void *arg, struct udp_pcb *, struct pbuf *p,
		const ip_addr_t *addr, u16_t port)
{
	auto *socket = static_cast<UdpSocket *>(arg);
	if (socket == nullptr) {
		if (p != nullptr) {
			pbuf_free(p);
		}
		return;
	}

	if (p == nullptr || addr == nullptr) {
		socket->errorCount_++;
		return;
	}

	socket->rxCount_++;
	if (socket->callback_ != nullptr) {
		const UdpEndpoint remote(*addr, port);
		socket->callback_(*socket, remote, PacketView(p), socket->userData_);
	}

	pbuf_free(p);
}

}
