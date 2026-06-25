#include "tcp_echo.h"
#include "udp_echo.h"

extern "C" void
modm_lwip_app_initialize(void)
{
	udp_echo_init();
	tcp_echo_init();
}

extern "C" void
modm_lwip_app_poll(void)
{
}
