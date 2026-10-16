// Echoes every byte that arrives on the link UART. The first thing the
// simulator needs from firmware: a device path that answers.

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(uart_echo, LOG_LEVEL_INF);

namespace {
const device *const kLink = DEVICE_DT_GET(DT_CHOSEN(fc_link_uart));
} // namespace

int main() {
	if (!device_is_ready(kLink)) {
		LOG_ERR("link uart %s not ready", kLink->name);
		return 0;
	}
	LOG_INF("echoing on %s", kLink->name);

	for (;;) {
		unsigned char c;
		if (uart_poll_in(kLink, &c) == 0) {
			uart_poll_out(kLink, c);
			continue;
		}
		// Nothing pending. Polling with a short sleep keeps native_sim off
		// 100% CPU while adding at most this much latency per byte.
		k_usleep(50);
	}
}
