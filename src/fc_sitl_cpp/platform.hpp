// platform.hpp - the only OS-specific file. Everything above it in
// fc_core.hpp stays portable; everything below it is #ifdef'd here so
// fc_min.cpp reads the same on both platforms.
//
// Windows notes that matter:
//
//  * The CRT opens stdin/stdout in TEXT mode. Every 0x0A byte going out
//    becomes 0x0D 0x0A. Float payloads contain 0x0A constantly, so this
//    silently shreds packets while compiling perfectly. We sidestep it by
//    using the raw HANDLE with ReadFile/WriteFile, which the CRT never
//    touches, instead of _read/_write. A defensive _setmode is still
//    applied in case anything else reaches for the CRT streams.
//
//  * termios does not exist. Serial goes through DCB + SetCommState.
//
//  * COM10 and above need the \\.\ prefix or CreateFile fails with
//    ERROR_FILE_NOT_FOUND. We add it automatically.

#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <fcntl.h>
#include <io.h>
#include <windows.h>

#else
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#endif

namespace plat {

// Call once at the top of main. No-op on POSIX.
inline void set_binary_stdio() noexcept {
#if defined(_WIN32)
	_setmode(_fileno(stdin), _O_BINARY);
	_setmode(_fileno(stdout), _O_BINARY);
#endif
}

class ByteStream {
  public:
#if defined(_WIN32)

	// pipes: the std handles, untouched by CRT text translation
	ByteStream() noexcept : rh_{::GetStdHandle(STD_INPUT_HANDLE)}, wh_{::GetStdHandle(STD_OUTPUT_HANDLE)} {
	}

	explicit ByteStream(const std::string &dev, int baud = 115200) noexcept {
		// COM10+ is unreachable without the device-namespace prefix
		std::string path = dev;
		if (path.rfind("\\\\.\\", 0) != 0 && (path.rfind("COM", 0) == 0 || path.rfind("com", 0) == 0)) {
			path = "\\\\.\\" + path;
		}

		HANDLE h = ::CreateFileA(path.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr, OPEN_EXISTING, 0, nullptr);
		if (h == INVALID_HANDLE_VALUE)
			return;

		DCB dcb{};
		dcb.DCBlength = sizeof(dcb);
		if (!::GetCommState(h, &dcb)) {
			::CloseHandle(h);
			return;
		}

		// Windows takes the rate as a plain integer, unlike the POSIX Bxxxxx
		// constants, so no lookup table is needed on this side.
		dcb.BaudRate = static_cast<DWORD>(baud);
		dcb.ByteSize = 8;
		dcb.Parity = NOPARITY;
		dcb.StopBits = ONESTOPBIT;
		dcb.fBinary = TRUE;
		dcb.fParity = FALSE;
		dcb.fOutxCtsFlow = FALSE;
		dcb.fOutxDsrFlow = FALSE;
		dcb.fDtrControl = DTR_CONTROL_ENABLE;
		dcb.fRtsControl = RTS_CONTROL_ENABLE;
		dcb.fOutX = FALSE; // no XON/XOFF: it would eat 0x11/0x13
		dcb.fInX = FALSE;
		if (!::SetCommState(h, &dcb)) {
			::CloseHandle(h);
			return;
		}

		// all-zero timeouts: ReadFile blocks until at least one byte,
		// which is the POSIX VMIN=1 VTIME=0 behaviour
		COMMTIMEOUTS to{};
		::SetCommTimeouts(h, &to);

		rh_ = wh_ = h;
		owned_ = true;
	}

	~ByteStream() {
		if (owned_ && rh_ != INVALID_HANDLE_VALUE)
			::CloseHandle(rh_);
	}

	bool valid() const noexcept {
		return rh_ != INVALID_HANDLE_VALUE && rh_ != nullptr && wh_ != INVALID_HANDLE_VALUE && wh_ != nullptr;
	}

	bool read_exact(void *buf, std::size_t n) noexcept {
		auto *p = static_cast<std::uint8_t *>(buf);
		while (n) {
			DWORD got = 0;
			if (!::ReadFile(rh_, p, static_cast<DWORD>(n), &got, nullptr) || got == 0)
				return false;
			p += got;
			n -= got;
		}
		return true;
	}

	bool write_all(const void *buf, std::size_t n) noexcept {
		const auto *p = static_cast<const std::uint8_t *>(buf);
		while (n) {
			DWORD put = 0;
			if (!::WriteFile(wh_, p, static_cast<DWORD>(n), &put, nullptr) || put == 0)
				return false;
			p += put;
			n -= put;
		}
		return true;
	}

#else // ---------------- POSIX ----------------

	ByteStream() noexcept : rfd_{STDIN_FILENO}, wfd_{STDOUT_FILENO} {
	}

	// The Bxxxxx constants are opaque values, not the integers they are named
	// after, so a table is the only way to turn a --baud number into one.
	static speed_t baud_constant(int baud) noexcept {
		switch (baud) {
		case 9600:
			return B9600;
		case 19200:
			return B19200;
		case 38400:
			return B38400;
		case 57600:
			return B57600;
		case 115200:
			return B115200;
		case 230400:
			return B230400;
		case 460800:
			return B460800;
		case 921600:
			return B921600;
		default:
			return 0; // caller reports it; 0 would otherwise mean hang up
		}
	}

	explicit ByteStream(const std::string &dev, int baud = 115200) noexcept {
		const speed_t speed = baud_constant(baud);
		if (speed == 0)
			return;

		const int fd = ::open(dev.c_str(), O_RDWR | O_NOCTTY);
		if (fd < 0)
			return;

		termios t{};
		if (::tcgetattr(fd, &t) != 0) {
			::close(fd);
			return;
		}
		// Raw mode is not optional here. A tty defaults to canonical mode with
		// echo, CR/LF translation and XON/XOFF flow control, and these frames
		// carry float bytes that hit 0x11 and 0x13 constantly - software flow
		// control would silently pause the link on them. This is the POSIX
		// counterpart of set_binary_stdio() above.
		::cfmakeraw(&t);
		::cfsetispeed(&t, speed);
		::cfsetospeed(&t, speed);
		t.c_cc[VMIN] = 1;
		t.c_cc[VTIME] = 0;
		if (::tcsetattr(fd, TCSANOW, &t) != 0) {
			::close(fd);
			return;
		}

		rfd_ = wfd_ = fd;
		owned_ = true;
	}

	~ByteStream() {
		if (owned_ && rfd_ >= 0)
			::close(rfd_);
	}

	bool valid() const noexcept {
		return rfd_ >= 0 && wfd_ >= 0;
	}

	bool read_exact(void *buf, std::size_t n) noexcept {
		auto *p = static_cast<std::uint8_t *>(buf);
		while (n) {
			const ssize_t r = ::read(rfd_, p, n);
			if (r <= 0)
				return false;
			p += r;
			n -= static_cast<std::size_t>(r);
		}
		return true;
	}

	bool write_all(const void *buf, std::size_t n) noexcept {
		const auto *p = static_cast<const std::uint8_t *>(buf);
		while (n) {
			const ssize_t w = ::write(wfd_, p, n);
			if (w <= 0)
				return false;
			p += w;
			n -= static_cast<std::size_t>(w);
		}
		return true;
	}

#endif

	ByteStream(const ByteStream &) = delete;
	ByteStream &operator=(const ByteStream &) = delete;

  private:
#if defined(_WIN32)
	HANDLE rh_{INVALID_HANDLE_VALUE};
	HANDLE wh_{INVALID_HANDLE_VALUE};
#else
	int rfd_{-1};
	int wfd_{-1};
#endif
	bool owned_{false};
};

} // namespace plat
