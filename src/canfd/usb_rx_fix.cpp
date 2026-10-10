#include "usb_class.h"
#include "usb_rx_stream.h"
#include <array>
#include <cstdio>

// Strong definitions override the build-local weak vendor entry points.
void usb_class::get_data_thread() { receive_fixed(0x81); }
void usb_class::can_rev_thread() { receive_fixed(0x83); }

void usb_class::receive_fixed(unsigned char endpoint) {
  UsbRxStream stream;
  std::array<uint8_t, RX_LENGTH> buffer{};
  uint64_t partial = 0, recovered_bytes = 0;
  while (!stop_thread.load()) {
    int transferred = 0;
    int rc = libusb_bulk_transfer(dev_handle, endpoint, buffer.data(),
                                 buffer.size(), &transferred, 1);
    if (rc == 0 || rc == LIBUSB_ERROR_TIMEOUT) {
      if (transferred > 0 && transferred <= int(buffer.size())) {
        if (rc == LIBUSB_ERROR_TIMEOUT) { ++partial; recovered_bytes += transferred; }
        stream.feed(buffer.data(), transferred, [this](uint8_t* p, size_t n) {
          std::lock_guard<std::mutex> lock(mutex_);
          unpack_can_frame(p, n);
        });
      }
    } else {
      // Exit on genuine transport errors; watchdog handles missing fresh feedback.
      std::fprintf(stderr, "[usb-rx] endpoint=%u error=%d\n", endpoint, rc);
      break;
    }
  }
  std::fprintf(stderr,
      "USB_RX_FIXED endpoint=%u partial_timeouts=%llu recovered_bytes=%llu "
      "frames=%llu rejected_bytes=%llu pending_bytes=%zu\n", endpoint,
      (unsigned long long)partial, (unsigned long long)recovered_bytes,
      (unsigned long long)stream.frames, (unsigned long long)stream.rejected,
      stream.pending());
}
