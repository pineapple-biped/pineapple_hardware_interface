// Passive LD_PRELOAD observer. Never changes transfer arguments or return values.
#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <dlfcn.h>

struct libusb_device_handle;
using Transfer = int (*)(libusb_device_handle*, unsigned char, unsigned char*,
                        int, int*, unsigned int);
struct Counts {
  std::atomic<unsigned long long> calls{0}, bytes{0}, timeouts{0};
  std::atomic<unsigned long long> partial_timeouts{0}, partial_bytes{0};
  std::atomic<unsigned long long> errors{0}, short_success{0};
};
static Counts counters[256];

extern "C" int libusb_bulk_transfer(libusb_device_handle* device,
    unsigned char endpoint, unsigned char* data, int length,
    int* transferred, unsigned int timeout) {
  static Transfer real = reinterpret_cast<Transfer>(
      dlsym(RTLD_NEXT, "libusb_bulk_transfer"));
  if (!real) {
    std::fputs("USB audit: cannot resolve real libusb_bulk_transfer\n", stderr);
    std::abort();
  }
  const int rc = real(device, endpoint, data, length, transferred, timeout);
  auto& c = counters[endpoint];
  c.calls.fetch_add(1, std::memory_order_relaxed);
  // libusb defines transferred for success and timeout; don't read on other errors.
  const int n = transferred && (rc == 0 || rc == -7) ? *transferred : 0;
  if (n > 0) c.bytes.fetch_add(n, std::memory_order_relaxed);
  if (rc == -7) {
    c.timeouts.fetch_add(1, std::memory_order_relaxed);
    if (n > 0) {
      c.partial_timeouts.fetch_add(1, std::memory_order_relaxed);
      c.partial_bytes.fetch_add(n, std::memory_order_relaxed);
    }
  } else if (rc != 0) {
    c.errors.fetch_add(1, std::memory_order_relaxed);
  } else if (n < length) {
    c.short_success.fetch_add(1, std::memory_order_relaxed);
  }
  return rc;
}

__attribute__((destructor)) static void report() {
  std::fputs("USB_BULK_AUDIT {\"endpoints\":[", stderr);
  bool first = true;
  for (unsigned i = 0; i < 256; ++i) {
    auto& c = counters[i];
    if (!c.calls.load()) continue;
    std::fprintf(stderr,
        "%s{\"endpoint\":%u,\"calls\":%llu,\"bytes\":%llu,"
        "\"timeouts\":%llu,\"partial_timeouts\":%llu,"
        "\"partial_timeout_bytes\":%llu,\"other_errors\":%llu,"
        "\"short_success\":%llu}",
        first ? "" : ",", i, c.calls.load(), c.bytes.load(),
        c.timeouts.load(), c.partial_timeouts.load(), c.partial_bytes.load(),
        c.errors.load(), c.short_success.load());
    first = false;
  }
  std::fputs("]}\n", stderr);
}
