#!/usr/bin/env bash
# Offline stub only. Does not link real libusb or access any device.
set -euo pipefail
root=$(cd "$(dirname "$0")/.." && pwd)
tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT
c++ -std=c++17 -Wall -Wextra -Werror -shared -fPIC "$root/diagnostics/usb_bulk_audit.cpp" -ldl -o "$tmp/audit.so"
cat > "$tmp/fake.cpp" <<'CPP'
extern "C" int libusb_bulk_transfer(void*, unsigned char, unsigned char* data,
    int length, int* n, unsigned timeout) {
  *n = timeout == 1 ? 8 : 0;
  data[0] = 42;
  return length == 88 ? 0 : -7;
}
CPP
cat > "$tmp/main.cpp" <<'CPP'
#include <cassert>
extern "C" int libusb_bulk_transfer(void*, unsigned char, unsigned char*, int, int*, unsigned);
int main() {
  unsigned char data[88] = {}; int n = -1;
  assert(libusb_bulk_transfer(nullptr, 131, data, 88, &n, 1) == 0);
  assert(n == 8 && data[0] == 42);
  assert(libusb_bulk_transfer(nullptr, 131, data, 32, &n, 1) == -7);
  assert(n == 8);
  assert(libusb_bulk_transfer(nullptr, 131, data, 32, &n, 2) == -7);
  assert(n == 0);
}
CPP
c++ -shared -fPIC "$tmp/fake.cpp" -o "$tmp/libfake.so"
c++ "$tmp/main.cpp" -L"$tmp" -lfake -Wl,-rpath,"$tmp" -o "$tmp/test"
LD_PRELOAD="$tmp/audit.so" "$tmp/test" 2> "$tmp/result"
grep -F '"calls":3,"bytes":16,"timeouts":2,"partial_timeouts":1,"partial_timeout_bytes":8,"other_errors":0,"short_success":1' "$tmp/result"
echo 'PASS: offline observer forwards results and counts partial timeouts.'
