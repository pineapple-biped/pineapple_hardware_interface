// No vendor constructor or real libusb: exercise actual replacement receive loop.
#include "usb_class.h"
#include <cassert>
static int step=0, decoded=0;
static std::vector<uint8_t> packet;
usb_class::usb_class(uint32_t,uint32_t,std::string) { stop_thread=false; }
usb_class::~usb_class() = default;
int usb_class::unpack_can_frame(uint8_t* p,size_t n) {
  assert(n==24 && std::equal(p,p+n,packet.data()+2)); ++decoded; return 0;
}
extern "C" int libusb_bulk_transfer(libusb_device_handle*, unsigned char endpoint,
    unsigned char* data,int length,int* count,unsigned timeout) {
  assert(endpoint==0x81 && length==RX_LENGTH && timeout==1);
  switch(step++) {
    case 0: *count=0; return LIBUSB_ERROR_TIMEOUT;
    case 1: *count=10; std::copy_n(packet.data(),10,data); return LIBUSB_ERROR_TIMEOUT;
    case 2: *count=17; std::copy_n(packet.data()+10,17,data); return 0;
    case 3: *count=27; std::copy(packet.begin(),packet.end(),data); return LIBUSB_ERROR_TIMEOUT;
    default: *count=0; return LIBUSB_ERROR_NO_DEVICE;
  }
}
int main() {
  packet.resize(27); packet[0]=0xa5; packet[2]=20; packet.back()=0x5a;
  auto crc=CRC16(packet.data()+2,22); packet[24]=crc&255; packet[25]=crc>>8;
  usb_class device(0,0,""); device.get_data_thread();
  assert(decoded==2 && step==5);
}
