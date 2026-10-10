#include "usb_rx_stream.h"
#include <cassert>
int main() {
  std::vector<uint8_t> frame(27, 0);
  frame[0]=0xa5; frame[2]=20; frame.back()=0x5a;
  auto crc=CRC16(frame.data()+2,22);
  frame[24]=crc & 255; frame[25]=crc >> 8;
  for (size_t split=0; split<=frame.size(); ++split) {
    UsbRxStream stream; int calls=0;
    auto emit=[&](uint8_t* p,size_t n) {
      assert(n==24); assert(std::equal(p,p+n,frame.data()+2)); ++calls;
    };
    stream.feed(frame.data(),split,emit);
    stream.feed(frame.data()+split,frame.size()-split,emit);
    assert(calls==1 && stream.pending()==0);
  }
  UsbRxStream stream; int calls=0;
  auto emit=[&](uint8_t*,size_t){++calls;};
  auto bad=frame; bad[10]^=1;
  stream.feed(bad.data(),bad.size(),emit);
  const uint8_t huge[]={0xa5,0,1,0};
  stream.feed(huge,4,emit);
  auto joined=frame; joined.insert(joined.end(),frame.begin(),frame.end());
  stream.feed(joined.data(),joined.size(),emit);
  assert(calls==2 && stream.pending()==0 && stream.rejected>0);
}
