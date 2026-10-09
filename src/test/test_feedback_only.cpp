#include "../canfd/include/feedback_only.h"
#include <cassert>
#include <iostream>

int main() {
    std::vector<uint8_t> disable(8, 0xFF);
    disable[7] = 0xFD;
    assert(FeedbackOnlyFrameAllowed(disable, 7));
    for (int command = 0; command < 256; ++command) {
        auto frame = disable;
        frame[7] = command;
        assert(FeedbackOnlyFrameAllowed(frame, 7) == (command == 0xFD));
    }
    for (int index = 0; index < 7; ++index) {
        auto frame = disable;
        frame[index] = 0;
        assert(!FeedbackOnlyFrameAllowed(frame, 7));
    }
    assert(FeedbackOnlyFrameAllowed({7, 0, 0xCC, 0}, 0x7FF));
    assert(!FeedbackOnlyFrameAllowed({7, 0, 0xCC, 0}, 7));
    for (int op : {0x33, 0x55, 0xAA})
        assert(!FeedbackOnlyFrameAllowed({7, 0, static_cast<uint8_t>(op), 1, 0, 0, 0, 0}, 0x7FF));
    assert(!FeedbackOnlyFrameAllowed({}, 7));
    assert(!FeedbackOnlyFrameAllowed(std::vector<uint8_t>(8, 0), 7));
    for (int rid = 0; rid < 256; ++rid) {
        const std::vector<uint8_t> read{7, 0, 0x33, static_cast<uint8_t>(rid), 0, 0, 0, 0};
        assert(FeedbackOnlyFrameAllowed(read, 0x7FF, true) == DiagnosticRegisterAllowed(rid));
        for (int op : {0x55, 0xAA}) {
            auto write = read; write[2] = op;
            assert(!FeedbackOnlyFrameAllowed(write, 0x7FF, true));
        }
    }
    assert(!FeedbackOnlyFrameAllowed({7,0,0x33,21,1,0,0,0}, 0x7FF, true));
    assert(!FeedbackOnlyFrameAllowed({255,255,255,255,255,255,255,0xFC}, 7, true));
    assert(!FeedbackOnlyFrameAllowed({255,255,255,255,255,255,255,0xFE}, 7, true));
    uint8_t reply[8]{7,0,0x33,21,0,0,0,0};
    assert(DiagnosticReplyMatches(reply, 8, 7, (7<<8)|21));
    assert(!DiagnosticReplyMatches(reply, 8, 6, (7<<8)|21));
    assert(!DiagnosticReplyMatches(reply, 8, 7, (7<<8)|22));
    assert(!DiagnosticReplyMatches(reply, 7, 7, (7<<8)|21));
    assert(!DiagnosticReplyMatches(reply, 8, 7, -1));
    std::cout << "feedback-only transmit allowlist tests passed\n";
}
