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
    std::cout << "feedback-only transmit allowlist tests passed\n";
}
