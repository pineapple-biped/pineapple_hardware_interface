#ifndef PINEAPPLE_FEEDBACK_ONLY_H
#define PINEAPPLE_FEEDBACK_ONLY_H
#include <algorithm>
#include <cstdint>
#include <vector>

inline bool DiagnosticRegisterAllowed(uint8_t rid) {
    return rid == 1 || rid == 10 || rid == 13 || rid == 14 ||
           rid == 20 || rid == 21 || rid == 22 || rid == 23 || rid == 36;
}

// Match only the outstanding diagnostic read; unrelated status frames cannot
// complete it. The one-shot diagnostic discards all other frames.
inline bool DiagnosticReplyMatches(const uint8_t* data, unsigned dlc,
                                   uint16_t motor_id, int pending) {
    const auto id = uint16_t(data[0]) | (uint16_t(data[1]) << 8);
    return dlc == 8 && id == motor_id && data[2] == 0x33 &&
           pending == ((int(id) << 8) | data[3]);
}

// Disable/status by default; explicit opt-in permits allowlisted register reads.
inline bool FeedbackOnlyFrameAllowed(const std::vector<uint8_t>& data, uint32_t id, bool register_reads = false) {
    if (id == 0x7FF) {
        if (register_reads && data.size() == 8 && data[2] == 0x33 &&
            DiagnosticRegisterAllowed(data[3]) &&
            std::all_of(data.begin() + 4, data.end(), [](uint8_t v) { return v == 0; }))
            return true;
        return data.size() == 4 && data[2] == 0xCC && data[3] == 0;
    }
    return data.size() == 8 && data[7] == 0xFD &&
        std::all_of(data.begin(), data.begin() + 7, [](uint8_t v) { return v == 0xFF; });
}
#endif
