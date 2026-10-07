#ifndef PINEAPPLE_FEEDBACK_ONLY_H
#define PINEAPPLE_FEEDBACK_ONLY_H
#include <algorithm>
#include <cstdint>
#include <vector>

// Motor traffic allowed in diagnostics: disable and status refresh only.
inline bool FeedbackOnlyFrameAllowed(const std::vector<uint8_t>& data, uint32_t id) {
    if (id == 0x7FF)
        return data.size() == 4 && data[2] == 0xCC && data[3] == 0;
    return data.size() == 8 && data[7] == 0xFD &&
        std::all_of(data.begin(), data.begin() + 7, [](uint8_t v) { return v == 0xFF; });
}
#endif
