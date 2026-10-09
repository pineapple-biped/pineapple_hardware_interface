#include "../pineapple_hardware/motor_send_order.h"
#include <array>
#include <cassert>

int main() {
    assert(MotorSendOrder(8, false) == std::vector<int>({0,1,2,3,4,5,6,7}));
    const auto order = MotorSendOrder(8, true);
    assert(order == std::vector<int>({1,2,3,4,5,6,7,0}));
    const std::array<int,8> ids{5,6,7,8,1,2,3,4};
    std::vector<int> sent_ids;
    std::array<int,8> visits{};
    for (int i : order) { sent_ids.push_back(ids[i]); ++visits[i]; }
    assert(sent_ids == std::vector<int>({6,7,8,1,2,3,4,5}));
    for (int v : visits) assert(v == 1);
    assert(MotorSendOrder(0, true).empty());
    assert(MotorSendOrder(1, true) == std::vector<int>({0}));
}
