#ifndef PINEAPPLE_MOTOR_SEND_ORDER_H
#define PINEAPPLE_MOTOR_SEND_ORDER_H
#include <algorithm>
#include <numeric>
#include <vector>

// Only submission traversal changes. The returned values remain original joint
// indices, used for commands, feedback, controller ownership and calibration.
inline std::vector<int> MotorSendOrder(int count, bool rotate_first_to_last) {
    std::vector<int> order(count);
    std::iota(order.begin(), order.end(), 0);
    if (rotate_first_to_last && count > 1)
        std::rotate(order.begin(), order.begin() + 1, order.end());
    return order;
}
#endif
