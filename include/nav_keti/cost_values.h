#ifndef COST_VALUES_H
#define COST_VALUES_H

namespace cost {
    static constexpr unsigned char NO_INFORMATION = 255;
    static constexpr unsigned char LETHAL_OBSTACLE = 254;
    static constexpr unsigned char INSCRIBED_INFLATED_OBSTACLE = 253;
    static constexpr unsigned char MAX_NON_OBSTACLE = 252;
    static constexpr unsigned char FREE_SPACE = 0;
}

#endif // COST_VALUES_H
