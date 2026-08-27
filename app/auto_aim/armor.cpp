#include "armor.hpp"

namespace app::auto_aim {

ArmorName armor_name_from_class_id(int class_id) {
    switch (class_id) {
        case 0: return ArmorName::one;
        case 1: return ArmorName::two;
        case 2: return ArmorName::three;
        case 3: return ArmorName::four;
        case 4: return ArmorName::five;
        case 5: return ArmorName::sentry;
        case 6: return ArmorName::outpost;
        case 7: return ArmorName::base;
        default: return ArmorName::unknown;
    }
}

ArmorType armor_type_from_name(ArmorName name) {
    if (name == ArmorName::base || name == ArmorName::one) return ArmorType::big;
    return name == ArmorName::unknown ? ArmorType::unknown : ArmorType::small;
}

ArmorType armor_type_from_ratio(double width_height_ratio) {
    return width_height_ratio > 3.0 ? ArmorType::big : ArmorType::small;
}

ArmorPriority armor_priority(ArmorName name) {
    switch (name) {
        case ArmorName::sentry: return ArmorPriority::first;
        case ArmorName::outpost: return ArmorPriority::second;
        case ArmorName::base: return ArmorPriority::third;
        case ArmorName::one:
        case ArmorName::two:
        case ArmorName::three:
        case ArmorName::four:
        case ArmorName::five: return ArmorPriority::fourth;
        default: return ArmorPriority::fifth;
    }
}

int armor_count(ArmorName name) {
    return name == ArmorName::outpost || name == ArmorName::base ? 3 : 4;
}

bool is_big_armor(ArmorName name) {
    return armor_type_from_name(name) == ArmorType::big;
}

}  // namespace app::auto_aim
