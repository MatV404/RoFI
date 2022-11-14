#include <array>
#include <cassert>
#include <vector>

#include <armadillo>

namespace rofi::isoreconfig {

std::vector<rofi::configuration::RofiWorld> bfsShapes(
    const rofi::configuration::RofiWorld& start, const rofi::configuration::RofiWorld& target,
    float step, unsigned int bound );

} // namespace rofi::isoreconfig
