// Dan Nguyen (z5206032) &&
// Mon-1500

#ifndef ENVIRONMENT_HPP_
#define ENVIRONMENT_HPP_

#include "interfaces.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "single_shape_display.hpp"

#include "cone.hpp"
#include "cube.hpp"
#include "cylinder.hpp"
#include "point_list.hpp"
#include "sphere.hpp"
#include "triangle_list.hpp"
#include "assembly.hpp"

#include <memory>
#include <string>
#include <tuple>
#include <vector>

namespace shapes
{
class Environment : public Assembly
{
public:
    explicit Environment(void);
    ~Environment(void);
};
} // namespace shapes
#endif // ENVIRONMENT_HPP_
