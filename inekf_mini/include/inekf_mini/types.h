#ifndef TYPES_H
#define TYPES_H

#include <Eigen/Dense>
#include <vector>
#include <memory>
#include <iostream>

namespace inekf {
    enum class FrameType { World, Body };
    enum class ErrorType { LeftInvariant, RightInvariant };
} // namespace inekf

#endif // TYPES_H