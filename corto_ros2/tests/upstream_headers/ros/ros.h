#pragma once
#include <string>
// Only parameter reads are used by the unmodified numerical source. The oracle
// links the same host Eigen/PCL/NLopt/MOSEK as the ROS 2 executable.
namespace ros {
class NodeHandle {
public:
    template<class T> void param(const std::string& name, T& value, const T& fallback) const {
        value = fallback;
        if (name == "n_sweep") value = static_cast<T>(7);
        if (name == "MaxPointCloud") value = static_cast<T>(10000);
        if (name == "nmpc/maxVel" || name == "nmpc/real_maxVel") value = static_cast<T>(10);
    }
};
}
