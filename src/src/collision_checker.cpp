#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <vector>
#include <utility>
#include <cmath>

namespace py = pybind11;

int check_collision_type(py::array_t<int> map_info, int free, int occupied, int unknown,
                         std::pair<int, int> start, std::pair<int, int> end) {
    auto buf = map_info.request();
    int* ptr = static_cast<int*>(buf.ptr);
    int rows = buf.shape[0];
    int cols = buf.shape[1];

    int x0 = start.first, y0 = start.second;
    int x1 = end.first, y1 = end.second;

    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;

    while (true) {
        if (x0 >= 0 && x0 < cols && y0 >= 0 && y0 < rows) {
            int value = ptr[y0 * cols + x0];
            if (value == occupied) {
                return occupied;
            }
            if (value == unknown) {
                return unknown;
            }
        }

        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x0 += sx;
        }
        if (e2 < dx) {
            err += dx;
            y0 += sy;
        }
    }

    return free;
}

PYBIND11_MODULE(collision_checker, m) {
    m.def("check_collision_type", &check_collision_type, "Check for collision in the map",
          py::arg("map_info"), py::arg("free"), py::arg("occupied"), py::arg("unknown"),
          py::arg("start"), py::arg("end"));
}