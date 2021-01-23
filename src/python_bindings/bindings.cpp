#include <pybind11/pybind11.h>
#include "ndarray_converter.h"
namespace py = pybind11;

void init_System(py::module &);
void init_ImuTypes(py::module &);


PYBIND11_MODULE(ORBSLAM3, m) {
    // IMPORTANT: Put the following line in any bindings file that needs CV::Mat
    NDArrayConverter::init_numpy();  // initialize ability to convert between numpy and cv::Mat
    init_System(m);
    init_ImuTypes(m);
}
