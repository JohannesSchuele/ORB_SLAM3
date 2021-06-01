#include <pybind11/pybind11.h>
#include "ndarray_converter.h"
namespace py = pybind11;

void init_System(py::module &);
void init_ImuTypes(py::module &);
void init_Atlas(py::module &);
void init_Converter(py::module &);
void init_Frame(py::module &);
void int_FrameDrawer(py::module &);
void int_Pinhole(py::module &);
void int_G2oTypes(py::module &);
void int_Initializer(py::module &);



PYBIND11_MODULE(ORBSLAM3, m) {
    // IMPORTANT: Put the following line in any bindings file that needs CV::Mat
    NDArrayConverter::init_numpy();  // initialize ability to convert between numpy and cv::Mat
    init_System(m);
    init_ImuTypes(m);
    init_Atlas(m);
    init_Converter(m);
    init_Frame(m);
    int_FrameDrawer(m);
    int_Pinhole(m);
    int_G2oTypes(m);
    int_Initializer(m);
}
