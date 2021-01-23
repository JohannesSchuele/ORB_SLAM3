#include <ImuTypes.h>

#include <pybind11/pybind11.h>

namespace py = pybind11;

void init_ImuTypes(py::module &m) {
//    Point(const float &acc_x, const float &acc_y, const float &acc_z,
//    const float &ang_vel_x, const float &ang_vel_y, const float &ang_vel_z,
//    const double &timestamp)
    py::class_<ORB_SLAM3::IMU::Point>(m, "Point")
    .def(py::init<const float &, const float &, const float &, const float &, const float &, const float &, const double &>(),
         py::arg("acc_x"), py::arg("acc_y"), py::arg("acc_z"),
         py::arg("ang_vel_x"), py::arg("ang_vel_y"), py::arg("ang_vel_z"),py::arg("timestamp"),
         R"mydelimiter(
                    //IMU measurement (accelerometer, gyro and timestamp)
                    )mydelimiter");


}
