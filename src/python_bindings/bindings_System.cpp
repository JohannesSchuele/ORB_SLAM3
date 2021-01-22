#include <System.h>
#include <pybind11/pybind11.h>

namespace py = pybind11;

PYBIND11_MODULE(ORBSLAM3, m){
    py::class_<ORB_SLAM3::System>(m, "System")
            .def(py::init<const std::string &, const std::string &, const ORB_SLAM3::System::eSensor, const bool>(),
                    py::arg("strVocFile"), py::arg("strSettingsFile"), py::arg("sensor"), py::arg("bUseViewer"))
            .def("ActivateLocalizationMode", &ORB_SLAM3::System::ActivateLocalizationMode);
}