#include <G2oTypes.h>
#include "ndarray_converter.h"

#include <pybind11/pybind11.h>
#include <pybind11/stl_bind.h>

namespace py = pybind11;
PYBIND11_MAKE_OPAQUE(std::vector<ORB_SLAM3::IMU::Point>)

void init_G2oTypes(py::module &m){
    // need to explicity tell it how to deal with a vector of type ORBSLAM3::IMU::POINT
    py::bind_vector<std::vector<ORB_SLAM3::IMU::Point>>(m, "VectorPoint", py::module_local(true));
    NDArrayConverter::init_numpy();  // initialize ability to convert between numpy and cv::Mat

/*
        .def("ExpSO3", &ORB_SLAM3::ExpSO3,
            py::arg("x"), py::arg("y"), py::arg("z"))
        .def("ExpSO3", &ORB_SLAM3::ExpSO3,
            py::arg("w"))

        .def("LogSO3", &ORB_SLAM3::G2oTypes::LogSO3,
            py::arg("R"))    void SetParam(const std::vector<Eigen::Matrix3d> &_Rcw, const std::vector<Eigen::Vector3d> &_tcw, const std::vector<Eigen::Matrix3d> &_Rbc,
                  const std::vector<Eigen::Vector3d> &_tbc, const double &_bf);

        .def("InverseRightJacobianSO3", &ORB_SLAM3::G2oTypes::InverseRightJacobianSO3,
            py::arg("v"))    
        .def("RightJacobianSO3", &ORB_SLAM3::G2oTypes::RightJacobianSO3,
            py::arg("v")) 
        .def("RightJacobianSO3", &ORB_SLAM3::G2oTypes::RightJacobianSO3,
            py::arg("x"), py::arg("y"), py::arg("z")) 

        .def("Skew", &ORB_SLAM3::G2oTypes::Skew,
            py::arg("w")) 
        .def("InverseRightJacobianSO3", &ORB_SLAM3::G2oTypes::InverseRightJacobianSO3,
            py::arg("x"), py::arg("y"), py::arg("z"))   

        .def("NormalizeRotation", &ORB_SLAM3::G2oTypes::NormalizeRotation,
            py::arg("R"));
*/
    //class ImuCamPose
    py::class_<ORB_SLAM3::ImuCamPose>(m, "ImuCamPose")
        //ImuCamPose(){}
        .def(py::init<>())
        //ImuCamPose(KeyFrame* pKF);
        .def(py::init<ORB_SLAM3::KeyFrame*>(),
            py::arg("pKF"))
        //ImuCamPose(Frame* pF);
        .def(py::init<ORB_SLAM3::Frame*>(),
            py::arg("pF"))
        //ImuCamPose(Eigen::Matrix3d &_Rwc, Eigen::Vector3d &_twc, KeyFrame* pKF);
        .def(py::init<Eigen::Matrix3d &, Eigen::Vector3d &, ORB_SLAM3::KeyFrame*>(),
            py::arg("_Rwc"), py::arg("_twc"), py::arg("pKF"))
        
        .def("SetParam", &ORB_SLAM3::ImuCamPose::SetParam,
            py::arg("_Rcw"), py::arg("_tcw"), py::arg("_Rbc"), py::arg("_tbc"), py::arg("_bf"))

        .def("Update", &ORB_SLAM3::ImuCamPose::Update,
            py::arg("*pu"),
            R"mydelimiter(
                 update in the imu reference
                 )mydelimiter")
        .def("UpdateW", &ORB_SLAM3::ImuCamPose::UpdateW,
            py::arg("*pu"),
            R"mydelimiter(
                 update in the world reference
                 )mydelimiter")
        .def("Project", &ORB_SLAM3::ImuCamPose::Project,
            py::arg("Xw"), py::arg("cam_idx")=0,
            R"mydelimiter(
                 Mono
                 )mydelimiter")
        .def("ProjectStereo", &ORB_SLAM3::ImuCamPose::ProjectStereo,
            py::arg("Xw"), py::arg("cam_idx")=0,
            R"mydelimiter(
                 Stereo
                 )mydelimiter")
        .def("isDepthPositive", &ORB_SLAM3::ImuCamPose::isDepthPositive,
            py::arg("Xw"), py::arg("cam_idx")=0)

         // For IMU
        .def_readwrite("Rwb", &ORB_SLAM3::ImuCamPose::Rwb)
        .def_readwrite("twb", &ORB_SLAM3::ImuCamPose::twb)

        // For set of cameras
        .def_readwrite("Rcw", &ORB_SLAM3::ImuCamPose::Rcw)
        .def_readwrite("tcw", &ORB_SLAM3::ImuCamPose::tcw)
        .def_readwrite("Rcb", &ORB_SLAM3::ImuCamPose::Rcb)
        .def_readwrite("Rbc", &ORB_SLAM3::ImuCamPose::Rbc)
        .def_readwrite("tcb", &ORB_SLAM3::ImuCamPose::tcb)
        .def_readwrite("tbc", &ORB_SLAM3::ImuCamPose::tbc)
        .def_readwrite("bf", &ORB_SLAM3::ImuCamPose::bf)
        .def_readwrite("pCamera", &ORB_SLAM3::ImuCamPose::pCamera)

        // For posegraph 4DoF
        .def_readwrite("Rwb0", &ORB_SLAM3::ImuCamPose::Rwb0)
        .def_readwrite("DR", &ORB_SLAM3::ImuCamPose::DR)

        .def_readwrite("its", &ORB_SLAM3::ImuCamPose::its);

    //class InvDepthPoint
    py::class_<ORB_SLAM3::InvDepthPoint>(m, "InvDepthPoint")

        //InvDepthPoint(){}
        //.def(py::init<>())
        
        //InvDepthPoint(double _rho, double _u, double _v, KeyFrame* pHostKF);
        .def(py::init<double, double, double, ORB_SLAM3::KeyFrame*>(),
            py::arg("_rho"), py::arg("_u"), py::arg("_v"), py::arg("pHostKF"))

        .def("Update", &ORB_SLAM3::InvDepthPoint::Update,
            py::arg("*pu"))

        .def_readwrite("rho", &ORB_SLAM3::InvDepthPoint::rho)

        // they are not variables, observation in the host frame
        .def_readwrite("u", &ORB_SLAM3::InvDepthPoint::u)
        .def_readwrite("v", &ORB_SLAM3::InvDepthPoint::v)

        // from host frame
        .def_readwrite("fx", &ORB_SLAM3::InvDepthPoint::fx)
        .def_readwrite("fy", &ORB_SLAM3::InvDepthPoint::fy)
        .def_readwrite("cy", &ORB_SLAM3::InvDepthPoint::cy)
        .def_readwrite("bf", &ORB_SLAM3::InvDepthPoint::bf)

        .def_readwrite("its", &ORB_SLAM3::InvDepthPoint::its);

        // many virtual functions are following
        
        
}