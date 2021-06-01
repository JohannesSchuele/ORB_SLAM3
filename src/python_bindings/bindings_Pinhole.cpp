#include <Pinhole.h>
#include "ndarray_converter.h"
#include <GeometricCamera.h>

#include <pybind11/pybind11.h>
#include <pybind11/stl_bind.h>

namespace py = pybind11;

void init_Pinhole(py::module &m){
    NDArrayConverter::init_numpy();  // initialize ability to convert between numpy and cv::Mat

    //class Pinhole : public GeometricCamera {
    py::class_<ORB_SLAM3::Pinhole>(m, "Pinhole")   
        //void serialize(Archive& ar, const unsigned int version)
        //.def("serialize", &ORB_SLAM3::Pinhole::serialize,
        //    py::arg("ar"), py::arg("version"))

        .def(py::init<>())
        .def(py::init<const std::vector<float>>(),
            py::arg("_vParameters"))
        .def(py::init<ORB_SLAM3::Pinhole*>(),
            py::arg("pPinhole"));  
/*
        //cv::Point2f project(const cv::Point3f &p3D);
        .def("project", &ORB_SLAM3::Pinhole::project,
            py::arg("p3D"))
        //cv::Point2f project(const cv::Mat &m3D);
        .def("project", &ORB_SLAM3::Pinhole::project,
            py::arg("m3D"))
        //Eigen::Vector2d project(const Eigen::Vector3d & v3D);
        .def("project", &ORB_SLAM3::Pinhole::project,
            py::arg("v3D"))
        //cv::Mat projectMat(const cv::Point3f& p3D);
        .def("projectMat", &ORB_SLAM3::Pinhole::projectMat,
            py::arg("p3D"))
          
        //float uncertainty2(const Eigen::Matrix<double,2,1> &p2D)
        .def("uncertainty2", &ORB_SLAM3::Pinhole::uncertainty2,
            py::arg("p2D"))

        //cv::Point3f unproject(const cv::Point2f &p2D);
        .def("unproject", &ORB_SLAM3::Pinhole::unproject,
            py::arg("p2D"))
        //cv::Mat unprojectMat(const cv::Point2f &p2D);
        .def("unprojectMat", &ORB_SLAM3::Pinhole::unprojectMat,
            py::arg("p2D"))
        
        //cv::Mat projectJac(const cv::Point3f &p3D);
        .def("projectJac", &ORB_SLAM3::Pinhole::projectJac,
            py::arg("p3D"))
        //Eigen::Matrix<double,2,3> projectJac(const Eigen::Vector3d& v3D);
        .def("projectJac", &ORB_SLAM3::Pinhole::projectJac,
            py::arg("v3D"))

        //cv::Mat projectJac(const cv::Point3f &p3D);
        .def("unprojectJac", &ORB_SLAM3::Pinhole::unprojectJac,
            py::arg("p2D"))

        //bool ReconstructWithTwoViews(const std::vector<cv::KeyPoint>& vKeys1, const std::vector<cv::KeyPoint>& vKeys2, const std::vector<int> &vMatches12,
         //                                    cv::Mat &R21, cv::Mat &t21, std::vector<cv::Point3f> &vP3D, std::vector<bool> &vbTriangulated);
        .def("ReconstructWithTwoViews", &ORB_SLAM3::Pinhole::ReconstructWithTwoViews,
            py::arg("vKeys1"), py::arg("vKeys2"), py::arg("Matches12"), py::arg("R21"), py::arg("t21"), py::arg("vP3D"), py::arg("vbTriangulated"))
        
        //cv::Mat toK();
        .def("toK", &ORB_SLAM3::Pinhole::toK)
        
        //bool epipolarConstrain(GeometricCamera* pCamera2, const cv::KeyPoint& kp1, const cv::KeyPoint& kp2, const cv::Mat& R12, const cv::Mat& t12, const float sigmaLevel, const float unc);
        .def("epipolarConstrain", &ORB_SLAM3::Pinhole::epipolarConstrain,
            py::arg("pCamera2"), py::arg("kp1"), py::arg("kp2"), py::arg("R12"), py::arg("t12"), py::arg("sigmaLevel"), py::arg("unc"))
        
        //bool matchAndtriangulate(const cv::KeyPoint& kp1, const cv::KeyPoint& kp2, GeometricCamera* pOther,
        //                         cv::Mat& Tcw1, cv::Mat& Tcw2,
        //                         const float sigmaLevel1, const float sigmaLevel2,
        //                         cv::Mat& x3Dtriangulated) { return false;} 
        .def("matchAndtriangulate", &ORB_SLAM3::Pinhole::matchAndtriangulate,
            py::arg("kp1"), py::arg("kp2"), py::arg("pOther"), py::arg("Tcw1"), py::arg("Tcw2"), py::arg("sigmaLevel1"), py::arg("sigmaLevel2"), py::arg("x3Dtriangulated"));
 
*/
}






