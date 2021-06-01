#include <Converter.h>
#include "ndarray_converter.h"

#include <pybind11/pybind11.h>
#include <pybind11/stl_bind.h>

namespace py = pybind11;
//PYBIND11_MAKE_OPAQUE(std::vector<ORB_SLAM3::IMU::Point>)

void init_Converter(py::module &m){

    //py::bind_vector<std::vector<ORB_SLAM3::IMU::Point>>(m, "VectorPoint", py::module_local(true));
    NDArrayConverter::init_numpy();  // initialize ability to convert between numpy and cv::Mat
    

    py::class_<ORB_SLAM3::Converter>(m, "Converter");

    py::module_ m2=m.def_submodule("Converter", "Module to convert between CV and g2o objects");

        //static std::vector<cv::Mat> toDescriptorVector(const cv::Mat &Descriptors);
        //m2.def("toDescriptorVector", 
        //    [](cv::Mat &Descriptors) {return ORB_SLAM3::Converter::toDescriptorVector(Descriptor);});,

        //static g2o::SE3Quat toSE3Quat(const cv::Mat &cvT);
        m2.def("toSE3Quat", 
            [](cv::Mat &cvt){return ORB_SLAM3::Converter::toSE3Quat(cvt);}); // this is a lambda function that calls the function we want to wrap instead of actually wrapping it. 
        // static g2o::SE3Quat toSE3Quat(const g2o::Sim3 &gSim3);
        m2.def("toSE3Quat", 
            [](g2o::Sim3 &gSim3){return ORB_SLAM3::Converter::toSE3Quat(gSim3);}); // this is a lambda function that calls the function we want to wrap instead of actually wrapping it. 
        //static cv::Mat toCvMat(const g2o::SE3Quat &SE3);
        m2.def("toCvMat",
             [](g2o::SE3Quat &SE3){return ORB_SLAM3::Converter::toCvMat(SE3);});
        //static cv::Mat toCvMat(const g2o::Sim3 &Sim3);
        m2.def("toCvMat", 
            [] (g2o::Sim3 &Sim3) {return ORB_SLAM3::Converter::toCvMat(Sim3);});    
        //static cv::Mat toCvMat(const Eigen::Matrix<double,4,4> &m);
        m2.def("toCvMat",
            [] (Eigen::Matrix<double,4,4> &m) {return ORB_SLAM3::Converter::toCvMat(m);});
        //static cv::Mat toCvMat(const Eigen::Matrix3d &m);
        m2.def("toCvMat",
            [] (Eigen::Matrix3d &m) {return ORB_SLAM3::Converter::toCvMat(m);});
        //static cv::Mat toCvMat(const Eigen::Matrix<double,3,1> &m);
        m2.def("toCvMat",
            [] (Eigen::Matrix<double,3,1> &m) {return ORB_SLAM3::Converter::toCvMat(m);});
        //static cv::Mat toCvMat(const Eigen::MatrixXd &m);
        m2.def("toCvMat",
            [] (Eigen::MatrixXd &m) {return ORB_SLAM3::Converter::toCvMat(m);});

        //static cv::Mat toCvSE3(const Eigen::Matrix<double,3,3> &R, const Eigen::Matrix<double,3,1> &t);
        m2.def("toCvSE3",
            [] (Eigen::Matrix<double,3,3> &R, Eigen::Matrix<double,3,1> &t) {return ORB_SLAM3::Converter::toCvSE3(R,t);});
        //static cv::Mat tocvSkewMatrix(const cv::Mat &v);
        m2.def("tocvSkewMatrix",
            [] (cv::Mat &v) {return ORB_SLAM3::Converter::tocvSkewMatrix(v);});

        //static Eigen::Matrix<double,3,1> toVector3d(const cv::Mat &cvVector);
        m2.def("toVector3d",
            [] (cv::Mat &cvVector) {return ORB_SLAM3::Converter::toVector3d(cvVector);});
        //static Eigen::Matrix<double,3,1> toVector3d(const cv::Point3f &cvPoint);
        m2.def("toVector3d",
            [] (cv::Point3f &cvPoint) {return ORB_SLAM3::Converter::toVector3d(cvPoint);});
        //static Eigen::Matrix<double,3,3> toMatrix3d(const cv::Mat &cvMat3);
        m2.def("toMatrix3d",
            [] (cv::Mat &cvMat3) {return ORB_SLAM3::Converter::toMatrix3d(cvMat3);});
        //static Eigen::Matrix<double,4,4> toMatrix4d(const cv::Mat &cvMat4);
         m2.def("toMatrix4d",
            [] (cv::Mat &cvMat4) {return ORB_SLAM3::Converter::toMatrix4d(cvMat4);});
        //static std::vector<float> toQuaternion(const cv::Mat &M);
        m2.def("toQuaternion",
            [] (cv::Mat &M) {return ORB_SLAM3::Converter::toQuaternion(M);});

        //static bool isRotationMatrix(const cv::Mat &R);
        m2.def("isRotationMatrix",
            [] (cv::Mat &R) {return ORB_SLAM3::Converter::isRotationMatrix(R);});
        //static std::vector<float> toEuler(const cv::Mat &R);
        m2.def("toEuler",
            [] (cv::Mat &R) {return ORB_SLAM3::Converter::toEuler(R);});
   
}    