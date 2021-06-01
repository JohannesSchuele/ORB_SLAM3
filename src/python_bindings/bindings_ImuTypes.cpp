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
         )mydelimiter")
 //     Point(const cv::Point3f Acc, const cv::Point3f Gyro, const double &timestamp)   
    .def(py::init<const cv::Point3f, const cv::Point3f, const double &>(),
        py::arg("Acc"), py::arg("Gyro"), py::arg("timestamp"),
        R"mydelimiter(
        //IMU measurement (accelerometer, gyro and timestamp)
        )mydelimiter")

//?
//   public:
//      cv::Point3f a;
//      cv::Point3f w;

    .def_readwrite("t", &ORB_SLAM3::IMU::Point::t);


    py::class_<ORB_SLAM3::IMU::Bias>(m, "Bias")


/*
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & bax;
        ar & bay;
        ar & baz;

        ar & bwx;
        ar & bwy;
        ar & bwz;
    }  
    
        .def("Serialize", &ORB_SLAM3::IMU::Bias::Serialize,
                py::arg("version"))

*/

//Bias(const float &b_acc_x, const float &b_acc_y, const float &b_acc_z,
//    const float &b_ang_vel_x, const float &b_ang_vel_y, const float &b_ang_vel_z):
//     bax(b_acc_x), bay(b_acc_y), baz(b_acc_z), bwx(b_ang_vel_x), bwy(b_ang_vel_y), bwz(b_ang_vel_z){}
        .def(py::init<const float &, const float &, const float &, const float &, const float &, const float &>(),
            py::arg("b_acc_x"), py::arg("b_acc_y"), py::arg("b_acc_z"),
            py::arg("b_ang_vel_x"), py::arg("b_ang_vel_y"), py::arg("b_ang_vel_z"))
         .def("CopyFrom", &ORB_SLAM3::IMU::Bias::CopyFrom,
                py::arg("b"))

        .def_readwrite("bax", &ORB_SLAM3::IMU::Bias::bax)
        .def_readwrite("bay", &ORB_SLAM3::IMU::Bias::bay)
        .def_readwrite("baz", &ORB_SLAM3::IMU::Bias::baz)
        .def_readwrite("bwx", &ORB_SLAM3::IMU::Bias::bwx)
        .def_readwrite("bwy", &ORB_SLAM3::IMU::Bias::bwy)
        .def_readwrite("bwz", &ORB_SLAM3::IMU::Bias::bwz);


 //IMU calibration (Tbc, Tcb, noise)
    py::class_<ORB_SLAM3::IMU::Calib>(m, "Calib")
//code vor public: Archive, friend
        .def(py::init<const cv::Mat &, const float &, const float &, const float &, const float &>(),
            py::arg("Tbc_"), py::arg("ng"), py::arg("na"), py::arg("ngw"), py::arg("naw"))            
        .def(py::init<const ORB_SLAM3::IMU::Calib &>(),
            py::arg("calib"))
        .def("Set", &ORB_SLAM3::IMU::Calib::Set,
                py::arg("Tbc_"), py::arg("ng"), py::arg("na"), py::arg("ngw"), py::arg("naw"))

        .def_readwrite("Tcb", &ORB_SLAM3::IMU::Calib::Tcb)
        .def_readwrite("Tbc", &ORB_SLAM3::IMU::Calib::Tbc)
        .def_readwrite("Cov", &ORB_SLAM3::IMU::Calib::Cov)
        .def_readwrite("CovWalk", &ORB_SLAM3::IMU::Calib::CovWalk);


//Integration of 1 gyro measurement
    py::class_<ORB_SLAM3::IMU::IntegratedRotation>(m, "IntegratedRotation")
        .def(py::init<const cv::Point3f &, const ORB_SLAM3::IMU::Bias &, const float &>(),
         py::arg("angVel"), py::arg("imuBias"), py::arg("time"))

        .def_readwrite("deltaT", &ORB_SLAM3::IMU::IntegratedRotation::deltaT)   //integration time
        .def_readwrite("deltaR", &ORB_SLAM3::IMU::IntegratedRotation::deltaR)   //integrated rotation
        .def_readwrite("rightJ", &ORB_SLAM3::IMU::IntegratedRotation::rightJ);   //right jacobian







// ! Compiler deletes the required function for class Preintegrated ! 
// the following code for class Preintegrated was commented out completely

/*
//Preintegration of Imu Measurements
py::class_<ORB_SLAM3::IMU::Preintegrated>(m, "Preintegrated")
        .def("serializeMatrix", &ORB_SLAM3::IMU::Preintegrated::serializeMatrix,
           py::arg("ar"), py::arg("mat"), py::arg("version"))
        .def("serialize", &ORB_SLAM3::IMU::Preintegrated::serialize,
           py::arg("ar"), py::arg("version"))

//Preintegrated(const Bias &b_, const Calib &calib);
    .def(py::init<const ORB_SLAM3::IMU::Bias &, const ORB_SLAM3::IMU::Calib &>(),
         py::arg("b_"), py::arg("calib"))
//Preintegrated(Preintegrated* pImuPre);   
    .def(py::init<ORB_SLAM3::IMU::Preintegrated>(),
         py::arg("pImuPre"))

    .def("Preintegrated", &ORB_SLAM3::IMU::Preintegrated::CopyFrom,
        py::arg("pImuPre"))

    .def("CopyFrom", &ORB_SLAM3::IMU::Preintegrated::CopyFrom,
        py::arg("pImuPre"))
    .def("Initialize", &ORB_SLAM3::IMU::Preintegrated::Initialize,
        py::arg("b_"))
    .def("IntegrateNewMeasurement", &ORB_SLAM3::IMU::Preintegrated::IntegrateNewMeasurement,
        py::arg("acceleration"), py::arg("angVel"), py::arg("dt"))
    .def("Reintegrate", &ORB_SLAM3::IMU::Preintegrated::Reintegrate)
    .def("MergePrevious", &ORB_SLAM3::IMU::Preintegrated::MergePrevious,
        py::arg("pPrev"))
    .def("SetNewBias", &ORB_SLAM3::IMU::Preintegrated::SetNewBias,
        py::arg("u_"))
      
//IMU::Bias GetDeltaBias(const Bias &b_);
    .def("GetDeltaBias", &ORB_SLAM3::IMU::Preintegrated::GetDeltaBias,
        py::arg("b_"))

  .def("GetDeltaRotation", &ORB_SLAM3::IMU::Preintegrated::GetDeltaRotation,
        py::arg("b_"))
    .def("GetDeltaVelocity", &ORB_SLAM3::IMU::Preintegrated::GetDeltaVelocity,
        py::arg("b_"))
    .def("GetDeltaPosition", &ORB_SLAM3::IMU::Preintegrated::GetDeltaPosition,
        py::arg("b_"))     
    .def("GetUpdatedDeltaRotation", &ORB_SLAM3::IMU::Preintegrated::GetUpdatedDeltaRotation)
    .def("GetUpdatedDeltaVelocity", &ORB_SLAM3::IMU::Preintegrated::GetUpdatedDeltaVelocity)
    .def("GetUpdatedDeltaPosition", &ORB_SLAM3::IMU::Preintegrated::GetUpdatedDeltaPosition)
    .def("GetOriginalDeltaRotation", &ORB_SLAM3::IMU::Preintegrated::GetOriginalDeltaRotation)
    .def("GetOriginalDeltaVelocity", &ORB_SLAM3::IMU::Preintegrated::GetOriginalDeltaVelocity)
    .def("GetOriginalDeltaPosition", &ORB_SLAM3::IMU::Preintegrated::GetOriginalDeltaPosition)    
    .def("GetInformationMatrix", &ORB_SLAM3::IMU::Preintegrated::GetInformationMatrix)

//cv::Mat GetDeltaBias(); error wegen IMU::Bias GetDeltaBias(const Bias &b_)
    .def("GetDeltaBias", &ORB_SLAM3::IMU::Preintegrated::GetDeltaBias)   
   
    .def("GetOriginalBias", &ORB_SLAM3::IMU::Preintegrated::GetOriginalBias)
    .def("GetUpdatedBias", &ORB_SLAM3::IMU::Preintegrated::GetUpdatedBias)

    .def_readwrite("dT", &ORB_SLAM3::IMU::Preintegrated::dT)
    .def_readwrite("C", &ORB_SLAM3::IMU::Preintegrated::C)
    .def_readwrite("Info", &ORB_SLAM3::IMU::Preintegrated::Info)
    .def_readwrite("Nga", &ORB_SLAM3::IMU::Preintegrated::Nga)
    .def_readwrite("NgaWalk", &ORB_SLAM3::IMU::Preintegrated::NgaWalk)

// Values for the original bias (when integration was computed)
    .def_readwrite("b", &ORB_SLAM3::IMU::Preintegrated::b)
    .def_readwrite("dR", &ORB_SLAM3::IMU::Preintegrated::dR)
    .def_readwrite("dV", &ORB_SLAM3::IMU::Preintegrated::dV)
    .def_readwrite("dP", &ORB_SLAM3::IMU::Preintegrated::dP)
    .def_readwrite("JRg", &ORB_SLAM3::IMU::Preintegrated::JRg)
    .def_readwrite("JVa", &ORB_SLAM3::IMU::Preintegrated::JVa)
    .def_readwrite("JPg", &ORB_SLAM3::IMU::Preintegrated::JPg)
    .def_readwrite("JPa", &ORB_SLAM3::IMU::Preintegrated::JPa)
    .def_readwrite("avgA", &ORB_SLAM3::IMU::Preintegrated::avgA)
    .def_readwrite("avgW", &ORB_SLAM3::IMU::Preintegrated::avgW);
*/

}


