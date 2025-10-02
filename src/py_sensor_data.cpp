//  Copyright (c) 2025 Feng Yang
//
//  I am making my contributions/submissions to this project solely in my
//  personal capacity and am not conveying any rights to any intellectual
//  property of any third parties.

#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/map.h>
#include <rtabmap/core/SensorData.h>
#include "cv_typecaster.h"

namespace nb = nanobind;
using namespace nb::literals;
using namespace rtabmap;

void bindSensorData(nb::module_ &m) {
    nb::class_<SensorData>(m, "SensorData")
            .def(nb::init<>())
            .def(nb::init<const cv::Mat &, int, double>(), "image"_a, "id"_a, "stamp"_a)
            .def(nb::init<const cv::Mat &, const CameraModel &, int, double>(), "image"_a, "cameraModel"_a, "id"_a,
                 "stamp"_a)
            .def(nb::init<const cv::Mat &, const cv::Mat &, const CameraModel &, int, double>(), "rgb"_a, "depth"_a,
                 "cameraModel"_a, "id"_a, "stamp"_a)
            // .def(nb::init<const cv::Mat &, const cv::Mat &, const cv::Mat &, const CameraModel &, int, double>())
            .def(nb::init<const LaserScan &, const cv::Mat &, const cv::Mat &, const CameraModel &, int, double>(),
                 "laserScan"_a, "rgb"_a, "depth"_a, "cameraModel"_a, "id"_a, "stamp"_a)
            .def(nb::init<const LaserScan &, const cv::Mat &, const cv::Mat &, const cv::Mat &, const CameraModel &,
                          int, double>(),
                 "laserScan"_a, "rgb"_a, "depth"_a, "depthConfidence"_a, "cameraModel"_a, "id"_a, "stamp"_a)
            .def(nb::init<const cv::Mat &, const cv::Mat &, const std::vector<CameraModel> &, int, double>(), "rgb"_a,
                 "depth"_a, "cameraModel"_a, "id"_a, "stamp"_a)
            .def(nb::init<const cv::Mat &, const cv::Mat &, const cv::Mat &, const std::vector<CameraModel> &, int,
                          double>(),
                 "rgb"_a, "depth"_a, "depthConfidence"_a, "cameraModel"_a, "id"_a, "stamp"_a)
            .def(nb::init<const LaserScan &, const cv::Mat &, const cv::Mat &, const std::vector<CameraModel> &, int,
                          double>(),
                 "laserScan"_a, "rgb"_a, "depth"_a, "cameraModel"_a, "id"_a, "stamp"_a)
            .def(nb::init<const LaserScan &, const cv::Mat &, const cv::Mat &, const cv::Mat &,
                          const std::vector<CameraModel> &, int, double>(),
                 "laserScan"_a, "rgb"_a, "depth"_a, "depthConfidence"_a, "cameraModel"_a, "id"_a, "stamp"_a)
            .def(nb::init<const cv::Mat &, const cv::Mat &, const StereoCameraModel &, int, double>(), "left"_a,
                 "right"_a, "cameraModel"_a, "id"_a, "stamp"_a)
            .def(nb::init<const LaserScan &, const cv::Mat &, const cv::Mat &, const StereoCameraModel &, int,
                          double>(),
                 "laserScan"_a, "left"_a, "right"_a, "cameraModel"_a, "id"_a, "stamp"_a)
            .def(nb::init<const cv::Mat &, const cv::Mat &, const std::vector<StereoCameraModel> &, int, double>(),
                 "rgb"_a, "depth"_a, "cameraModel"_a, "id"_a, "stamp"_a)
            .def(nb::init<const LaserScan &, const cv::Mat &, const cv::Mat &, const std::vector<StereoCameraModel> &,
                          int, double>(),
                 "laserScan"_a, "rgb"_a, "depth"_a, "cameraModel"_a, "id"_a, "stamp"_a)
            .def(nb::init<const IMU &, int, double>(), "imu"_a, "id"_a, "stamp"_a)

            .def("isValid", &SensorData::isValid)
            .def("id", &SensorData::id)
            .def("setId", &SensorData::setId, "id"_a)
            .def("stamp", &SensorData::stamp)
            .def("setStamp", &SensorData::setStamp, "stamp"_a)
            .def("imageCompressed", &SensorData::imageCompressed)
            .def("depthOrRightCompressed", &SensorData::depthOrRightCompressed)
            .def("depthConfidenceCompressed", &SensorData::depthConfidenceCompressed)
            .def("laserScanCompressed", &SensorData::laserScanCompressed)
            .def("imageRaw", &SensorData::imageRaw)
            .def("depthOrRightRaw", &SensorData::depthOrRightRaw)
            .def("depthConfidenceRaw", &SensorData::depthConfidenceRaw)
            .def("laserScanRaw", &SensorData::laserScanRaw)
            .def("setRGBDImage",
                 nb::overload_cast<const cv::Mat &, const cv::Mat &, const CameraModel &, bool>(
                         &SensorData::setRGBDImage),
                 "rgb"_a, "depth"_a, "model"_a, "clearPreviousData"_a = true)
            .def("setRGBDImage",
                 nb::overload_cast<const cv::Mat &, const cv::Mat &, const CameraModel &, bool>(
                         &SensorData::setRGBDImage),
                 "rgb"_a, "depth"_a, "model"_a, "clearPreviousData"_a = true)
            .def("setRGBDImage",
                 nb::overload_cast<const cv::Mat &, const cv::Mat &, const std::vector<CameraModel> &, bool>(
                         &SensorData::setRGBDImage),
                 "rgb"_a, "depth"_a, "models"_a, "clearPreviousData"_a = true)
            .def("setRGBDImage",
                 nb::overload_cast<const cv::Mat &, const cv::Mat &, const cv::Mat &, const CameraModel &, bool>(
                         &SensorData::setRGBDImage),
                 "rgb"_a, "depth"_a, "depth_confidence"_a, "model"_a, "clearPreviousData"_a = true)
            .def("setStereoImage",
                 nb::overload_cast<const cv::Mat &, const cv::Mat &, const StereoCameraModel &, bool>(
                         &SensorData::setStereoImage),
                 "left"_a, "right"_a, "stereoCameraModel"_a, "clearPreviousData"_a = true)
            .def("setStereoImage",
                 nb::overload_cast<const cv::Mat &, const cv::Mat &, const std::vector<StereoCameraModel> &, bool>(
                         &SensorData::setStereoImage),
                 "left"_a, "right"_a, "stereoCameraModel"_a, "clearPreviousData"_a = true)
            .def("setLaserScan", &SensorData::setLaserScan, "laserScan"_a, "clearPreviousData"_a = true)
            .def("setCameraModel", &SensorData::setCameraModel, "model"_a)
            .def("setCameraModels", &SensorData::setCameraModels, "models"_a)
            .def("setStereoCameraModel", &SensorData::setStereoCameraModel, "stereoCameraModel"_a)
            .def("setStereoCameraModels", &SensorData::setStereoCameraModels, "stereoCameraModels"_a)
            .def("depthRaw", &SensorData::depthRaw)
            .def("rightRaw", &SensorData::rightRaw)

            .def("uncompressData", nb::overload_cast<>(&SensorData::uncompressData))

            .def("cameraModels", &SensorData::cameraModels)
            .def("stereoCameraModels", &SensorData::stereoCameraModels)

            .def("setUserData", &SensorData::setUserData, "userData"_a, "clearPreviousData"_a = true)
            .def("userDataRaw", &SensorData::userDataRaw)
            .def("userDataCompressed", &SensorData::userDataCompressed)

            .def("setOccupancyGrid", &SensorData::setOccupancyGrid, "ground"_a, "obstacles"_a, "empty"_a, "cellSize"_a,
                 "viewPoint"_a)
            .def("clearOccupancyGridRaw", &SensorData::clearOccupancyGridRaw)
            .def("gridGroundCellsRaw", &SensorData::gridGroundCellsRaw)
            .def("gridGroundCellsCompressed", &SensorData::gridGroundCellsCompressed)
            .def("gridObstacleCellsRaw", &SensorData::gridObstacleCellsRaw)
            .def("gridObstacleCellsCompressed", &SensorData::gridObstacleCellsCompressed)
            .def("gridEmptyCellsRaw", &SensorData::gridEmptyCellsRaw)
            .def("gridEmptyCellsCompressed", &SensorData::gridEmptyCellsCompressed)
            .def("gridCellSize", &SensorData::gridCellSize)
            .def("gridViewPoint", &SensorData::gridViewPoint)

            .def("setFeatures", &SensorData::setFeatures, "keypoints"_a, "keypoints3D"_a, "descriptors"_a)
            .def("keypoints", &SensorData::keypoints)
            .def("keypoints3D", &SensorData::keypoints3D)
            .def("descriptors", &SensorData::descriptors)

            .def("addGlobalDescriptor", &SensorData::addGlobalDescriptor, "descriptor"_a)
            .def("setGlobalDescriptors", &SensorData::setGlobalDescriptors, "descriptors"_a)
            .def("clearGlobalDescriptors", &SensorData::clearGlobalDescriptors)
            .def("globalDescriptors", &SensorData::globalDescriptors)

            .def("setGroundTruth", &SensorData::setGroundTruth, "pose"_a)
            .def("groundTruth", &SensorData::groundTruth)

            .def("setGlobalPose", &SensorData::setGlobalPose, "pose"_a, "covariance"_a)
            .def("globalPose", &SensorData::globalPose)
            .def("globalPoseCovariance", &SensorData::globalPoseCovariance)

            .def("setGPS", &SensorData::setGPS, "gps"_a)
            .def("gps", &SensorData::gps)

            .def("setIMU", &SensorData::setIMU, "imu"_a)
            .def("imu", &SensorData::imu)

            .def("setEnvSensors", &SensorData::setEnvSensors, "sensors"_a)
            .def("addEnvSensor", &SensorData::addEnvSensor, "sensor"_a)
            .def("envSensors", &SensorData::envSensors)

            .def("setLandmarks", &SensorData::setLandmarks, "landmarks"_a)
            .def("landmarks", &SensorData::landmarks)

            .def("getMemoryUsed", &SensorData::getMemoryUsed)
            .def("clearCompressedData", &SensorData::clearCompressedData, "images"_a = true, "scan"_a = true,
                 "userData"_a = true)
            .def("clearRawData", &SensorData::clearRawData, "images"_a = true, "scan"_a = true, "userData"_a = true)
            .def("isPointVisibleFromCameras", &SensorData::isPointVisibleFromCameras, "pt"_a);

    nb::class_<IMU>(m, "IMU")
            .def(nb::init<const cv::Vec4d &,  // qx qy qz qw
                          const cv::Mat &, const cv::Vec3d &, const cv::Mat &, const cv::Vec3d &, const cv::Mat &>(),
                 "orientation"_a, "orientationCovariance"_a, "angularVelocity"_a, "angularVelocityCovariance"_a,
                 "linearAcceleration"_a, "linearAccelerationCovariance"_a)
            .def(nb::init<const cv::Vec3d &, const cv::Mat &, const cv::Vec3d &, const cv::Mat &>(),
                 "angularVelocity"_a, "angularVelocityCovariance"_a, "linearAcceleration"_a,
                 "linearAccelerationCovariance"_a)
            .def("orientation", &IMU::orientation)
            .def("orientationCovariance", &IMU::orientationCovariance)
            .def("angularVelocity", &IMU::angularVelocity)
            .def("angularVelocityCovariance", &IMU::angularVelocityCovariance)
            .def("linearAcceleration", &IMU::linearAcceleration)
            .def("linearAccelerationCovariance", &IMU::linearAccelerationCovariance)
            .def("localTransform", &IMU::localTransform)
            .def("convertToBaseFrame", &IMU::convertToBaseFrame)
            .def("empty", &IMU::empty);

    nb::class_<GPS>(m, "GPS")
            .def(nb::init<>())
            .def(nb::init<const double &, const double &, const double &, const double &, const double &,
                          const double &>(),
                 "stamp"_a, "longitude"_a, "latitude"_a, "altitude"_a, "error"_a, "bearing"_a)

            .def("stamp", &GPS::stamp)
            .def("longitude", &GPS::longitude)
            .def("latitude", &GPS::latitude)
            .def("altitude", &GPS::altitude)
            .def("error", &GPS::error)
            .def("bearing", &GPS::bearing)
            .def("toGeodeticCoords", &GPS::toGeodeticCoords);

    nb::class_<GeodeticCoords>(m, "GeodeticCoords")
            .def(nb::init<>())
            .def(nb::init<double, double, double>(), "latitude"_a, "longitude"_a, "altitude"_a)
            .def("latitude", &GeodeticCoords::latitude)
            .def("longitude", &GeodeticCoords::longitude)
            .def("altitude", &GeodeticCoords::altitude)
            .def("setLatitude", &GeodeticCoords::setLatitude, "value"_a)
            .def("setLongitude", &GeodeticCoords::setLongitude, "value"_a)
            .def("setAltitude", &GeodeticCoords::setAltitude, "value"_a)
            .def("toGeocentric_WGS84", &GeodeticCoords::toGeocentric_WGS84)
            .def("toENU_WGS84", &GeodeticCoords::toENU_WGS84, "origin"_a)
            .def("fromGeocentric_WGS84", &GeodeticCoords::fromGeocentric_WGS84, "geocentric"_a)
            .def("fromENU_WGS84", &GeodeticCoords::fromENU_WGS84, "enu"_a, "origin"_a);

    nb::class_<EnvSensor>(m, "EnvSensor")
            .def(nb::init<>())
            .def(nb::init<const EnvSensor::Type &, const double &, const double &>(), "type"_a, "value"_a,
                 "stamp"_a = 0)

            .def("type", &EnvSensor::type)
            .def("value", &EnvSensor::value)
            .def("stamp", &EnvSensor::stamp);
    nb::enum_<EnvSensor::Type>(m, "EnvSensorType")
            .value("kUndefined", EnvSensor::Type::kUndefined)
            .value("kWifiSignalStrength", EnvSensor::Type::kWifiSignalStrength)
            .value("kAmbientTemperature", EnvSensor::Type::kAmbientTemperature)
            .value("kAmbientAirPressure", EnvSensor::Type::kAmbientAirPressure)
            .value("kAmbientLight", EnvSensor::Type::kAmbientLight)
            .value("kAmbientRelativeHumidity", EnvSensor::Type::kAmbientRelativeHumidity)
            .value("kCustomSensor1", EnvSensor::Type::kCustomSensor1)
            .value("kCustomSensor2", EnvSensor::Type::kCustomSensor2)
            .value("kCustomSensor3", EnvSensor::Type::kCustomSensor3)
            .value("kCustomSensor4", EnvSensor::Type::kCustomSensor4)
            .value("kCustomSensor5", EnvSensor::Type::kCustomSensor5)
            .value("kCustomSensor6", EnvSensor::Type::kCustomSensor6)
            .value("kCustomSensor7", EnvSensor::Type::kCustomSensor7)
            .value("kCustomSensor8", EnvSensor::Type::kCustomSensor8)
            .value("kCustomSensor9", EnvSensor::Type::kCustomSensor9);

    nb::class_<GlobalDescriptor>(m, "GlobalDescriptor")
            .def(nb::init<>())
            .def(nb::init<int, const cv::Mat &, const cv::Mat &>(), "type"_a, "data"_a, "info"_a)
            .def("type", &GlobalDescriptor::type)
            .def("info", &GlobalDescriptor::info)
            .def("data", &GlobalDescriptor::data);

    nb::class_<Landmark>(m, "Landmark")
            .def(nb::init<>())
            .def(nb::init<const int &, const float &, const Transform &, const cv::Mat &>(), "id"_a, "size"_a, "pose"_a,
                 "covariance"_a)
            .def("id", &Landmark::id)
            .def("size", &Landmark::size)
            .def("pose", &Landmark::pose)
            .def("covariance", &Landmark::covariance);
}