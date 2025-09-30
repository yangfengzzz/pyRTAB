//  Copyright (c) 2025 Feng Yang
//
//  I am making my contributions/submissions to this project solely in my
//  personal capacity and am not conveying any rights to any intellectual
//  property of any third parties.

#include <nanobind/nanobind.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/map.h>
#include <nanobind/stl/set.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/list.h>
#include <nanobind/stl/pair.h>

#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/Memory.h>

namespace nb = nanobind;
using namespace rtabmap;

using namespace nb::literals;

void bindOdometry(nb::module_ &m);
void bindTransform(nb::module_ &m);
void bindCameraModel(nb::module_ &m);
void bindSensorData(nb::module_ &m);
void bindLaserScan(nb::module_ &m);
void bindStereoCameraModel(nb::module_ &m);

NB_MODULE(rtab_ext, m) {
    m.doc() = "This is a \"hello world\" example with nanobind";
    m.def("add", [](int a, int b) { return a + b; }, "a"_a, "b"_a);

    bindOdometry(m);
    bindTransform(m);
    bindCameraModel(m);
    bindSensorData(m);
    bindLaserScan(m);
    bindStereoCameraModel(m);

    nb::class_<Rtabmap>(m, "Rtabmap")
            .def(nb::init<>())
            .def("process",
                 nb::overload_cast<const SensorData &, Transform, const cv::Mat &, const std::vector<float> &,
                                   const std::map<std::string, float> &>(&Rtabmap::process),
                 "data"_a, "odomPose"_a, "odomCovariance"_a, "odomVelocity"_a, "externalStats"_a)
            .def("process",
                 nb::overload_cast<const SensorData &, Transform, float, float, const std::vector<float> &,
                                   const std::map<std::string, float> &>(&Rtabmap::process),
                 "data"_a, "odomPose"_a, "odomLinearVariance"_a, "odomAngularVariance"_a, "odomVelocity"_a,
                 "externalStats"_a)
            .def("process",
                 nb::overload_cast<const cv::Mat &, int, const std::map<std::string, float> &>(&Rtabmap::process),
                 "image"_a, "id"_a, "externalStats"_a)
            .def("init", nb::overload_cast<const ParametersMap &, const std::string &, bool>(&Rtabmap::init),
                 "parameters"_a, "databasePath"_a, "loadDatabaseParameters"_a)
            .def("init", nb::overload_cast<const std::string &, const std::string &, bool>(&Rtabmap::init),
                 "configFile"_a, "databasePath"_a, "loadDatabaseParameters"_a)
            .def("close", &Rtabmap::close)
            .def("getWorkingDir", &Rtabmap::getWorkingDir)
            .def("isRGBDMode", &Rtabmap::isRGBDMode)
            .def("getLoopClosureId", &Rtabmap::getLoopClosureId)
            .def("getLoopClosureValue", &Rtabmap::getLoopClosureValue)
            .def("getHighestHypothesisId", &Rtabmap::getHighestHypothesisId)
            .def("getHighestHypothesisValue", &Rtabmap::getHighestHypothesisValue)
            .def("getLastLocationId", &Rtabmap::getLastLocationId)
            .def("getWM", &Rtabmap::getWM)
            .def("getSTM", &Rtabmap::getSTM)
            .def("getWMSize", &Rtabmap::getWMSize)
            .def("getSTMSize", &Rtabmap::getSTMSize)
            .def("getWeights", &Rtabmap::getWeights)
            .def("getTotalMemSize", &Rtabmap::getTotalMemSize)
            .def("getLastProcessTime", &Rtabmap::getLastProcessTime)
            .def("isInSTM", &Rtabmap::isInSTM)
            .def("isIDsGenerated", &Rtabmap::isIDsGenerated)
            .def("getStatistics", &Rtabmap::getStatistics)
            .def("getLocalOptimizedPoses", &Rtabmap::getLocalOptimizedPoses)
            // .def("getLocalConstraints", &Rtabmap::getLocalConstraints)
            .def("getPose", &Rtabmap::getPose)
            .def("getMapCorrection", &Rtabmap::getMapCorrection)
            .def("getMemory", &Rtabmap::getMemory)
            .def("getGoalReachedRadius", &Rtabmap::getGoalReachedRadius)
            .def("getLocalRadius", &Rtabmap::getLocalRadius)
            .def("getLastLocalizationPose", &Rtabmap::getLastLocalizationPose)
            .def("getTimeThreshold", &Rtabmap::getTimeThreshold)
            .def("setTimeThreshold", &Rtabmap::setTimeThreshold)
            .def("getMemoryThreshold", &Rtabmap::getMemoryThreshold)
            .def("setMemoryThreshold", &Rtabmap::setMemoryThreshold)
            .def("setInitialPose", &Rtabmap::setInitialPose)
            .def("triggerNewMap", &Rtabmap::triggerNewMap)
            .def("labelLocation", &Rtabmap::labelLocation)
            .def("setUserData", &Rtabmap::setUserData)
            .def("generateDOTGraph", &Rtabmap::generateDOTGraph)
            .def("exportPoses", &Rtabmap::exportPoses)
            .def("resetMemory", &Rtabmap::resetMemory)
            .def("dumpPrediction", &Rtabmap::dumpPrediction)
            .def("dumpData", &Rtabmap::dumpData)
            .def("parseParameters", &Rtabmap::parseParameters)
            .def("getParameters", &Rtabmap::getParameters)
            .def("setWorkingDirectory", &Rtabmap::setWorkingDirectory)
            .def("rejectLastLoopClosure", &Rtabmap::rejectLastLoopClosure)
            .def("deleteLastLocation", &Rtabmap::deleteLastLocation)
            // .def("setOptimizedPoses", &Rtabmap::setOptimizedPoses)
            .def("getSignatureCopy", &Rtabmap::getSignatureCopy)
            // .def("getGraph", &Rtabmap::getGraph)
            .def("getNodesInRadius",
                 nb::overload_cast<const Transform &, float, int, std::map<int, float> *>(&Rtabmap::getNodesInRadius),
                 "pose"_a, "radius"_a, "k"_a, "distsSqr"_a)
            .def("getNodesInRadius",
                 nb::overload_cast<int, float, int, std::map<int, float> *>(&Rtabmap::getNodesInRadius), "nodeId"_a,
                 "radius"_a, "k"_a, "distsSqr"_a)
            .def("detectMoreLoopClosures", &Rtabmap::detectMoreLoopClosures)
            .def("globalBundleAdjustment", &Rtabmap::globalBundleAdjustment)
            .def("cleanupLocalGrids", &Rtabmap::cleanupLocalGrids)
            .def("refineLinks", &Rtabmap::refineLinks)
            .def("addLink", &Rtabmap::addLink)
            .def("getInformation", &Rtabmap::getInformation)
            .def("addNodesToRepublish", &Rtabmap::addNodesToRepublish)
            .def("getPathStatus", &Rtabmap::getPathStatus)
            .def("clearPath", &Rtabmap::clearPath)
            .def("computePath", nb::overload_cast<int, bool>(&Rtabmap::computePath), "targetNode"_a, "global"_a)
            .def("computePath", nb::overload_cast<const Transform &, float>(&Rtabmap::computePath), "targetPose"_a,
                 "tolerance"_a)
            .def("getPath", &Rtabmap::getPath)
            .def("getPathNextPoses", &Rtabmap::getPathNextPoses)
            .def("getPathNextNodes", &Rtabmap::getPathNextNodes)
            .def("getPathCurrentGoalId", &Rtabmap::getPathCurrentGoalId)
            .def("getPathCurrentIndex", &Rtabmap::getPathCurrentIndex)
            .def("getPathCurrentGoalIndex", &Rtabmap::getPathCurrentGoalIndex)
            .def("getPathTransformToGoal", &Rtabmap::getPathTransformToGoal)
            .def("getForwardWMPoses", &Rtabmap::getForwardWMPoses)
            .def("getPaths", &Rtabmap::getPaths)
            .def("adjustLikelihood", &Rtabmap::adjustLikelihood);
            // .def("selectHypothesis", &Rtabmap::selectHypothesis);
}