//  Copyright (c) 2025 Feng Yang
//
//  I am making my contributions/submissions to this project solely in my
//  personal capacity and am not conveying any rights to any intellectual
//  property of any third parties.

#include <nanobind/nanobind.h>
#include <nanobind/stl/map.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/string.h>
#include <rtabmap/core/Odometry.h>
#include <rtabmap/core/OdometryInfo.h>
#include "cv_typecaster.h"

namespace nb = nanobind;
using namespace nb::literals;
using namespace rtabmap;

void bindOdometry(nb::module_ &m) {
    nb::enum_<Odometry::Type>(m, "OdometryType")
            .value("kTypeUndef", Odometry::kTypeUndef)
            .value("kTypeF2M", Odometry::kTypeF2M)
            .value("kTypeF2F", Odometry::kTypeF2F)
            .value("kTypeFovis", Odometry::kTypeFovis)
            .value("kTypeViso2", Odometry::kTypeViso2)
            .value("kTypeDVO", Odometry::kTypeDVO)
            .value("kTypeORBSLAM", Odometry::kTypeORBSLAM)
            .value("kTypeOkvis", Odometry::kTypeOkvis)
            .value("kTypeLOAM", Odometry::kTypeLOAM)
            .value("kTypeMSCKF", Odometry::kTypeMSCKF)
            .value("kTypeVINSFusion", Odometry::kTypeVINSFusion)
            .value("kTypeOpenVINS", Odometry::kTypeOpenVINS)
            .value("kTypeFLOAM", Odometry::kTypeFLOAM)
            .value("kTypeOpen3D", Odometry::kTypeOpen3D);

    nb::class_<Odometry>(m, "Odometry")
            .def_static("create", []() { return Odometry::create(); })
            .def_static(
                    "create", [](Odometry::Type &type) { return Odometry::create(type); }, "type"_a)
            .def("process", nb::overload_cast<SensorData &, OdometryInfo *>(&Odometry::process), "data"_a, "info"_a)
            .def("process", nb::overload_cast<SensorData &, const Transform &, OdometryInfo *>(&Odometry::process),
                 "data"_a, "guess"_a, "info"_a)
            .def("reset", &Odometry::reset, "initialPose"_a)
            .def("getType", &Odometry::getType)
            .def("canProcessRawImages", &Odometry::canProcessRawImages)
            .def("canProcessAsyncIMU", &Odometry::canProcessAsyncIMU)
            .def("getPose", &Odometry::getPose)
            .def("isInfoDataFilled", &Odometry::isInfoDataFilled)
            .def("getVelocityGuess", &Odometry::getVelocityGuess)
            .def("previousStamp", &Odometry::previousStamp)
            .def("framesProcessed", &Odometry::framesProcessed)
            .def("imagesAlreadyRectified", &Odometry::imagesAlreadyRectified);

    nb::class_<OdometryInfo>(m, "OdometryInfo")
            .def(nb::init<>())
            .def("statistics", &OdometryInfo::statistics, "pose"_a)
            .def_rw("lost", &OdometryInfo::lost)
            .def_rw("reg", &OdometryInfo::reg)
            .def_rw("features", &OdometryInfo::features)
            .def_rw("localMapSize", &OdometryInfo::localMapSize)
            .def_rw("localScanMapSize", &OdometryInfo::localScanMapSize)
            .def_rw("localKeyFrames", &OdometryInfo::localKeyFrames)
            .def_rw("localBundleOutliers", &OdometryInfo::localBundleOutliers)
            .def_rw("localBundleConstraints", &OdometryInfo::localBundleConstraints)
            .def_rw("localBundleTime", &OdometryInfo::localBundleTime)
            .def_rw("localBundlePoses", &OdometryInfo::localBundlePoses)
            .def_rw("localBundleModels", &OdometryInfo::localBundleModels)
            .def_rw("localBundleAvgInlierDistance", &OdometryInfo::localBundleAvgInlierDistance)
            .def_rw("localBundleMaxKeyFramesForInlier", &OdometryInfo::localBundleMaxKeyFramesForInlier)
            .def_rw("localBundleOutliersPerCam", &OdometryInfo::localBundleOutliersPerCam)
            .def_rw("keyFrameAdded", &OdometryInfo::keyFrameAdded)
            .def_rw("timeDeskewing", &OdometryInfo::timeDeskewing)
            .def_rw("timeEstimation", &OdometryInfo::timeEstimation)
            .def_rw("timeParticleFiltering", &OdometryInfo::timeParticleFiltering)
            .def_rw("stamp", &OdometryInfo::stamp)
            .def_rw("interval", &OdometryInfo::interval)
            .def_rw("transform", &OdometryInfo::transform)
            .def_rw("transformFiltered", &OdometryInfo::transformFiltered)
            .def_rw("transformGroundTruth", &OdometryInfo::transformGroundTruth)
            .def_rw("guessVelocity", &OdometryInfo::guessVelocity)
            .def_rw("guess", &OdometryInfo::guess)
            .def_rw("distanceTravelled", &OdometryInfo::distanceTravelled)
            .def_rw("memoryUsage", &OdometryInfo::memoryUsage)
            .def_rw("gravityRollError", &OdometryInfo::gravityRollError)
            .def_rw("gravityPitchError", &OdometryInfo::gravityPitchError)
            .def_rw("type", &OdometryInfo::type)
            // .def_rw("words", &OdometryInfo::words)
            .def_rw("localMap", &OdometryInfo::localMap)
            .def_rw("localScanMap", &OdometryInfo::localScanMap)
            .def_rw("refCorners", &OdometryInfo::refCorners)
            .def_rw("newCorners", &OdometryInfo::newCorners)
            .def_rw("cornerInliers", &OdometryInfo::cornerInliers);

    nb::class_<RegistrationInfo>(m, "RegistrationInfo")
            .def(nb::init<>())
            .def("copyWithoutData", &RegistrationInfo::copyWithoutData)
            .def_rw("covariance", &RegistrationInfo::covariance)
            .def_rw("rejectedMsg", &RegistrationInfo::rejectedMsg)
            .def_rw("totalTime", &RegistrationInfo::totalTime)
            .def_rw("inliers", &RegistrationInfo::inliers)
            .def_rw("inliersRatio", &RegistrationInfo::inliersRatio)
            .def_rw("inliersMeanDistance", &RegistrationInfo::inliersMeanDistance)
            .def_rw("inliersDistribution", &RegistrationInfo::inliersDistribution)
            .def_rw("inliersIDs", &RegistrationInfo::inliersIDs)
            .def_rw("matches", &RegistrationInfo::matches)
            .def_rw("matchesIDs", &RegistrationInfo::matchesIDs)
            .def_rw("projectedIDs", &RegistrationInfo::projectedIDs)
            .def_rw("inliersPerCam", &RegistrationInfo::inliersPerCam)
            .def_rw("matchesPerCam", &RegistrationInfo::matchesPerCam)
            .def_rw("icpInliersRatio", &RegistrationInfo::icpInliersRatio)
            .def_rw("icpTranslation", &RegistrationInfo::icpTranslation)
            .def_rw("icpRotation", &RegistrationInfo::icpRotation)
            .def_rw("icpStructuralComplexity", &RegistrationInfo::icpStructuralComplexity)
            .def_rw("icpStructuralDistribution", &RegistrationInfo::icpStructuralDistribution)
            .def_rw("icpCorrespondences", &RegistrationInfo::icpCorrespondences)
            .def_rw("icpRMS", &RegistrationInfo::icpRMS);
}