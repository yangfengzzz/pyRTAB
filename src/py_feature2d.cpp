//  Copyright (c) 2025 Feng Yang
//
//  I am making my contributions/submissions to this project solely in my
//  personal capacity and am not conveying any rights to any intellectual
//  property of any third parties.

#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/map.h>
#include <rtabmap/core/Features2d.h>
#include "cv_typecaster.h"

namespace nb = nanobind;
using namespace nb::literals;
using namespace rtabmap;

void bindFeature2D(nb::module_ &m) {
    nb::class_<Feature2D>(m, "Feature2D")
            .def_static("create", [](Feature2D::Type type) { return Feature2D::create(type); })
            .def("getMaxFeatures", &Feature2D::getMaxFeatures)
            .def("getSSC", &Feature2D::getSSC)
            .def("getMinDepth", &Feature2D::getMinDepth)
            .def("getMaxDepth", &Feature2D::getMaxDepth)
            .def("getGridRows", &Feature2D::getGridRows)
            .def("getGridCols", &Feature2D::getGridCols)
            .def("generateKeypoints", &Feature2D::generateKeypoints, "image"_a, "mask"_a)
            .def("generateDescriptors", &Feature2D::generateDescriptors, "image"_a, "keypoints"_a)
            .def("generateKeypoints3D", &Feature2D::generateKeypoints3D, "data"_a, "keypoints"_a)
            .def("parseParameters", &Feature2D::parseParameters, "parameters"_a)
            .def("getParameters", &Feature2D::getParameters);

    nb::enum_<Feature2D::Type>(m, "Feature2DType")
            .value("kFeatureUndef", Feature2D::Type::kFeatureUndef)
            .value("kFeatureSurf", Feature2D::Type::kFeatureSurf)
            .value("kFeatureSift", Feature2D::Type::kFeatureSift)
            .value("kFeatureOrb", Feature2D::Type::kFeatureOrb)
            .value("kFeatureFastFreak", Feature2D::Type::kFeatureFastFreak)
            .value("kFeatureFastBrief", Feature2D::Type::kFeatureFastBrief)
            .value("kFeatureGfttFreak", Feature2D::Type::kFeatureGfttFreak)
            .value("kFeatureGfttBrief", Feature2D::Type::kFeatureGfttBrief)
            .value("kFeatureBrisk", Feature2D::Type::kFeatureBrisk)
            .value("kFeatureGfttOrb", Feature2D::Type::kFeatureGfttOrb)
            .value("kFeatureKaze", Feature2D::Type::kFeatureKaze)
            .value("kFeatureOrbOctree", Feature2D::Type::kFeatureOrbOctree)
            .value("kFeatureSuperPointTorch", Feature2D::Type::kFeatureSuperPointTorch)
            .value("kFeatureSurfFreak", Feature2D::Type::kFeatureSurfFreak)
            .value("kFeatureGfttDaisy", Feature2D::Type::kFeatureGfttDaisy)
            .value("kFeatureSurfDaisy", Feature2D::Type::kFeatureSurfDaisy)
            .value("kFeaturePyDetector", Feature2D::Type::kFeaturePyDetector);
}