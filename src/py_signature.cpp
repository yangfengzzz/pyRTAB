//  Copyright (c) 2025 Feng Yang
//
//  I am making my contributions/submissions to this project solely in my
//  personal capacity and am not conveying any rights to any intellectual
//  property of any third parties.

#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/map.h>
#include <nanobind/stl/list.h>
#include <nanobind/stl/vector.h>
#include <rtabmap/core/Signature.h>
#include "cv_typecaster.h"

namespace nb = nanobind;
using namespace nb::literals;
using namespace rtabmap;

void bindSignature(nb::module_ &m) {
    nb::class_<Signature>(m, "Signature")
            .def(nb::init<>())
            .def(nb::init<int>(), "id"_a)
            .def("compareTo", &Signature::compareTo, "signature"_a)
            .def("isBadSignature", &Signature::isBadSignature)
            .def("id", &Signature::id)
            .def("id", &Signature::id)
            .def("mapId", &Signature::mapId)
            .def("setWeight", &Signature::setWeight, "weight"_a)
            .def("getWeight", &Signature::getWeight)
            .def("setLabel", &Signature::setLabel, "label"_a)
            .def("getLabel", &Signature::getLabel)
            .def("getStamp", &Signature::getStamp)

            .def("addLinks", nb::overload_cast<const std::list<Link> &>(&Signature::addLinks), "links"_a)
            .def("addLinks", nb::overload_cast<const std::map<int, Link> &>(&Signature::addLinks), "links"_a)
            .def("addLink", &Signature::addLink, "link"_a)

            .def("hasLink", &Signature::hasLink, "idTo"_a, "type"_a)

            .def("changeLinkIds", &Signature::changeLinkIds, "idFrom"_a, "idTo"_a)

            .def("removeLinks", &Signature::removeLinks, "keepSelfReferringLinks"_a = false)
            .def("removeLink", &Signature::removeLink, "idTo"_a)
            .def("removeVirtualLinks", &Signature::removeVirtualLinks)

            .def("addLandmark", &Signature::addLandmark, "landmark"_a)
            .def("getLandmarks", &Signature::getLandmarks)
            .def("removeLandmarks", &Signature::removeLandmarks)
            .def("removeLandmark", &Signature::removeLandmark, "landmarkId"_a)

            .def("setSaved", &Signature::setSaved, "saved"_a)
            .def("setModified", &Signature::setModified, "modified"_a)

            // .def("getLinks", &Signature::getLinks)
            .def("isSaved", &Signature::isSaved)
            .def("isModified", &Signature::isModified)
            .def("isLinksModified", &Signature::isLinksModified)

            .def("removeAllWords", &Signature::removeAllWords)
            .def("changeWordsRef", &Signature::changeWordsRef)
            // .def("setWords", &Signature::setWords)
            .def("isEnabled", &Signature::isEnabled)
            .def("setEnabled", &Signature::setEnabled, "enabled"_a)
            // .def("getWords", &Signature::getWords)
            .def("getWordsKpts", &Signature::getWordsKpts)
            .def("getInvalidWordsCount", &Signature::getInvalidWordsCount)
            .def("getWordsChanged", &Signature::getWordsChanged)
            .def("getWordsDescriptors", &Signature::getWordsDescriptors)
            .def("setWordsDescriptors", &Signature::setWordsDescriptors, "descriptors"_a)

            .def("setPose", &Signature::setPose, "pose"_a)
            .def("setGroundTruthPose", &Signature::setGroundTruthPose, "pose"_a)
            .def("setVelocity", &Signature::setVelocity, "vx"_a, "vy"_a, "vz"_a, "vroll"_a, "vpitch"_a, "vyaw"_a)

            .def("getWords3", &Signature::getWords3)
            .def("getPose", &Signature::getPose)
            .def("getPoseCovariance", &Signature::getPoseCovariance)
            .def("getGroundTruthPose", &Signature::getGroundTruthPose)
            .def("getVelocity", &Signature::getVelocity)

            .def("sensorData", nb::overload_cast<>(&Signature::sensorData, nb::const_))

            .def("getMemoryUsed", &Signature::getMemoryUsed, "withSensorData"_a = true);
}