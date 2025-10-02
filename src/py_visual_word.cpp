//  personal capacity and am not conveying any rights to any intellectual
//  property of any third parties.

#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/map.h>
#include <rtabmap/core/VisualWord.h>
#include "cv_typecaster.h"

namespace nb = nanobind;
using namespace nb::literals;
using namespace rtabmap;

void bindVisualWord(nb::module_ &m) {
    nb::class_<VisualWord>(m, "VisualWord")
            .def(nb::init<int, const cv::Mat &, int>(), "id"_a, "descriptor"_a, "signatureId"_a = 0)
            .def("addRef", &VisualWord::addRef, "signatureId"_a)
            .def("removeAllRef", &VisualWord::removeAllRef, "signatureId"_a)
            .def("getMemoryUsed", &VisualWord::getMemoryUsed)
            .def("getTotalReferences", &VisualWord::getTotalReferences)
            .def("id", &VisualWord::id)
            .def("getDescriptor", &VisualWord::getDescriptor)
            .def("getReferences", &VisualWord::getReferences)
            .def("isSaved", &VisualWord::isSaved)
            .def("setSaved", &VisualWord::setSaved, "saved"_a);
}