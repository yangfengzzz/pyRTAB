//  Copyright (c) 2025 Feng Yang
//
//  I am making my contributions/submissions to this project solely in my
//  personal capacity and am not conveying any rights to any intellectual
//  property of any third parties.

#include <iostream>
#include <nanobind/nanobind.h>

namespace nb = nanobind;

using namespace nb::literals;

void bindOdometry(nb::module_ &m);
void bindTransform(nb::module_ &m);

NB_MODULE(rtab_ext, m) {
    m.doc() = "This is a \"hello world\" example with nanobind";
    m.def("add", [](int a, int b) { return a + b; }, "a"_a, "b"_a);

    bindOdometry(m);
    bindTransform(m);
}