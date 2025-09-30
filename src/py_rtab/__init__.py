import ctypes
import os
import sys

install_path = os.path.join(os.path.dirname(__file__), '')
postfix = ""
if sys.platform.startswith('linux'):
    postfix = ".so"

if sys.platform.startswith('darwin'):
    postfix = ".dylib"

ctypes.CDLL(install_path + "/librtabmap_utilite" + postfix, ctypes.RTLD_LOCAL)
ctypes.CDLL(install_path + "/librtabmap_core" + postfix, ctypes.RTLD_LOCAL)

from .rtab_ext import *
