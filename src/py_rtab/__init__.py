import ctypes
import os
import sys

# install_path = os.path.join(os.path.dirname(__file__), '../..')
# postfix = ""
# if sys.platform.startswith('linux'):
#     postfix = ".so"
#     ctypes.CDLL("libcuda.so", ctypes.RTLD_GLOBAL)
#     ctypes.CDLL(install_path + "/libusd_hd" + postfix, ctypes.RTLD_LOCAL)
#     ctypes.CDLL(install_path + "/libusd_usdHydra" + postfix, ctypes.RTLD_LOCAL)
#     ctypes.CDLL(install_path + "/libusd_usdPhysicsImaging" + postfix, ctypes.RTLD_LOCAL)
#
# if sys.platform.startswith('darwin'):
#     postfix = ".dylib"
#
# ctypes.CDLL(install_path + "/libusd_boost" + postfix, ctypes.RTLD_LOCAL)


from .rtab_ext import *