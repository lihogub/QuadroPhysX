from conans import ConanFile, CMake
import traceback


class QuadroPhysX(ConanFile):
    name = "QuadroPhysX"
    version = "0.0.1"
    settings = "os", "compiler", "build_type", "arch", "cppstd"
    requires = "physx/4.1.1"
    generators = "cmake"
