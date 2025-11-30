from conan import ConanFile
from conan.tools.cmake import cmake_layout


class ExampleRecipe(ConanFile):
    settings = "os", "compiler", "build_type", "arch"
    generators = "CMakeDeps", "CMakeToolchain"

    def requirements(self):
        self.requires("motioncontroller/1.0.1")
        # self.requires("yaml-cpp/0.8.0")
        # self.requires("aimrt/1.1.0")
        # self.requires("jsoncpp/1.9.6")

    def layout(self):
        cmake_layout(self)
