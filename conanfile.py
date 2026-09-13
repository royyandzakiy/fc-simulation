from conan import ConanFile

class MyProjectConan(ConanFile):
    settings = "os", "compiler", "build_type", "arch"
    generators = "CMakeConfigDeps"

    def requirements(self):
        self.requires("fmt/12.1.0")
        self.requires("gtest/1.17.0")
        self.requires("cli11/2.6.2")
        self.requires("sdl/3.4.14")