import os
import platform
import ctypes
import subprocess
import distutils.command.build_py
from distutils.core import setup


class build_trical(distutils.command.build_py.build_py):
    description = """Build the TRICAL shared library"""

    def run(self):
        subprocess.call("cmake . && make && cp libTRICAL.* ./python/TRICAL/",
                        shell=True,
                        cwd=os.path.dirname(os.path.abspath(__file__)))
        self.data_files = self.get_data_files()
        distutils.command.build_py.build_py.run(self)


setup(
    name="trical",
    url="https://github.com/sfwa/TRICAL",
    author="Ben Dyer",
    author_email="",
    version="1.0.0",
    description="""UKF-based real-time scale and bias calibration algorithm \
for tri-axial field sensors (e.g. magnetometers).""",
    long_description=open("README.md").read(),
    package_dir={"": "python"},
    packages=["TRICAL"],
    package_data={
        "TRICAL": ["TRICAL.dll", "libTRICAL.so", "libTRICAL.dylib"]
    },
    license="MIT License",
    classifiers=[
        "Development Status :: 4 - Beta",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
        "Programming Language :: Python",
        "Topic :: Software Development :: Libraries"
    ],
    cmdclass={"build_py": build_trical}
)
