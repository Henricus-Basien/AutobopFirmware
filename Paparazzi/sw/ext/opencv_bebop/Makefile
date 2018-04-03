
PWD	= $(shell pwd)

# Building needs two settings: the crosscompiler and the target directory
# both can be set from command line: make BUILD_DIR=./build_parrot CMAKE_FILE=bebop.toolchain.cmake.none-linux

BUILD_DIR ?= ./build

ifeq ($(OS),Windows_NT)
  $(warning Warning: OpenCV compilation on Windows not supported)
else
  UNAME_S := $(shell uname -s)
  ifeq ($(UNAME_S),Linux)
    CMAKE_FILE     ?= bebop.toolchain.cmake.linux
  else
    ifeq ($(UNAME_S),Darwin)
      CMAKE_FILE     ?= bebop.toolchain.cmake.osx
    endif
  endif
endif


all:
	git submodule init
	git submodule update
	cd opencv && git am --signoff < ../fix_compiler_crash.patch && cd ..
	make cc
	make build
	./link.py > install/opencv.xml

build:
	make -C $(BUILD_DIR)
	make -C $(BUILD_DIR) install

cs:
	make BUILD_DIR=./build_parrot CMAKE_FILE=bebop.toolchain.cmake.none-linux

osx:
	make BUILD_DIR=./build_osx CMAKE_FILE=bebop.toolchain.cmake.osx

cc:
	mkdir -p $(BUILD_DIR);
	cmake -H./opencv -B$(BUILD_DIR) -DCMAKE_TOOLCHAIN_FILE=$(PWD)/$(CMAKE_FILE) \
		 -DCMAKE_INSTALL_PREFIX=$(PWD)/install \
		 -DBUILD_CUDA_STUBS=FALSE \
		 -DBUILD_DOCS=FALSE  \
		 -DBUILD_EXAMPLES=FALSE \
		 -DBUILD_FAT_JAVA_LIB=TRUE \
		 -DBUILD_JASPER=FALSE \
		 -DBUILD_JPEG=FALSE \
		 -DBUILD_OPENEXR=FALSE \
		 -DBUILD_PACKAGE=FALSE \
		 -DBUILD_PERF_TESTS=FALSE \
		 -DBUILD_PNG=FALSE \
		 -DBUILD_SHARED_LIBS=FALSE \
		 -DBUILD_TBB=FALSE \
		 -DBUILD_TESTS=FALSE \
		 -DBUILD_TIFF=FALSE \
		 -DBUILD_WITH_DEBUG_INFO=FALSE \
		 -DBUILD_WITH_DYNAMIC_IPP=FALSE \
		 -DBUILD_ZLIB=FALSE \
		 -DBUILD_opencv_apps=TRUE \
		 -DBUILD_opencv_calib3d=TRUE \
		 -DBUILD_opencv_core=TRUE \
		 -DBUILD_opencv_features2d=TRUE \
		 -DBUILD_opencv_flann=TRUE \
		 -DBUILD_opencv_highgui=FALSE \
		 -DBUILD_opencv_imgcodecs=TRUE \
		 -DBUILD_opencv_imgproc=TRUE \
		 -DBUILD_opencv_ximgproc=TRUE \
		 -DBUILD_opencv_dnn=TRUE \
		 -DBUILD_opencv_optflow=TRUE \
		 -DBUILD_opencv_bgsegm=TRUE \
		 -DBUILD_opencv_bioinspired=TRUE \
		 -DBUILD_opencv_tracking=TRUE \
		 -DBUILD_opencv_java=FALSE \
		 -DBUILD_opencv_ml=TRUE \
		 -DBUILD_opencv_objdetect=TRUE \
		 -DBUILD_opencv_xobjdetect=TRUE \
		 -DBUILD_opencv_photo=TRUE \
		 -DBUILD_opencv_shape=TRUE \
		 -DBUILD_opencv_stitching=TRUE \
		 -DBUILD_opencv_superres=TRUE \
		 -DBUILD_opencv_ts=FALSE \
		 -DBUILD_opencv_video=TRUE \
		 -DBUILD_opencv_videoio=TRUE \
		 -DBUILD_opencv_videostab=TRUE \
		 -DBUILD_opencv_world=TRUE \
		 -DCUDA_BUILD_CUBIN=FALSE \
		 -DCUDA_BUILD_EMULATION=FALSE \
		 -DCUDA_SEPARABLE_COMPILATION=FALSE \
		 -DCUDA_VERBOSE_BUILD=FALSE \
		 -DDOWNLOAD_EXTERNAL_TEST_DATA=FALSE \
		 -DENABLE_NEON=TRUE \
		 -DENABLE_AVX=FALSE \
		 -DENABLE_AVX2=FALSE \
		 -DENABLE_COVERAGE=FALSE \
		 -DENABLE_FAST_MATH=FALSE \
		 -DENABLE_IMPL_COLLECTION=FALSE \
		 -DENABLE_NOISY_WARNINGS=FALSE \
		 -DENABLE_OMIT_FRAME_POINTER=FALSE \
		 -DENABLE_POPCNT=FALSE \
		 -DENABLE_PRECOMPILED_HEADERS=FALSE \
		 -DENABLE_PROFILING=FALSE \
		 -DENABLE_SOLUTION_FOLDERS=FALSE \
		 -DENABLE_SSE=FALSE \
		 -DENABLE_SSE2=FALSE \
		 -DENABLE_SSE3=FALSE \
		 -DENABLE_SSE41=FALSE \
		 -DENABLE_SSE42=FALSE \
		 -DENABLE_SSSE3=FALSE \
		 -DINSTALL_CREATE_DISTRIB=FALSE \
		 -DINSTALL_C_EXAMPLES=TRUE \
		 -DINSTALL_PYTHON_EXAMPLES=FALSE \
		 -DINSTALL_TESTS=FALSE \
		 -DOPENCV_WARNINGS_ARE_ERRORS=FALSE \
		 -DWITH_1394=FALSE \
		 -DWITH_CLP=FALSE \
		 -DWITH_CUBLAS=FALSE \
		 -DWITH_CUDA=FALSE \
		 -DWITH_CUFFT=FALSE \
		 -DWITH_EIGEN=FALSE \
		 -DWITH_FFMPEG=FALSE \
		 -DWITH_GDAL=FALSE \
		 -DWITH_GIGEAPI=FALSE \
		 -DWITH_GPHOTO2=FALSE \
		 -DWITH_GSTREAMER=TRUE \
		 -DWITH_GSTREAMER_0_10=FALSE \
		 -DWITH_GTK=FALSE \
		 -DWITH_GTK_2_X=FALSE \
		 -DWITH_IPP=FALSE \
		 -DWITH_IPP_A=FALSE \
		 -DWITH_JASPER=FALSE \
		 -DWITH_ZLIB=TRUE \
		 -DWITH_JPEG=TRUE \
		 -DWITH_LIBV4L=TRUE \
		 -DWITH_MATLAB=FALSE \
		 -DWITH_NVCUVID=FALSE \
		 -DWITH_OPENCL=FALSE \
		 -DWITH_OPENCLAMDBLAS=FALSE \
		 -DWITH_OPENCLAMDFFT=FALSE \
		 -DWITH_OPENCL_SVM=FALSE \
		 -DWITH_OPENEXR=FALSE \
		 -DWITH_OPENGL=TRUE \
		 -DWITH_OPENMP=FALSE \
		 -DWITH_OPENNI=FALSE \
		 -DWITH_OPENNI2=FALSE \
		 -DWITH_PNG=TRUE \
		 -DWITH_PTHREADS_PF=TRUE \
		 -DWITH_PVAPI=FALSE \
		 -DWITH_QT=FALSE \
		 -DWITH_TBB=FALSE \
		 -DWITH_TIFF=TRUE \
		 -DWITH_UNICAP=FALSE \
		 -DWITH_V4L=TRUE \
		 -DWITH_VA=FALSE \
		 -DWITH_VA_INTEL=FALSE \
		 -DWITH_VTK=FALSE \
		 -DWITH_WEBP=FALSE \
		 -DWITH_XIMEA=FALSE \
		 -DWITH_XINE=FALSE



patch:
	cd opencv && git format-patch 3.2.0 --stdout > ../fix_compiler_crash.patch && cd ..


clean:
	rm -rf ./build
	rm -rf ./build_parrot
	rm -rf ./install
	rm -rf *~

.PHONY: build cc clean
