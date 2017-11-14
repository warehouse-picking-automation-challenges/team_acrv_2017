# Modify INCLUDE_DIRS in Makefile.config to the following
INCLUDE_DIRS := $(PYTHON_INCLUDE) /usr/local/include /usr/include/hdf5/serial/ /opt/ros/kinetic/include/opencv-3.2.0-dev
LIBRARY_DIRS := $(PYTHON_LIB) /usr/local/lib /usr/lib /opt/ros/kinetic/lib

# Modify BLAS to open if using OpenBlas, leave as ATLAS for default

# Create symlinks from hdf5_serial to hdf5
cd /usr/lib/x86_64-linux-gnu
sudo ln -s libhdf5_serial.so libhdf5.so
sudo ln -s libhdf5_serial_hl.so libhdf5_hl.so

# Create symlinks from opencv3 to opencv
cd /opt/ros/kinetic/lib
sudo ln -s ./libopencv_core3.so ./libopencv_core.so
sudo ln -s ./libopencv_highgui3.so ./libopencv_highgui.so
sudo ln -s ./libopencv_imgproc3.so ./libopencv_imgproc.so
sudo ln -s ./libopencv_imgcodecs3.so ./libopencv_imgcodecs.so
