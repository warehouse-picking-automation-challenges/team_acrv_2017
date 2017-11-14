# realsense_tools

Repository for cross-platform realsense tools used by ACRV.

## Build Instructions

From the root of this repository:

```
mkdir build
cd build
cmake ../
make
```

## Usage Instructions
### capture_checkerboard_images

From acrv_apc_2017/acrv_realsense_tools:

```
cd build/
./capture_checkerboard_images
```
* Move the checkerboard around so that both the IR and colour video streams can connect the points on the checkerboard.
* A capture will only be taken if checkerboard is identified in both views and about 1 second has passed.
* Take at least 30 captures for which a count is seen in the terminal.
* The captures should be as varied as possible, moved in the x, y, and z. Rotation is also needed but must be less than 90 degrees.

To exit:
```
ctrl-c
```

NOTE: This needs a folder 'data' within the acrv_apc_2017/acrv_realsense_tools, which contains folders 'color' and 'ir'.

### stereo_calibrate.py
From acrv_apc_2017/acrv_realsense_tools:

```
cd scripts/
python stereo_calibrate.py
```
This will produce the calibration matrices of the camera, from the checkerboard captures taken from capture_checkerboard_images.

* NOTE: This needs a folder 'data' within the acrv_apc_2017/acrv_realsense_tools, which contains a folder 'color' within the rgb captures of the checkerboard, and a folder 'ir' with the IR captures of the checkerboard. As well as a folder 'output' with folders 'color' and 'ir'.
