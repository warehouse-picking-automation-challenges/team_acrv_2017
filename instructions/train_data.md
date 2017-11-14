## On apc-cameras: Get set up.
* Start the robot, `roslaunch state_machine launch_robot.launch`
* Put stow tote in position.
* Move the robot to realsense_above_stow_tote
* Start the vision `roslaunch state_machine launch_vision_datacap.launch`
* Make sure acrv_item_data.json exists from acrv_item_data_base.json and same with measured_weights.json
* Watch the video stream in rviz. (from the wrist realsense, just to be able to see it.)

## On apc-puppy: Get set up
* Make sure finetune_tote.sh is correct. Make sure the initial model is correct.
* Verify that train_images and train_labels folders exist within  cloudstor/acrv_apc_2017_data/results_refinenet/20170701_tote_bin/ and that they have 92 images each.
* Remove f1shots folder if it exists within ../results_refinenet/20170701_tote_bin/ 
* Copy cloudstor/acrv_apc_2017_data/results_refinenet/20170701_tote_bin/train.txt and rename to f1shots_list.txt
* Run `export ROS_MASTER_URI=http://apc-cameras:11311`
* `roslaunch acrv_apc_2017_perception run_binary_refinenet_service.launch`


## On apc-cameras: When they give us the data.
* Remove everything from `state_machine/data/item_data`
* Reset acrv_item_meta.json by duplicating acrv_item_data_base.json
* Remove everything except {} out of measured_weights.json
* Copy the item information into `state_machine/data/item_data/`

## On apc-cameras: Record data
* `roscd state_machine/data`
* `python image_capture.py`
* Follow the prompts.  For each two items, have someone put them in the tote and rearrange.

## On apc-puppy: Check automatic segmentations
* It will pop up the images.
* y/n to confirm, or draw new segments.
* Do not close the image_capture.py until all images are processed.

## On apc-puppy: Start training
* KILL THE BINARY REFINENET SERVICE - IT IS USING MEMORY ON THE GPUS.
* `roscd acrv_apc_2017_perception/src/refinenet`
* `./finetune_tote.sh`
* Make sure it starts correctly. Accuracy should be high, loss should be a real number.
* Update the item list:
  * in folder `acrv_apc_2017_perception/src/refinenet/snapshots`
  * rename file `ITEMS_LIST_NEW-items.json` to the <model_name>-items.json
  * E.g. if the model output is 170717_1713_tote-200.params, then call it 170717_1713_tote-items.json

## On apc-cameras: Measure the item weights
* Start the stow tote scales `roslaunch scales_interface stow_tote_scales.launch`
* Monitor the weights topic so that you can see it is stable when hitting enter. `rostopic echo /stow_tote_scales/weight`
* `python measure_weights.py`
* Follow the prompts.

## On apc-cameras: Set up the item meta information.
* `roscd state_machine/scripts/states/helpers/`
* `python item_meta.py` will add the new items to the file.
* Update the item meta file (`acrv_item_data.json`) with all of the new items.
* `python item_meta.py` will tell you if you stuffed. up.


## On apc-camera: When the training is done.
* Opening run_refinenet_service_remote.launch
* Update model name and epoch.
* Run `roslaunch state_machine launch_vision.launch` and make sure it works.
