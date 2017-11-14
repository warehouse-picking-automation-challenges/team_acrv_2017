import numpy as np
import glob
import os
import sys
import operator
import itertools
import time
import pprint

from shutil import copyfile

import rospy

from std_srvs.srv import Trigger


### USER INPUTS ###

dataset_full_path_string = '/home/adamwtow/cloudstor/Shared/ARChallenge2017/acrv_apc_2017_data/data'
save_dataset_full_path_string = '/home/adamwtow/cloudstor/Shared/ARChallenge2017/acrv_apc_2017_data/data_regenerated_v3'

### USER INPUTS ###


generated_files_list = [
    'cloud_hd.pcd',
    'cloud.pcd',
    'color_hd_raw.png',
    'color_hd_rect.png',
    'color_raw.png',
    'color_rect.png',
    'depth_aligned.png',
    'depth_hd_aligned.png',
    'depth_raw.png',
    'depth_rect.png',
    'ir_raw.png',
    'ir_rect.png'
]

other_files_list = [
    'annotations.xml',
    'map.png',
    'map.txt',
    'overlay.png'
]


def main():
    print('Waiting for service: /data_regeneration_server_node/publish_raw_data_once')
    rospy.wait_for_service('/data_regeneration_server_node/publish_raw_data_once')
    trigger_data_regeneration_service = rospy.ServiceProxy('/data_regeneration_server_node/publish_raw_data_once', Trigger)

    # Get a list of folders that need to be processed and a list of folders with missing files
    folders_to_check = False
    root_dirs_list = []
    root_dirs_to_check_list = []
    for root, dirs, files in os.walk(dataset_full_path_string):
        if ('color.png' in files) and ('depth.png' in files) and ('ir.png' in files):
            root_dirs_list.append(root)
        else:
            if ('color.png' in files) or ('depth.png' in files) or ('ir.png' in files):
                root_dirs_to_check_list.append(root)
                folders_to_check = True

    # Check which root dirs have already been generated and remove from list:
    for root, dirs, files in os.walk(save_dataset_full_path_string):
        remove_root_dir = True
        for gen_file_name in generated_files_list:
            if gen_file_name not in files:
                remove_root_dir = False
        if remove_root_dir:
            root_to_remove = root.replace(save_dataset_full_path_string, dataset_full_path_string)
            print('Removing from list as already generated: ', root_to_remove)
            root_dirs_list.remove(root_to_remove)

    print('\nList of dirs to process:')
    pprint.pprint(root_dirs_list)

    if folders_to_check:
        print('\nError! The following folders appear to be incomplete:')
        pprint.pprint(root_dirs_to_check_list)
        # print('Exiting now...')
        # return

    failed_read_dirs_list = []
    failed_save_dirs_list = []

    for root_dir in root_dirs_list:
        # data_regeneration_server_node assumes paths do have a final slash
        read_folder_path = root_dir.rstrip('/') + '/'
        save_folder_path = read_folder_path.replace(dataset_full_path_string, save_dataset_full_path_string)
        rospy.set_param('/data_regeneration_server_node/read_folder_path', read_folder_path)
        rospy.set_param('/data_regeneration_server_node/save_folder_path', save_folder_path)
        response = trigger_data_regeneration_service()
        if not response.success:
            failed_read_dirs_list.append(read_folder_path)
            failed_save_dirs_list.append(save_folder_path)
        time.sleep(5)

    # NOTE The below will still crash if the final folder doesn't exist
    for root, dirs, files in os.walk(dataset_full_path_string):
        if root in root_dirs_to_check_list:
            continue
        if ('color.png' in files) and ('depth.png' in files) and ('ir.png' in files):  # Prevents looping into other dirs
            for file_to_copy in other_files_list:
                if file_to_copy in files:
                    if file_to_copy not in glob.glob(os.path.join(root.replace(dataset_full_path_string, save_dataset_full_path_string), file_to_copy)):
                        # print('I want to copy', file_to_copy)
                        copyfile(os.path.join(root, file_to_copy), os.path.join(root.replace(dataset_full_path_string, save_dataset_full_path_string), file_to_copy))
            for image_path in glob.glob(os.path.join(root, 'mask_*.png')):
                fn = os.path.basename(image_path)
                if fn not in glob.glob(os.path.join(save_dataset_full_path_string, fn)):
                    # print('I want to copy', fn)
                    copyfile(os.path.join(root, fn), os.path.join(root.replace(dataset_full_path_string, save_dataset_full_path_string), fn))

    if len(failed_read_dirs_list) > 0:
        print('\nThe following read dirs failed to be processed:')
        pprint.pprint(failed_read_dirs_list)
        print('\nTheir saved dir counterparts are here:')
        pprint.pprint(failed_save_dirs_list)


if __name__ == '__main__':
    main()
