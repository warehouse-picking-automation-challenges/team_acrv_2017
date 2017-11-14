# README
This directory contains the setup scripts for the ACRV APC 2017 repository.

It also contains the setup scripts for the Buildbot for the same.

To set up:

`chmod u+x ./qut-apc-base-setup-script.sh`

`./qut-apc-base-setup-script.sh`


Chances are not everything will install properly the first time through (especially the realsense libraries for some reason).  You may need to go through and run some of the commands individually.  

Have a look at `RunEverything.md` for run instructions.

## Changes

The following extra requirements have been added to the setup script.  If your installation is giving you errors, you may need to update it by manually doing the below:

*2/5/17 - Added segmentation library from 2016*

If you are getting an error saying that library segmentation is not installed, run the steps under 'Install Segmentation Library' in the setup script to install it.

*29/5/17 - Setting up SSH Keys*
To have known_hosts in the correct format for ros to read, first connection to a computer with ssh -oHostKeyAlgorithms='ssh-rsa' <host>.  ROS can't read the ECDSA format that SSH uses by default and will tell you the computer isn't in the known hosts file.
