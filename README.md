# dvrk_recording

For recording video and kinematics on the DVRK system. 

Usage: roslaunch record_op.launch

This will pop-up a GUI for recording / stop recording purposes. Everytime you hit record, it will create a new folder with current date/time, and it will save left/right images with kinematics.

If not working, make sure the rostopic names are correct in the python script.
