# outreach24
## Initial stuff
1. Clone this repo to remote PC. Make sure to run rosdep or make sure `cv_bridge` is installed.
2. Assign permissions to `capture.sh` in the repo's folder with 
```bash
cd outreach24
chmod +x capture.sh
``` 
3. Build
```bash
# make sure you are in the `outreach24` folder.
colcon build --symlink-install
```

## To capture images
1. On robot, run
```bash
ROS_DOMAIN_ID=8 ros2 run camera_ros camera_node --ros-args -p width:=100 -p height:=100 -p format:=RGB888
```
2. On remote PC containing this repo, run the `.sh` with
```bash
./capture.sh
```
3. To capture an image, simply type `c` into the terminal. The images will be stored in the `img` folder.
4. To adjust the prefix label of the images (default is `stop`), extension (default `.jpg`), and other options, check out `src/capture/params/capture.yaml`
5. To preview the images, on another terminal on the remote PC:
```bash
ROS_DOMAIN_ID=8 ros2 run image_tools showimage --remap /image:=/camera/image_raw
```
Note that the blue and red channels in the preview are swapped because of the format "BGR888" in the first instruction. The showimage window also shows the 100% size of the image, so small images will appear quite small, and large images may exceed the size of the computer monitor.

## Marking Images
1. Build and Install DarkNet, followed by DarkHelp, then DarkMark
2. Upload image file with the 'Add...' button
3. Click on 'Class IDs' to modify the image classes
4. Click on 'Load' to start marking images

## Formatting data
1. Add the images into a folder
2. Add the image labels (not the .json files) into a separate folder
3. Put both folders into 1 folder, named 'datasets'

## Training model

1. Run the following:
```
python3 run traffic_train.py
```
2. After training, move the 'best.pt' folder from home/yolov4/runs/detect/train/weights into the outreach24 folder

## Testing

1. Build the outreach24 folder
2. For running terminal, run:

```
cd outreach24
source install/setup.bash
```
3. Run:
```
ROS_DOMAIN_ID=8 ros2 run traffic traffic
```

## Debug
```bash
ros2 run image_tools cam2image --ros-args -p burger_mode:=true --remap image:=camera/image_raw
python3 -m pip uninstall setuptools
python3 -m pip install setuptools
```

