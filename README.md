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
ROS_DOMAIN_ID=8 ros2 run camera_ros camera_node --ros-args -p width:=160 -p height:=120 -p format:=BGR888
```
2. On remote PC containing this repo, run the `.sh` with
```bash
./capture.sh
```
3. To capture an image, simply type `c` into the terminal. The images will be stored in the `img` folder.
4. To adjust the prefix label of the images (default is `stop`), extension (default `.jpg`), and other options, check out `src/capture/params/capture.yaml`
5. To preview the images, on another terminal on the remote PC:
```bash
ROS_DOMAIN=8 ros2 run image_tools showimage --remap /image:=/camera/image_raw
```
Note that the blue and red channels in the preview are swapped because of the format "BGR888" in the first instruction. The showimage window also shows the 100% size of the image, so small images will appear quite small, and large images may exceed the size of the computer monitor.

## Debug
```bash
ros2 run image_tools cam2image --ros-args -p burger_mode:=true --remap image:=camera/image_raw
python3 -m pip uninstall setuptools
python3 -m pip install setuptools
```

