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
4. To adjust the prefix label of the images (default is `stop`), extension (default `.jpg`), and other options, check out `src/capture/params/capture.yaml`.