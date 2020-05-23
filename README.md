# rviz_spacenav
Move around in Rviz using [Spacenav](http://spacenav.sourceforge.net/)

## How To Build

```bash
cd /path/to/catkin/workspace/src
wget https://raw.githubusercontent.com/rkoyama1623/rviz_spacenav/master/rosinstall -O /tmp/rosinstall
wstool init && wstool merge /tmp/rosinstall
wstool update
rosdep install --from-paths . --ignore-src -r -y
catkin build
```

## How to Use
```bash
roslaunch rviz_spacenav rviz_spacenav.launch rviz:=true
```
