**Launch file to display laser scanner data in rviz**

If laser scanner is connected to drone
```bash
roslaunch ursa laser_scanner_to_cartographer_and_rviz.launch
```
If laser scanner is connected to desktop
```bash
roslaunch ursa laser_scanner_to_cartographer_and_rviz.launch desktop:=1
```
