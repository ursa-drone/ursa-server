**Ensure ROS_MASTER_URI on local and remote is the same**

add to ~/.bashrc  
```bash
export ROS_MASTER_URI="http://10.42.0.1:11311"
```
	
**Setup SSH key pair**  

```bash
ssh-keygen
```      
```bash
ping -c 3 pi@10.42.0.2
```  
```bash
ssh-copy-id pi@10.42.0.2
```  
need to do this step because of ROS bug/feature
```bash
ssh -oHostKeyAlgorithms='ssh-rsa' pi@10.42.0.2 
```
```bash
ssh-add
```  
	
**Remove SSH key pair** (if you accidentally forget to connect to remote first time without -oHostKeyAlgorithms='ssh-rsa')  

on remote
```bash
ssh-keygen -R pi@10.42.0.2
``` 

**Check permissions for laser scanner & make permissions permanent**
```bash
sudo touch /etc/udev/rules.d/50-myusb.rules
```
```bash
SUBSYSTEMS=="usb", KERNEL=="ttyACM[0-9]*", ACTION=="add", ATTRS{idVendor}=="15d1", ATTRS{idProduct}=="0000", MODE="666", PROGRAM="/opt/ros/kinetic/lib/urg_node/getID /dev/%k q", SYMLINK+="sensors/hokuyo_%c", GROUP="dialout"
```
```bash
sudo udevadm control --reload
```