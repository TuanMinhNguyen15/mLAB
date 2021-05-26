# Name: Tuan Minh Nguyen
# Institution: University of Pennsylvania
# Lab: mLAB


### Run the controller code
Access the bash of the running container using `docker exec -it graic_con /bin/bash` in a new terminal.

You can run the controller using the following commands.
```
cd graic-workspace
```
```
catkin_make
```
```
. devel/setup.bash
```
```
pip3 install mip
```
```
roslaunch race carla_single.launch num_wheels:=4 model_type:=model_free
```

A vehicle should appear in the CARLA window.

Now open a new terminal and use `docker exec -it graic_con /bin/bash` and run the following in the CARLA container.
```
cd graic-workspace/
```
```
. devel/setup.bash
```
```
cd src/race/src/
```
```
python3 HMPC_Tuan.py 
```
The vehicle should start moving 



