******** SETUP ********

1. Make sure computer sound is on.

2. Run this command in each new terminal
> source make_and_source.sh 

3. Check ip address and change in tablet if needed



******** RUN ROBOTS & STORY CONTROLLER ********

1. Launch roscore
> roscore

2. Launch robots (if they are on ACM0 and ACM1):
> roslaunch eikeepon_pkg keeponstage.launch

3. Story controller (make sure you're in the right folder):
> rosrun eikeepon_pkg StoryController.py 

4. Open tablet app, insert ip address and touch start



******** DATA COLLECTION ONLY ******** 

1. LAUNCH OPENNI 

> roslaunch openni_launch openni.launch depth_registration:=true

2. VIEW DEPTH
> rosrun image_view disparity_view image:=/camera/depth_registered/disparity 

3. Record rosbags with logs
GROUPS:       
> ./logs_record.sh 
    
INDIVIDUALS    
> ./kinect_logs_record.sh
