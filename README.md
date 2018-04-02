Assuming the user has opencv and ros installed on their system.  

Task 1  
To compile the file, go to AUV_TaskRound/Task1  
Edit line 20 of a.cpp to point to video file  
Now Run the following on a terminal from folder Task1  
..$make a  
..$./a  
    
Task 2  
To compile the file, go to AUV_TaskRound/Task2  
Edit line 77 of track2.cpp to point to video file (best output on "frontcam.avi")  
Now run the following on a terminal from folder Task2  
   $make track2  
   $./track2  
Press any key to move from one frame to next  
    
Task 3  
ROS package Structure:  
catkin_ws  
..-build                                          //build files  
..-devel  
..-src                                            //This folder contains the actual source code  
....-auv_task                                   //Code for the tasks  
......-include  
......-src                                    //Source code  
........-kalman.cpp                         //Kalman Filter Code  
              -listener.cpp                       //Node to publish measurements  
              -talker.cpp                         //Node to publish video feed  
              -trackerdisp.cpp                    //Node to display video feed with predictions  
          -CMakeLists.txt  
          -package.xml  
  -beginner_tutorials //not relevant to this submission  

Edit line 33 of talker.cpp to point to video file (best output on "frontcam.avi")  
To compile the file, go to AUV_TaskRound/catkin_ws.  
This folder is a catkin workspace that contains package auv_task,which is to be run.  
Copy-paste the AUV_TaskRound/catkin_ws/auv_task folder into a workspace on your machine.  
Then run the following on a terminal from your workspace-  
  $catkin_make  
  $roscore  
    
In another terminal run  
  $source ./devel/setup.bash  
  $rosrun auv_task talker_image  
  
In another terminal run  
  $source ./devel/setup.bash  
  $rosrun auv_task listener_image  

In another terminal run  
  $source ./devel/setup.bash  
  $rosrun auv_task kalman  
    
In another terminal run  
  $source ./devel/setup.bash  
  $rosrun auv_task trackerdisp  
  
All 5 terminals need to be running simultaneously for the program to function  

