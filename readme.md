### Environment setup 
#### Install Python
Either Python2 or Python3 can be used. But make sure to install Dynamixel SDK for your python environment. This SDK can be downloaded from https://github.com/ROBOTIS-GIT/DynamixelSDK . 

If you are planning to use a linux or Mac computer , then you may not need to install any serial drivers. But some windows computers may need serial drivers.You can use the PL2303 serial driver for windows. This driver is available at  http://www.prolific.com.tw/US/ShowProduct.aspx?p_id=225&pcid=41. 

### Scripts

The script ik_motor_control.py should be used after connecting the platform to the computer. In this script , function named trajectories to be edited to track described trajectories. By default, there are multiple trajectories written already. Make sure to comment out the trajectories that are not required. If you are planning to add more trajectories, make sure this function returns the X,Y,Z and Alpha, Beta and Gamma of the trajectories in vector format. 

<img src = 'https://github.com/shivakumar-tekumatla/Stewart-Platform-Inverse-Kinematics/blob/main/video%20gifs/XYZ%20motion.gif' >
