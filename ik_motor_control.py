"""Importing the required Modules """
import numpy as np 
import math 
import dynamixel_sdk as ds
import math


def set_home_configuration(motor_details,port,packets,ADDR_TORQUE_ENABLE,TORQUE_ENABLE,ADDR_GOAL_POSITION):
    """This function sets the home configuration of the platform.
    This function is called in the beginning of tracking any trajectory.
    This fucntion also sets all the motors in torque mode before achieving the home configuration"""
    output = [] # List to store the results of operations while trying to achive home configuration and enabling torques
    for motor in motor_details:
        ID = motor_details[motor]
        dxl_comm_result, dxl_error = packets.write1ByteTxRx(port, ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        dxl_comm_result, dxl_error = packets.write4ByteTxRx(port, ID, ADDR_GOAL_POSITION, 0)
        output.append(dxl_comm_result)
    print("Home Configuration is acheived")
    return output 

def rotz(alpha):
    """Rotation matrix in Z direction"""
    return np.round(np.asarray([[np.cos(alpha),-np.sin(alpha),0],[np.sin(alpha),np.cos(alpha),0],[0,0,1]]),4)

def roty(beta):
    """Rotation matrix in Y Direction"""
    return np.round(np.asarray([[np.cos(beta),0,np.sin(beta)],[0,1,0],[-np.sin(beta),0,np.cos(beta)]]),4)

def rotx(gamma):
    """Rotation matrix in X direction """
    return np.round(np.asarray([[1,0,0],[0,np.cos(gamma),-np.sin(gamma)],[0,np.sin(gamma),np.cos(gamma)]]),4)

def rotation(alpha,beta,gamma):
    """Overall rotation matrix"""
    return np.round(np.linalg.multi_dot([rotz(alpha),roty(beta),rotx(gamma)]),2)

def InverseKinematics(Orientation,P,PF,Base):
    """This function computes the inverse kinematics of a given Orientation and Translation"""
    gamma,beta,alpha = Orientation # Orientation of the trajectory
    Px,Py,Pz         = P           # Translarion of the trajectory 

    P_whole = np.transpose(np.asarray([P for i in range(6)]))  # We need to compute inverse kinematics for 6 legs . Hence forming a list of size 6 with required center point translation

    Q = P_whole+np.dot(rotation(alpha,beta,gamma),PF)
    L = Q-Base

    x= L[0]
    y = L[1]
    z= L[2]
    lenghts=[]
    # print(x[0],y[0],z[0])
    """Computing lengths required for each prismatic joint. 
    We have used motors so we need to convert these lengths to angles 
    which will be done in the later parts of the program"""
    for i in range(0,len(x)):
        lenghts.append(math.sqrt(x[i]**2+y[i]**2+z[i]**2))
    
    Xpi = Q[0]
    Ypi = Q[1]
    Zpi = Q[2]
    A = [i**2-(d**2-r**2) for i in lenghts]
    B = [2*r*i for i in Zpi]
    Base_X = Base[0]
    Base_Y = Base[1]
    
    C  = [2*r*(np.cos(sigmas[i])*(Xpi[i]-Base_X[i]) +np.sin(sigmas[i])*(Ypi[i]-Base_Y[i])) for i in range(6)]
    """Angles required"""
    Angles = [np.arcsin(A[i]/math.sqrt(B[i]**2+C[i]**2))-np.arctan2(C[i],B[i]) for i in range(0,6)]
    """Converting Angles to steps. Because motors require step inputs"""
    steps = [i*2048/np.pi for i in Angles]
    for i in range(0,len(steps)):
        steps[i] = pow(-1,i+1)*steps[i]
    return L, lenghts,Angles ,steps 

def trajectories():
    """Comment out the portion that is not required"""
    x = np.arange(0,100*np.pi,0.5)
    # z = [76 for i in x]
    # x= 1
    """sine wave in all directions.
    20 Magnitude in X direction. 
    20 Magnitide in Y direction 
    5 Magnitude in Z direction . 76 is added because it is height in  Z at home configuration"""
    Px, Py,Pz = [20*np.sin(x),20*np.cos(x),76+5*np.sin(x)]
    gamma,beta,alpha = [0*np.sin(x),0*np.sin(x),0*np.sin(x)] # All the orinetations are set to zero

    """Sinusoids in XY Direction """
    Px, Py,Pz = [20*np.sin(x),20*np.cos(x),76+0*np.sin(x)]
    gamma,beta,alpha = [0*np.sin(x),0*np.sin(x),0*np.sin(x)] # All the orinetations are set to zero

    """Sinusoids in YZ Directions"""
    Px, Py,Pz = [0*np.sin(x),20*np.cos(x),76+5*np.sin(x)]
    gamma,beta,alpha = [0*np.sin(x),0*np.sin(x),0*np.sin(x)] # All the orinetations are set to zero
    """Sinusoids in XZ Directions"""
    # Px, Py,Pz = [20*np.sin(x),0*np.cos(x),76+5*np.sin(x)]
    # gamma,beta,alpha = [0*np.sin(x),0*np.sin(x),0*np.sin(x)] # All the orinetations are set to zero
    """Breathing Motion in Z direction """
    # e = math.e
    # Px,Py,Pz = [0*(np.exp(np.sin(x))-1/e)*(2*10/(e-(1/e))),0*(np.exp(np.sin(x))-1/e)*(2*5/(e-(1/e))),76+(np.exp(np.sin(x))-1/e)*(2*5/(e-(1/e)))]
    # gamma,beta,alpha = [0*np.sin(x),0*np.sin(x),0*np.sin(x)] # All the orinetations are set to zero

    """Heart Beat  in XZ direction"""
    T = -2.5
    b= 1
    x = np.arange(70,85,4)
    #Increasing the size of X 
    x= np.append(x,x[::-1])
    x= np.append(x,x[::-1])
    x= np.append(x,x[::-1])
    x= np.append(x,x[::-1])
    x= np.append(x,x[::-1])
    x= np.append(x,x[::-1])
    x= np.append(x,x[::-1])
    x= np.append(x,x[::-1])
    x= np.append(x,x[::-1])
    xd = x/40
    Px,Py,Pz = [3*(xd**3+T*xd+b), 0*x,x]
    gamma,beta,alpha = [0*x,0*x,0*x]

    """Parabolic path"""
    # Px= np.arange(-10,10,0.5)
    # Py=[0 for x in Px]
    # a = 10#4.4814
    # Pz= [-(76+((x**2)/(4*a))) for x in Px]
    # gamma,beta,alpha = [0*np.sin(Px),0*np.pi/30*np.sin(Px),0*np.sin(Px)]

    return Px,Py,Pz,gamma,beta,alpha

def control():
    """This function defines the Addresses required for each control action 
    and sets of control environment"""
    ADDR_TORQUE_ENABLE          = 64
    ADDR_GOAL_POSITION          = 116
    ADDR_PRESENT_POSITION       = 132
    ADDR_PRESENT_VELOCITY       = 128
    ADDR_POSITION_TRAJECTORY    = 140
    DXL_MINIMUM_POSITION_VALUE  = 0         # Refer to the Minimum Position Limit of product eManual
    DXL_MAXIMUM_POSITION_VALUE  = 4095      # Refer to the Maximum Position Limit of product eManual
    BAUDRATE                    = 57600
    ADDR_DRIVEMODE              = 10

    TORQUE_ENABLE               = 1     # Value for enabling the torque
    TORQUE_DISABLE              = 0     # Value for disabling the torque
    DXL_MOVING_STATUS_THRESHOLD = 20    # Dynamixel moving status threshold


    USB_Port ="/dev/tty.usbserial-FT62AGGK"  # The serial port name 
    protocol_version = 2 # Using Dynamixel 2.0 protocol 
    port = ds.PortHandler(USB_Port)
    packets = ds.PacketHandler(protocol_version)

    if port.openPort():
        print("Succesful opening the port")
    else:
        print("Failed to connect ")
        input("Press Enter key  to terminate")
        quit()
    if port.setBaudRate(BAUDRATE):
        print("Succesful Setting Baudrate")

    motor_details = {"M1":1,"M2":2,"M3":3,"M4":4,"M5":5,"M6":6} # Setting up motor details 
    set_home_configuration(motor_details,port,packets,ADDR_TORQUE_ENABLE,TORQUE_ENABLE,ADDR_GOAL_POSITION) # Setting up home configuration 

    return port,packets,ADDR_GOAL_POSITION,motor_details

if __name__ == "__main__":
    port,packets,ADDR_GOAL_POSITION,motor_details = control() # Control environment setup 
    ADDR_TORQUE_ENABLE          = 64
    ADDR_GOAL_POSITION          = 116
    ADDR_PRESENT_POSITION       = 132
    ADDR_PRESENT_VELOCITY       = 128
    ADDR_POSITION_TRAJECTORY    = 140
    TORQUE_ENABLE               = 1     # Value for enabling the torque
    TORQUE_DISABLE              = 0     # Value for disabling the torque
    DXL_MOVING_STATUS_THRESHOLD = 20    # Dynamixel moving status threshold

    """Coordinates of anchor points on the base obtained from CAD design"""
    Base = np.transpose(np.asarray([[76.83,8.91,0],
                                    [46.13,62.08,0],
                                    [-46.13,62.08,0],
                                    [-76.83,8.91,0],
                                    [-30.70,-70.99,0],
                                    [30.70,-70.99,0]]))
    """Coordinates of anchor points on the platform obtained from CAD design"""
    PF = np.transpose(np.asarray([[74.96,30.08,0],
                                    [63.53,49.88,0],
                                    [-63.53,49.88,0],
                                    [-74.96,30.08,0],
                                    [-11.43,-79.96,0],
                                    [11.43,-79.96,0]]))
    """Orientation of crank with respect to axis seen at home configuration """
    sigmas = [-np.pi/3,2*np.pi/3,np.pi/3,4*np.pi/3,np.pi,0]
    r = 12.7   # Crank Length
    d = 83   # Connecting Rod measurement on actual platform 
    height=76  # Height of the platform at home configuration 

    Px,Py,Pz,gamma,beta,alpha = trajectories() # Generating the trajectories 
    all_steps=[]
    for i in range(0,len(Px)):
        P = [Px[i],Py[i],Pz[i]]
        Orientation = [gamma[i],beta[i],alpha[i]]
        L, lenghts,Angles ,steps  = InverseKinematics(Orientation,P,PF,Base)
        all_steps.append(steps)
    print("Inverse Kinematics is calculated")
    
    """Feeding the steps to servos """
    for steps in all_steps:    
        # print(steps)
        output = [packets.write4ByteTxRx(port, ID, ADDR_GOAL_POSITION, int(steps[ID-1])) for ID in [1,2,3,4,5,6]]
