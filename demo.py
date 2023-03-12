from numpy import True_
from st import *
import time

sleep_period = 0
# initialise arm, this should connect and run START, ROBOFORTH, CALIBRATE, HOME, JOINT
arm=StArm()

def main():
    

    while True:
        arm.joint_move([1800,2800,8000,1334,1500]) #1
        time.sleep(sleep_period)
        arm.joint_move([1800,6000,7000,1334,1500]) #2
        time.sleep(sleep_period)
        arm.joint_move([1800,8000,6000,1334,1500]) #3
        time.sleep(sleep_period)
        arm.joint_move([1800,8000,5000,1334,1500]) #4
        time.sleep(sleep_period)
        arm.joint_move([2000,6000,5000,1334,1500]) #5
        time.sleep(sleep_period)
        arm.joint_move([2000,4000,8000,1334,4000]) #6
        time.sleep(sleep_period) 
        arm.joint_move([2000,6000,8000,334,3000]) #7
        time.sleep(sleep_period)
        arm.joint_move([-2000,6000,8000,334,7000]) #8
        time.sleep(sleep_period)
        arm.joint_move([-2500,7000,7000,334,9000]) #9
        time.sleep(sleep_period)
        arm.joint_move([-3000,8000,5000,734,10500]) #10
        time.sleep(sleep_period)
        arm.joint_move([-2500,8000,4000,1434,11500]) #11
        time.sleep(sleep_period)
        arm.joint_move([-2500,6000,5000,0,11500]) #12
        time.sleep(sleep_period)
        arm.joint_move([0,6000,5000,0,12500]) #13
        time.sleep(sleep_period)
        arm.joint_move([2000,6000,5000,0,-3000]) #14
        time.sleep(sleep_period)

        arm.joint_move([1800,2800,8000,1334,1500]) #1
        time.sleep(sleep_period)

        arm.home()
        arm.calibrate()

        if input("press y to continue") == "y":
            continue
        else:
            arm.home()
            arm.close_connection()
            break
            



        

if __name__ == "__main__":
    main()
