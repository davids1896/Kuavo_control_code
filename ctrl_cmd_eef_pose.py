import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from ocs2_msgs.msg import mpc_target_trajectories
from std_msgs.msg import Int32
from kuavo_msgs.srv import changeArmCtrlMode
from motion_capture_ik.msg import twoArmHandPoseCmd, ikSolveParam
import numpy as np
import argparse
import sys
import select
import termios
import tty

# decide use custom ik param or not
use_custom_ik_param = True
# joint angles as initial guess for ik
joint_angles_as_q0 = False # True for custom initial guess, False for default initial guess
# ik solver param
ik_solve_param = ikSolveParam()
# snopt params
ik_solve_param.major_optimality_tol = 1e-3
ik_solve_param.major_feasibility_tol = 1e-3
ik_solve_param.minor_feasibility_tol = 1e-3
ik_solve_param.major_iterations_limit = 100
# constraint and cost params
ik_solve_param.oritation_constraint_tol= 1e-3
ik_solve_param.pos_constraint_tol = 1e-3 # work when pos_cost_weight==0.0
ik_solve_param.pos_cost_weight = 0.0 # If U need high accuracy, set this to 0.0 !!!

class RobotControl:
    def __init__(self, arm_trajectory, forward_speed, strafe_speed, rotate_speed, height_speed, stop_step_num, speed_increment, delta_joint, eef_speed):
        self.arm_trajectory = arm_trajectory
        self.current_index = len(arm_trajectory) - 1
        
        self.robot_height = 0.00
        self.forward_speed = forward_speed
        self.strafe_speed = strafe_speed
        self.rotate_speed = rotate_speed
        self.height_speed = height_speed
        self.stop_step_num = stop_step_num
        self.speed_increment = speed_increment
        self.delta_joint = delta_joint
        self.eef_speed=eef_speed

        #self.arm_mode_change(2)

        rospy.init_node("robot_ctrl_cmd_ext", anonymous=True)

        self.joint_pub = rospy.Publisher("/kuavo_arm_traj", JointState, queue_size=10)
        self.vel_pub = rospy.Publisher("/cmd_pose", Twist, queue_size=10)
        self.step_pub = rospy.Publisher("/humanoid_mpc_stop_step_num", Int32, queue_size=10)
        self.eef_pub=rospy.Publisher('/ik/two_arm_hand_pose_cmd', twoArmHandPoseCmd, queue_size=10)

        rospy.Subscriber("/kuavo_arm_traj", JointState, self.arm_traj_callback)
        rospy.Subscriber("/cmd_pose", Twist, self.cmd_vel_callback)
        rospy.Subscriber("/humanoid_mpc_stop_step_num", Int32, self.stop_step_callback)
        rospy.Subscriber("/ik/two_arm_hand_pose_cmd", twoArmHandPoseCmd, self.eef_callback)
        # rospy.Subscriber("/humanoid_mpc_target_pose", mpc_target_trajectories, self.mpc_target_callback)

        # Initialize arm trajectory values if not provided
        self.left_arm_traj = [0, 0, 0, 0, 0, 0, 0]
        self.right_arm_traj = [0, 0, 0, 0, 0, 0, 0]
        self.left_eef_position_xyz= [0.35, 0.26, 0.2]
        self.right_eef_position_xyz= [0.35, -0.26, 0.2]
        self.left_eef_quat_xyzw=[0.0, -0.706825181105366, 0.0, 0.7073882691671997]
        self.right_eef_quat_xyzw=[0.0, -0.706825181105366, 0.0, 0.7073882691671997]


    def get_key(self):
        old_attrs = termios.tcgetattr(sys.stdin)
        try:
            tty.setraw(sys.stdin.fileno())
            select.select([sys.stdin], [], [], 0)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attrs)
        return key

    def arm_mode_change(self, mode: int):
        service_name = "/humanoid_change_arm_ctrl_mode"
        try:
            rospy.wait_for_service(service_name)
            change_hand_tracking_mode_srv = rospy.ServiceProxy(service_name, changeArmCtrlMode)
            change_hand_tracking_mode_srv(mode)
            self.arm_mode = mode
        except rospy.ROSException:
            rospy.logerr(f"Service {service_name} not available")

    def arm_pose_publish(self, left_arm_traj, right_arm_traj):
        msg = JointState()
        msg.name = ["arm_joint_" + str(i) for i in range(1, 15)]
        msg.header.stamp = rospy.Time.now()
        msg.position = np.array(left_arm_traj + right_arm_traj)
        self.joint_pub.publish(msg)
        print("======= Published to /kuavo_arm_traj")
    
    def eef_pose_publish(self,left_eef_position, right_eef_position, left_quat_xyzw, right_quat_xyzw):
        #TODO
        msg = twoArmHandPoseCmd()
        msg.ik_param=ik_solve_param
        msg.use_custom_ik_param = use_custom_ik_param
        msg.joint_angles_as_q0 = joint_angles_as_q0

        msg.hand_poses.left_pose.joint_angles = np.zeros(7) # rads
        msg.hand_poses.right_pose.joint_angles = np.zeros(7)

        msg.hand_poses.left_pose.pos_xyz = left_eef_position
        msg.hand_poses.left_pose.quat_xyzw = left_quat_xyzw
        msg.hand_poses.left_pose.elbow_pos_xyz = np.zeros(3)

        msg.hand_poses.right_pose.pos_xyz = right_eef_position
        msg.hand_poses.right_pose.pos_xyz[1] = -right_eef_position[1]
        msg.hand_poses.right_pose.quat_xyzw = right_quat_xyzw
        msg.hand_poses.right_pose.elbow_pos_xyz = np.zeros(3)
        self.eef_pub.publish(msg)
        print("======= Published to /ik/two_arm_hand_pose_cmd")

    def publish_robot_vel(self, velocities):
        robot_vel = Twist()
        robot_vel.linear.x = velocities[0]
        robot_vel.linear.y = velocities[1]
        robot_vel.linear.z = velocities[3]
        robot_vel.angular.z = velocities[2]
        self.vel_pub.publish(robot_vel)
        print("======= Published to /cmd_pose")

    def publish_robot_stp(self, step_num):
        robot_stp = Int32(data=step_num)
        self.step_pub.publish(robot_stp)
        print("======= Published to /humanoid_mpc_stop_step_num")

    def arm_traj_callback(self, msg):
        print(f"======= Received from /kuavo_arm_traj:\n{msg}")

    def cmd_vel_callback(self, msg):
        print(f"======= Received from /cmd_pose\n{msg}")

    def stop_step_callback(self, msg):
        print(f"======= Received from /humanoid_mpc_stop_step_num\n{msg}")

    def mpc_target_callback(self, msg):
        print(f"======= Received from /humanoid_mpc_target_pose\n{msg}")

    def eef_callback(self, msg):
        print(f"======= Received from /ik/two_arm_hand_pose_cmd\n{msg}")


    def run(self):
        print("Control the robot using the keyboard:")
       # print("1-5: Adjust left arm joints 1-5 positively")
        print("6is deleted 6-0: Adjust left arm joints 1-5 negatively")
        print("q/e: Previous/Next arm position")
        print("w/s: Forward/Backward")
        print("a/d: Rotate Left/Right")
        print("z/c: Strafe Left/Right")
        print("v/n: Up/Down")
        print("b: Set the Number of Stopping Step")
        print("x: Stop")
        print("i/j: Increase/Decrease Forward Distance")
        print("o/k: Increase/Decrease Rotate Distance")
        print("p/l: Increase/Decrease Strafe Distance")
        print("u/h: Increase/Decrease Height Distance")
        print("1/2: Increase/Decrease eef_x_position")
        print("3/4: Increase/Decrease eef_y_position")
        print("5/6: Increase/Decrease eef_z_position")
        print("Press Space to exit.")
        rospy.sleep(1)
        try:
            while not rospy.is_shutdown():
                key = self.get_key()
                print(f"current key: {key}")

                if key == "q":
                    self.current_index = (self.current_index - 1) % len(self.arm_trajectory)
                    self.left_arm_traj = self.arm_trajectory[self.current_index][0]
                    self.right_arm_traj = self.arm_trajectory[self.current_index][1]
                    self.arm_pose_publish(self.left_arm_traj, self.right_arm_traj)

                elif key == "e":
                    self.current_index = (self.current_index + 1) % len(self.arm_trajectory)
                    self.left_arm_traj = self.arm_trajectory[self.current_index][0]
                    self.right_arm_traj = self.arm_trajectory[self.current_index][1]
                    self.arm_pose_publish(self.left_arm_traj, self.right_arm_traj)

                # Arm control for keys 1-5 (increase left arm joint values)
                elif key == "1":
                    # self.left_arm_traj[0] += self.delta_joint
                    # self.right_arm_traj[0] += self.delta_joint
                    # self.arm_pose_publish(self.left_arm_traj, self.right_arm_traj)
                    self.left_eef_position_xyz[0]+=self.eef_speed
                    self.right_eef_position_xyz[0]+=self.eef_speed
                    self.eef_pose_publish(self.left_eef_position_xyz,self.right_eef_position_xyz,self.left_eef_quat_xyzw,self.right_eef_quat_xyzw)

                elif key == "2":
                    self.left_eef_position_xyz[0]-=self.eef_speed
                    self.right_eef_position_xyz[0]-=self.eef_speed
                    self.eef_pose_publish(self.left_eef_position_xyz,self.right_eef_position_xyz,self.left_eef_quat_xyzw,self.right_eef_quat_xyzw)
                    # self.left_arm_traj[1] += self.delta_joint
                    # self.right_arm_traj[1] -= self.delta_joint
                    # self.arm_pose_publish(self.left_arm_traj, self.right_arm_traj)

                elif key == "3":
                    self.left_eef_position_xyz[1]+=self.eef_speed
                    self.right_eef_position_xyz[1]-=self.eef_speed
                    self.eef_pose_publish(self.left_eef_position_xyz,self.right_eef_position_xyz,self.left_eef_quat_xyzw,self.right_eef_quat_xyzw)
                    # self.left_arm_traj[2] += self.delta_joint
                    # self.right_arm_traj[2] -= self.delta_joint
                    # self.arm_pose_publish(self.left_arm_traj, self.right_arm_traj)

                elif key == "4":
                    self.left_eef_position_xyz[1]-=self.eef_speed
                    self.right_eef_position_xyz[1]+=self.eef_speed
                    self.eef_pose_publish(self.left_eef_position_xyz,self.right_eef_position_xyz,self.left_eef_quat_xyzw,self.right_eef_quat_xyzw)
                    # self.left_arm_traj[3] += self.delta_joint
                    # self.right_arm_traj[3] += self.delta_joint
                    # self.arm_pose_publish(self.left_arm_traj, self.right_arm_traj)

                elif key == "5":
                    self.left_eef_position_xyz[2]+=self.eef_speed
                    self.right_eef_position_xyz[2]+=self.eef_speed
                    self.eef_pose_publish(self.left_eef_position_xyz,self.right_eef_position_xyz,self.left_eef_quat_xyzw,self.right_eef_quat_xyzw)
                    # self.left_arm_traj[4] += self.delta_joint
                    # self.right_arm_traj[4] -= self.delta_joint
                    # self.arm_pose_publish(self.left_arm_traj, self.right_arm_traj)

                # Arm control for keys 6-0 (decrease left arm joint values)
                elif key == "0":
                    self.left_arm_traj[0] -= self.delta_joint
                    self.right_arm_traj[0] -= self.delta_joint
                    self.arm_pose_publish(self.left_arm_traj, self.right_arm_traj)

                elif key == "9":
                    self.left_arm_traj[1] -= self.delta_joint
                    self.right_arm_traj[1] += self.delta_joint
                    self.arm_pose_publish(self.left_arm_traj, self.right_arm_traj)

                elif key == "8":
                    self.left_arm_traj[2] -= self.delta_joint
                    self.right_arm_traj[2] += self.delta_joint
                    self.arm_pose_publish(self.left_arm_traj, self.right_arm_traj)

                elif key == "7":
                    self.left_arm_traj[3] -= self.delta_joint
                    self.right_arm_traj[3] -= self.delta_joint
                    self.arm_pose_publish(self.left_arm_traj, self.right_arm_traj)

                elif key == "6":
                    self.left_eef_position_xyz[2]-=self.eef_speed
                    self.right_eef_position_xyz[2]-=self.eef_speed
                    self.eef_pose_publish(self.left_eef_position_xyz,self.right_eef_position_xyz,self.left_eef_quat_xyzw,self.right_eef_quat_xyzw)
                    # self.left_arm_traj[4] -= self.delta_joint
                    # self.right_arm_traj[4] += self.delta_joint
                    # self.arm_pose_publish(self.left_arm_traj, self.right_arm_traj)

                elif key == "w":
                    self.left_eef_position_xyz= [0.35, 0.26, 0.2]
                    self.right_eef_position_xyz= [0.35, -0.26, 0.2]
                    self.eef_pose_publish(self.left_eef_position_xyz,self.right_eef_position_xyz,self.left_eef_quat_xyzw,self.right_eef_quat_xyzw)
                    #self.publish_robot_vel([+self.forward_speed, 0, 0, self.robot_height])

                elif key == "s":
                    self.publish_robot_vel([-self.forward_speed, 0, 0, self.robot_height])

                elif key == "a":
                    self.publish_robot_vel([0, 0, +self.rotate_speed*2*180/3.14, self.robot_height])

                elif key == "d":
                    self.publish_robot_vel([0, 0, -self.rotate_speed*2*180/3.14, self.robot_height])

                elif key == "z":
                    self.publish_robot_vel([0, +self.strafe_speed, 0, self.robot_height])

                elif key == "c":
                    self.publish_robot_vel([0, -self.strafe_speed, 0, self.robot_height])

                elif key == "v":
                    self.robot_height = self.robot_height + self.height_speed
                    self.publish_robot_vel([0, 0, 0, self.robot_height])

                elif key == "n":
                    self.robot_height = self.robot_height - self.height_speed
                    self.publish_robot_vel([0, 0, 0, self.robot_height])

                elif key == "x":
                    self.publish_robot_vel([0, 0, 0, 0])
                
                elif key == "b":
                    self.publish_robot_stp(self.stop_step_num)

                elif key == "i":
                    self.forward_speed += self.speed_increment
                    if self.forward_speed > 2.00:
                        self.forward_speed = 2.00
                    print(f"Forward Speed: {self.forward_speed}")

                elif key == "j":
                    self.forward_speed -= self.speed_increment
                    if self.forward_speed < 0.00:
                        self.forward_speed = 0.00
                    print(f"Forward Speed: {self.forward_speed}")

                elif key == "o":
                    self.rotate_speed += self.speed_increment
                    if self.rotate_speed > 2.00:
                        self.rotate_speed = 2.00
                    print(f"Rotate Speed: {self.rotate_speed}")

                elif key == "k":
                    self.rotate_speed -= self.speed_increment
                    if self.rotate_speed < 0.00:
                        self.rotate_speed = 0.00
                    print(f"Rotate Speed: {self.rotate_speed}")

                elif key == "p":
                    self.strafe_speed += self.speed_increment
                    if self.strafe_speed > 0.50:
                        self.strafe_speed = 0.50
                    print(f"Strafe Speed: {self.strafe_speed}")

                elif key == "l":
                    self.strafe_speed -= self.speed_increment
                    if self.strafe_speed < 0.00:
                        self.strafe_speed = 0.00                    
                    print(f"Strafe Speed: {self.strafe_speed}")

                elif key == "u":
                    self.height_speed += self.speed_increment
                    if self.height_speed > 0.08:
                        self.height_speed = 0.08
                    print(f"Height Speed: {self.height_speed}")

                elif key == "h":
                    self.height_speed -= self.speed_increment
                    if self.height_speed < 0.00:
                        self.height_speed = 0.00
                    print(f"Height Speed: {self.height_speed}")

                elif key == "y":
                    if self.arm_mode == 2:
                        self.arm_mode_change(1)
                    else:
                        self.arm_mode_change(2)
                    print(f"arm_mode_change: {self.arm_mode}")


                elif key == " ":
                    self.publish_robot_vel([0, 0, 0, 0])
                    self.arm_mode_change(1)
                    print("\nExiting program.")
                    break

                rospy.sleep(0.05)
        except KeyboardInterrupt:
            self.publish_robot_vel([0, 0, 0, 0])
            self.arm_mode_change(1)
            print("\nExiting program.")
        

def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--forward_speed", type=float, default=0.50, help="Forward/backward speed")
    parser.add_argument("--strafe_speed", type=float, default=0.20, help="Strafe speed")
    parser.add_argument("--rotate_speed", type=float, default=0.25, help="Rotate speed")
    parser.add_argument("--height_speed", type=float, default=0.04, help="Up/down speed")
    parser.add_argument("--stop_step_num", type=int, default=3, help="Number of stopping steps")
    parser.add_argument("--speed_increment", type=float, default=0.05, help="Speed increment/decrement value")
    parser.add_argument("--delta_joint", type=float, default=5.0, help="Joint increment/decrement value")
    parser.add_argument("--eef_speed",type=float,default=0.01, help="moving speed of end effector")
    return parser.parse_args()

if __name__ == "__main__":
    args = get_args()
    arm_trajectory = [
        [[  0,   0,   0,   0,   0,   0,   0], [  0,   0,   0,   0,   0,   0,   0]],
        [[ 50,   0,   0, -95,   0,   0,   0], [ 50,   0,   0, -95,   0,   0,   0]],
        [[ 50,  30,   0, -95,  40,   0,   0], [ 50, -30,   0, -95, -40,   0,   0]],
        [[ 25,  30, 115, -95,  80,   0,   0], [ 25, -30,-115, -95, -80,   0,   0]],
        [[ 25,  85, 115, -95,  80,   0,   0], [ 25, -85,-115, -95, -80,   0,   0]],
        [[ 25, 115, 120, -65,  80,   0,   0], [ 25,-115,-120, -65, -80,   0,   0]],
        [[ 80, 145, 110, -65, 110,   0,   0], [ 80,-145,-100, -65,-110,   0,   0]],
        [[ 80, 115, 120, -65,  80,   0,   0], [ 80,-115,-120, -65, -80,   0,   0]],
        [[ 55,  85, 115, -95,  80,   0,   0], [ 55, -85,-115, -95, -80,   0,   0]],
        [[ 50,  30, 115, -95,  80,   0,   0], [ 50, -30,-115, -95, -80,   0,   0]],
        [[  0,   0,   0,   0,   0,   0,   0], [  0,   0,   0,   0,   0,   0,   0]], 

        [[ 50,   0,   0, -95,   0,   0,   0], [ 50,   0,   0, -95,   0,   0,   0]],
        [[ 50,  30,   0, -95,  40,   0,   0], [ 50, -30,   0, -95, -40,   0,   0]],
        [[ 50,  30,  60, -95,  80,   0,   0], [ 50, -30, -60, -95, -80,   0,   0]],
        [[ 55,  85,  60, -95,  80,   0,   0], [ 55, -85, -60, -95, -80,   0,   0]],
        [[ 90, 115,  70, -65,  80,   0,   0], [ 90,-115, -70, -65, -80,   0,   0]],
        [[ 90, 145,  70, -65, 110,   0,   0], [ 90,-145, -70, -65,-110,   0,   0]],
        [[ 90, 115,  70, -65,  80,   0,   0], [ 90,-115, -70, -65, -80,   0,   0]],
        [[ 55,  85,  60, -95,  80,   0,   0], [ 55, -85, -60, -95, -80,   0,   0]],
        [[ 50,  30,  60, -95,  80,   0,   0], [ 50, -30, -60, -95, -80,   0,   0]],
        [[  0,   0,   0,   0,   0,   0,   0], [  0,   0,   0,   0,   0,   0,   0]]
    ]

    rc = RobotControl(
        arm_trajectory=arm_trajectory,
        forward_speed=args.forward_speed,
        strafe_speed=args.strafe_speed,
        rotate_speed=args.rotate_speed,
        height_speed=args.height_speed,
        stop_step_num=args.stop_step_num,
        speed_increment=args.speed_increment,
        delta_joint = args.delta_joint,
        eef_speed=args.eef_speed
    )
    rc.run()