from legged_gym.envs.base.legged_robot_config import LeggedRobotCfg, LeggedRobotCfgPPO

class G1RoughCfg(LeggedRobotCfg):
    class init_state(LeggedRobotCfg.init_state):
        pos = [0.0, 0.0, 0.8] # x,y,z [m]
        default_joint_angles = { # = target angles [rad] when action = 0.0
            #    'left_hip_yaw_joint' : 0. ,   
            #    'left_hip_roll_joint' : 0,               
            #    'left_hip_pitch_joint' : -0.4,         
            #    'left_knee_joint' : 0.8,       
            #    'left_ankle_joint' : -0.4,     
            #    'right_hip_yaw_joint' : 0., 
            #    'right_hip_roll_joint' : 0, 
            #    'right_hip_pitch_joint' : -0.4,                                       
            #    'right_knee_joint' : 0.8,                                             
            #    'right_ankle_joint' : -0.4,                                     
            #    'torso_joint' : 0., 
            #    'left_shoulder_pitch_joint' : 0., 
            #    'left_shoulder_roll_joint' : 0, 
            #    'left_shoulder_yaw_joint' : 0.,
            #    'left_elbow_joint'  : 0.,
            #    'right_shoulder_pitch_joint' : 0.,
            #    'right_shoulder_roll_joint' : 0.0,
            #    'right_shoulder_yaw_joint' : 0.,
            #    'right_elbow_joint' : 0.,
            "left_hip_pitch_joint": 0,
            "left_hip_roll_joint": 0,
            "left_hip_yaw_joint": 0,
            "left_knee_joint": 0,
            "left_ankle_pitch_joint": 0,
            "left_ankle_roll_joint": 0,
            "right_hip_pitch_joint": 0,
            "right_hip_roll_joint": 0,
            "right_hip_yaw_joint": 0,
            "right_knee_joint": 0,
            "right_ankle_pitch_joint": 0,
            "right_ankle_roll_joint": 0,
            "torso_joint": 0,
            "left_shoulder_pitch_joint": 0,
            "left_shoulder_roll_joint": 0,
            "left_shoulder_yaw_joint": 0,
            "left_elbow_pitch_joint": 0,
            "left_elbow_roll_joint": 0,
            "left_five_joint": 0,
            "left_six_joint": 0,
            "left_three_joint": 0,
            "left_four_joint": 0,
            "left_zero_joint": 0,
            "left_one_joint": 0,
            "left_two_joint": 0,
            "right_shoulder_pitch_joint": 0,
            "right_shoulder_roll_joint": 0,
            "right_shoulder_yaw_joint": 0,
            "right_elbow_pitch_joint": 0,
            "right_elbow_roll_joint": 0,
            "right_five_joint": 0,
            "right_six_joint": 0,
            "right_three_joint": 0,
            "right_four_joint": 0,
            "right_zero_joint": 0,
            "right_one_joint": 0,
            "right_two_joint": 0
        }
    
    class env(LeggedRobotCfg.env):
        num_observations = 123
        num_actions = 37

    class control(LeggedRobotCfg.control):
        # PD Drive parameters:
        control_type = 'P'
          # PD Drive parameters:
        stiffness = {'hip_yaw': 200,
                     'hip_roll': 200,
                     'hip_pitch': 200,
                     'knee': 300,
                     'ankle': 40,
                     'torso': 300,
                     'shoulder': 100,
                     "elbow":100,
                     }  # [N*m/rad]
        damping = {  'hip_yaw': 5,
                     'hip_roll': 5,
                     'hip_pitch': 5,
                     'knee': 6,
                     'ankle': 2,
                     'torso': 6,
                     'shoulder': 2,
                     "elbow":2,
                     }  # [N*m/rad]  # [N*m*s/rad]
        # action scale: target angle = actionScale * action + defaultAngle
        action_scale = 0.25
        # decimation: Number of control action updates @ sim DT per policy DT
        decimation = 4

    class asset(LeggedRobotCfg.asset):
        file = '{LEGGED_GYM_ROOT_DIR}/resources/robots/g1/urdf/g1.urdf'
        name = "g1"
        foot_name = "ankle"
        penalize_contacts_on = ["hip", "knee"]
        terminate_after_contacts_on = ["pelvis"]
        self_collisions = 1 # 1 to disable, 0 to enable...bitwise filter
        flip_visual_attachments = False
  
    class rewards(LeggedRobotCfg.rewards):
        soft_dof_pos_limit = 0.9
        base_height_target = 0.78
        class scales(LeggedRobotCfg.rewards.scales):
            tracking_lin_vel = 1.0
            tracking_ang_vel = 0.5
            lin_vel_z = -2.0
            ang_vel_xy = -1.0
            orientation = -1.0
            base_height = -100.0
            dof_acc = -3.5e-8
            feet_air_time = 1.0
            collision = 0.0
            action_rate = -0.01
            torques = 0.0
            dof_pos_limits = -10.0

class G1RoughCfgPPO(LeggedRobotCfgPPO):
    class algorithm(LeggedRobotCfgPPO.algorithm):
        entropy_coef = 0.01
    class runner(LeggedRobotCfgPPO.runner):
        run_name = ""
        experiment_name = "g1"

  
