import math

import isaaclab.sim as sim_utils
import isaaclab.terrains as terrain_gen
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import CurriculumTermCfg as CurrTerm
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import ContactSensorCfg, RayCasterCfg, patterns
from isaaclab.terrains import TerrainImporterCfg
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR, ISAACLAB_NUCLEUS_DIR
from isaaclab.utils.noise import AdditiveUniformNoiseCfg as Unoise
# from omni.isaac.lab.sensors import ContactSensorCfg

from unitree_rl_lab.assets.robots.unitree import UNITREE_GO2_CFG as ROBOT_CFG
from unitree_rl_lab.tasks.locomotion import mdp

COBBLESTONE_ROAD_CFG = terrain_gen.TerrainGeneratorCfg(
    size=(8.0, 8.0),
    border_width=20.0,
    num_rows=10,
    num_cols=20,
    horizontal_scale=0.1,
    vertical_scale=0.005,
    slope_threshold=0.75,
    difficulty_range=(0.0, 1.0),
    use_cache=False,
    sub_terrains={
        "flat": terrain_gen.MeshPlaneTerrainCfg(proportion=0.1),
        "random_rough": terrain_gen.HfRandomUniformTerrainCfg(
            proportion=0.1, noise_range=(0.01, 0.06), noise_step=0.01, border_width=0.25
        ),
        "hf_pyramid_slope": terrain_gen.HfPyramidSlopedTerrainCfg(
            proportion=0.1, slope_range=(0.0, 0.4), platform_width=2.0, border_width=0.25
        ),
        "hf_pyramid_slope_inv": terrain_gen.HfInvertedPyramidSlopedTerrainCfg(
            proportion=0.1, slope_range=(0.0, 0.4), platform_width=2.0, border_width=0.25
        ),
        "boxes": terrain_gen.MeshRandomGridTerrainCfg(
            proportion=0.0, grid_width=0.45, grid_height_range=(0.05, 0.2), platform_width=2.0
        ),
        "pyramid_stairs": terrain_gen.MeshPyramidStairsTerrainCfg(
            proportion=0.3,
            step_height_range=(0.05, 0.23),
            step_width=0.3,
            platform_width=3.0,
            border_width=1.0,
            holes=False,
        ),
        "pyramid_stairs_inv": terrain_gen.MeshInvertedPyramidStairsTerrainCfg(
            proportion=0.3,
            step_height_range=(0.05, 0.23),
            step_width=0.3,
            platform_width=3.0,
            border_width=1.0,
            holes=False,
        ),
    },
)

import logging
logging.warning(f"DEBUG module level: COBBLESTONE_ROAD_CFG.size = {COBBLESTONE_ROAD_CFG.size}, id = {id(COBBLESTONE_ROAD_CFG)}")


@configclass
class RobotSceneCfg(InteractiveSceneCfg):
    """Configuration for the terrain scene with a legged robot."""

    # ground terrain
    terrain = TerrainImporterCfg(
        prim_path="/World/ground",
        terrain_type="generator",  # "plane", "generator"
        terrain_generator=COBBLESTONE_ROAD_CFG,  # None, ROUGH_TERRAINS_CFG
        max_init_terrain_level=1,
        collision_group=-1,
        physics_material=sim_utils.RigidBodyMaterialCfg(
            friction_combine_mode="multiply",
            restitution_combine_mode="multiply",
            static_friction=1.0,
            dynamic_friction=1.0,
        ),
        visual_material=sim_utils.MdlFileCfg(
            mdl_path=f"{ISAACLAB_NUCLEUS_DIR}/Materials/TilesMarbleSpiderWhiteBrickBondHoned/TilesMarbleSpiderWhiteBrickBondHoned.mdl",
            project_uvw=True,
            texture_scale=(0.25, 0.25),
        ),
        debug_vis=False,
    )
    # robots
    robot: ArticulationCfg = ROBOT_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

    # sensors
    height_scanner = RayCasterCfg(
        prim_path="{ENV_REGEX_NS}/Robot/base",
        offset=RayCasterCfg.OffsetCfg(pos=(0.0, 0.0, 20.0)),
        ray_alignment="yaw",
        pattern_cfg=patterns.GridPatternCfg(resolution=0.1, size=[1.6, 1.0]),
        debug_vis=False,
        mesh_prim_paths=["/World/ground"],
    )
    contact_forces = ContactSensorCfg(prim_path="{ENV_REGEX_NS}/Robot/.*", history_length=3, track_air_time=True)
    # lights
    sky_light = AssetBaseCfg(
        prim_path="/World/skyLight",
        spawn=sim_utils.DomeLightCfg(
            intensity=750.0,
            texture_file=f"{ISAAC_NUCLEUS_DIR}/Materials/Textures/Skies/PolyHaven/kloofendal_43d_clear_puresky_4k.hdr",
        ),
    )


@configclass
class EventCfg:
    """Configuration for events."""

    # startup
    physics_material = EventTerm(
        func=mdp.randomize_rigid_body_material,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=".*"),
            "static_friction_range": (0.3, 1.2),
            "dynamic_friction_range": (0.3, 1.2),
            "restitution_range": (0.0, 0.15),
            "num_buckets": 64,
        },
    )

    add_base_mass = EventTerm(
        func=mdp.randomize_rigid_body_mass,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names="base"),
            "mass_distribution_params": (-1.0, 3.0),
            "operation": "add",
        },
    )

    # reset
    base_external_force_torque = EventTerm(
        func=mdp.apply_external_force_torque,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names="base"),
            "force_range": (0.0, 0.0),
            "torque_range": (-0.0, 0.0),
        },
    )

    # base_external_force_torque_curriculum = EventTerm(
    #     func=mdp.apply_external_force_torque_curriculum,  # 你刚才在events.py写的
    #     mode="reset",
    #     params={
    #         "asset_cfg": SceneEntityCfg("robot", body_names="base"),
    #     },
    # )

    reset_base = EventTerm(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "pose_range": {"x": (-0.5, 0.5), "y": (-0.5, 0.5), "yaw": (-3.14, 3.14)},
            "velocity_range": {
                "x": (0.0, 0.0),
                "y": (0.0, 0.0),
                "z": (0.0, 0.0),
                "roll": (0.0, 0.0),
                "pitch": (0.0, 0.0),
                "yaw": (0.0, 0.0),
            },
        },
    )

    reset_robot_joints = EventTerm(
        func=mdp.reset_joints_by_scale,
        mode="reset",
        params={
            "position_range": (1.0, 1.0),
            "velocity_range": (-1.0, 1.0),
        },
    )

    # interval
    push_robot = EventTerm(
        func=mdp.push_by_setting_velocity,
        mode="interval",
        interval_range_s=(5.0, 10.0),
        params={"velocity_range": {"x": (-0.5, 0.5), "y": (-0.5, 0.5)}},
    )

    #     # 新增：MoB 时钟相位更新（关键！）
    # update_gait_phase = EventTerm(
    #     func=mdp.update_gait_phase_accum,
    #     mode="pre_step",  # 每仿真步前执行
    #     params={}
    # )
    
    # reset_gait_phase = EventTerm(
    #     func=mdp.reset_gait_phase,
    #     mode="reset",  # 环境 reset 时随机化相位
    #     params={}
    # )

    randomize_kp = EventTerm(
        func=mdp.randomize_actuator_gains,
        mode="startup",  # 或 "interval" 每N步随机化
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=".*"),
            "stiffness_distribution_params": (0.8,1.2),    # KP范围: 默认30，±5
            "damping_distribution_params": (0.8,1.2),        # KD范围: 默认1.0，±0.2
            "operation": "scale",                 # "set"直接设置，"scale"相对缩放
        },
    )



@configclass
class CommandsCfg:
    """Command specifications for the MDP."""

    base_velocity = mdp.UniformLevelVelocityCommandCfg(
        asset_name="robot",
        resampling_time_range=(10.0, 10.0),
        rel_standing_envs=0.1,   #0.1
        debug_vis=True,
        ranges=mdp.UniformLevelVelocityCommandCfg.Ranges(
            lin_vel_x=(-0.1, 0.1), lin_vel_y=(-0.1, 0.1), ang_vel_z=(-1.0, 1.0)
        ),
        limit_ranges=mdp.UniformLevelVelocityCommandCfg.Ranges(
            lin_vel_x=(-1.0, 1.0), lin_vel_y=(-0.4, 0.4), ang_vel_z=(-1.0, 1.0)
        ),
    )

# @configclass
# class CommandsCfg:
#     """Command specifications for the MDP."""

#     base_velocity = mdp.UniformVelocityCommandCfg(
#         asset_name="robot",
#         resampling_time_range=(10.0, 10.0),
#         rel_standing_envs=0.1,
#         rel_heading_envs=1.0,
#         heading_command=True,
#         heading_control_stiffness=0.5,
#         debug_vis=True,
#         ranges=mdp.UniformVelocityCommandCfg.Ranges(
#             lin_vel_x=(-1.0, 1.0), lin_vel_y=(-1.0, 1.0), ang_vel_z=(-1.0, 1.0), heading=(-math.pi, math.pi)
#         ),
#     )

# @configclass
# class CommandsCfg:
#     """Command specifications for the MDP."""

#     base_velocity = mdp.UniformLevelVelocityCommandCfg(
#         asset_name="robot",
#         resampling_time_range=(10.0, 10.0),
#         rel_standing_envs=0.08,   #0.1
#         debug_vis=True,
#         ranges=mdp.UniformLevelVelocityCommandCfg.Ranges(
#             lin_vel_x=(-0.1, 0.1), lin_vel_y=(-0.1, 0.1), ang_vel_z=(-1.0, 1.0)
#         ),
#         limit_ranges=mdp.UniformLevelVelocityCommandCfg.Ranges(
#             lin_vel_x=(-1.5, 1.5), lin_vel_y=(-0.6, 0.6), ang_vel_z=(-1.0, 1.0)
#         ),
#     )

@configclass
class ActionsCfg:
    """Action specifications for the MDP."""

    JointPositionAction = mdp.JointPositionActionCfg(
        asset_name="robot", joint_names=[".*"], scale=0.25, use_default_offset=True, clip={".*": (-100.0, 100.0)}
    )


@configclass
class ObservationsCfg:
    """Observation specifications for the MDP."""

    @configclass
    class PolicyCfg(ObsGroup):
        """Observations for policy group."""

        # observation terms (order preserved)
        base_ang_vel = ObsTerm(func=mdp.base_ang_vel, scale=0.2, clip=(-100, 100), noise=Unoise(n_min=-0.2, n_max=0.2))
        projected_gravity = ObsTerm(func=mdp.projected_gravity, clip=(-100, 100), noise=Unoise(n_min=-0.05, n_max=0.05))
        velocity_commands = ObsTerm(
            func=mdp.generated_commands, clip=(-100, 100), params={"command_name": "base_velocity"}
        )
        joint_pos_rel = ObsTerm(func=mdp.joint_pos_rel, clip=(-100, 100), noise=Unoise(n_min=-0.01, n_max=0.01))
        joint_vel_rel = ObsTerm(
            func=mdp.joint_vel_rel, scale=0.05, clip=(-100, 100), noise=Unoise(n_min=-1.5, n_max=1.5)
        )
        last_action = ObsTerm(func=mdp.last_action, clip=(-100, 100))

        gait_phase = ObsTerm(
            func=mdp.gait_phase, 
            params={"period": 0.5}  # 根据你训练的步态频率调整
        )

        def __post_init__(self):
            self.history_length = 3
            self.enable_corruption = True
            self.concatenate_terms = True

    # observation groups
    policy: PolicyCfg = PolicyCfg()

    @configclass
    class CriticCfg(ObsGroup):
        """Observations for critic group."""

        base_lin_vel = ObsTerm(func=mdp.base_lin_vel, clip=(-100, 100))
        base_ang_vel = ObsTerm(func=mdp.base_ang_vel, scale=0.2, clip=(-100, 100))
        projected_gravity = ObsTerm(func=mdp.projected_gravity, clip=(-100, 100))
        velocity_commands = ObsTerm(
            func=mdp.generated_commands, clip=(-100, 100), params={"command_name": "base_velocity"}
        )
        joint_pos_rel = ObsTerm(func=mdp.joint_pos_rel, clip=(-100, 100))
        joint_vel_rel = ObsTerm(func=mdp.joint_vel_rel, scale=0.05, clip=(-100, 100))
        joint_effort = ObsTerm(func=mdp.joint_effort, scale=0.01, clip=(-100, 100))
        last_action = ObsTerm(func=mdp.last_action, clip=(-100, 100))
        height_scanner = ObsTerm(func=mdp.height_scan,
            params={"sensor_cfg": SceneEntityCfg("height_scanner")},
            clip=(-1.0, 5.0),
        )


        def __post_init__(self):
            self.history_length = 5

    # privileged observations
    critic: CriticCfg = CriticCfg()


@configclass
class RewardsCfg:
    """Reward terms for the MDP."""

    # -- task
    track_lin_vel_xy = RewTerm(
        func=mdp.track_lin_vel_xy_exp, weight=3.5, params={"command_name": "base_velocity", "std": math.sqrt(0.25)}
    )
    track_ang_vel_z = RewTerm(
        func=mdp.track_ang_vel_z_exp, weight=2.0, params={"command_name": "base_velocity", "std": math.sqrt(0.25)}
    )

    # -- base
    base_linear_velocity = RewTerm(func=mdp.lin_vel_z_l2, weight=-2.0)
    base_angular_velocity = RewTerm(func=mdp.ang_vel_xy_l2, weight=-0.05)
    joint_vel = RewTerm(func=mdp.joint_vel_l2, weight=-0.001)
    joint_acc = RewTerm(func=mdp.joint_acc_l2, weight=-2.5e-7)
    joint_torques = RewTerm(func=mdp.joint_torques_l2, weight=-2.5e-5)
    action_rate = RewTerm(func=mdp.action_rate_l2, weight=-0.15)
    dof_pos_limits = RewTerm(func=mdp.joint_pos_limits, weight=-10.0)
    energy = RewTerm(func=mdp.energy, weight=-3e-5)

    # -- robot

    base_height_l2 = RewTerm(
        func=mdp.base_height,
        weight=-1.0,
        params={
            "target_height": 0.3,
            "sensor_cfg": SceneEntityCfg("height_scanner"),
        },
    )

    # feet_height_body = RewTerm(
    #     func=mdp.feet_height_body,
    #     weight=-0.01,
    #     params={
    #         "asset_cfg": SceneEntityCfg("robot", body_names=".*_foot"),
    #         "target_height": -0.2,
    #         "command_name": "base_velocity",
    #     }
    # )


    # smoothness = RewTerm(func=mdp.smoothness, weight=-0.01)

    flat_orientation_l2 = RewTerm(func=mdp.flat_orientation_l2, weight=-2.5)

    joint_pos = RewTerm(
        func=mdp.joint_position_penalty,
        weight=-0.7,
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=".*"),
            "stand_still_scale": 5.0,
            "velocity_threshold": 0.3,
        },
    )

    # # # -- feet   (官方源码有问题，符号反了)
    # feet_air_time = RewTerm(
    #     func=mdp.feet_air_time,
    #     weight=0.15, #0.15
    #     params={
    #         "sensor_cfg": SceneEntityCfg("contact_forces", body_names=".*_foot"),
    #         "command_name": "base_velocity",
    #         "threshold": 0.5,
    #     },
    # )

    # air_time_variance = RewTerm(
        
    #     func=mdp.air_time_variance_penalty,
    #     weight=-1.0,
    #     params={"sensor_cfg": SceneEntityCfg("contact_forces", body_names=".*_foot")
    #     },
    # )

    feet_slide = RewTerm(
        func=mdp.feet_slide,
        weight=-0.30,  #-0.50 -0.30
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=".*_foot"),
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=".*_foot"),
        },
    )

    # foot_slip = RewTerm(
    #     func=mdp.foot_slip_penalty_robust,
    #     weight=-0.30,  # 注意是负的！如果是 Cost 就用正的，看你框架定义
    #     params={
    #         "sensor_cfg": SceneEntityCfg("contact_forces", body_names=["FL_foot", "FR_foot", "RL_foot", "RR_foot"],preserve_order=True),
    #         "asset_cfg": SceneEntityCfg("robot", body_names=["FL_foot", "FR_foot", "RL_foot", "RR_foot"],preserve_order=True),
    #         "vel_deadzone": 0.1, # 如果地面很不平，可以稍微加大到 0.1
    #     }
    # )

    # foot_slip_stance_anchor = RewTerm(
    #     func=mdp.foot_slip_penalty_stance_anchor,  # 用平滑版
    #     weight=-0.01,
    #     params={
    #         "sensor_cfg": SceneEntityCfg("contact_forces", body_ids=[3, 7, 11, 15]),
    #         "asset_cfg": SceneEntityCfg("robot", body_ids=[3, 7, 11, 15]),
    #         "threshold": 3.0,
    #         "vel_deadzone": 0.08,      # 允许8cm/s微动
    #         "max_penalty": 1.0,
    #         "pos_anchor_scale": 3.0,   # 允许10cm级漂移
    #         "stance_forgiveness": 0.1,  # 前100ms不惩罚位置
    #     }
    # )

    # #抬脚到目标高度
    # feet_clearance = RewTerm(
    #     func=mdp.rewards.foot_clearance_reward,
    #     weight=0.1,
    #     params={
    #         "asset_cfg": SceneEntityCfg("robot", body_names=".*_foot"),
    #         "target_height": 0.16,
    #         "std": 0.02,
    #         "tanh_mult": 4.0,
    #     },
    # )


    feet_height_body: RewTerm = RewTerm(
        func=mdp.feet_height_body,
        weight=-0.8,  # 负值：惩罚偏离，鼓励接近target_height 0.6 0.8
        params={
            "command_name": "base_velocity",
            "asset_cfg": SceneEntityCfg("robot", body_names=["FL_foot", "FR_foot", "RL_foot", "RR_foot"]),
            "target_height": -0.15,  # 期望的足端相对身体高度
            "tanh_mult": 10.0,  # 水平速度阈值，控制摆动相检测灵敏度
        },
    )
    
    # #落地速度惩罚
    # fall_vel = RewTerm(
    #     func=mdp.body_lin_acc_l2,
    #     weight=-0.4,
    #     params={"asset_cfg": SceneEntityCfg("robot", body_names="base")
    #     },
    # )

    feet_gait_diagonal: RewTerm = RewTerm(
        func=mdp.rewards.feet_gait_diagonal_sync,
        weight=0.6,  # 可适当提高，因为去掉了周期性累积
        params={
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=".*_foot"),
            "threshold": 0.001,  # 1ms 接触即算触地
            "command_name": "base_velocity",
        },
    )

    # 步态奖励配置
    feet_gait = RewTerm(
        func=mdp.feet_gait,
        params={
            "period": 0.5,  # 0.5秒一个周期
            "offset": [0.0, 0.5, 0.0, 0.5],  # 4条腿的相位偏移
            "sensor_cfg": SceneEntityCfg(
                "contact_forces",  # 传感器名称
                body_names=[".*_foot"],  # 匹配所有脚部
            ),
            "threshold": 0.5,  # 站立相位阈值
            "command_name": "base_velocity",  # 基础速度命令
        },
        weight=1.0,  # 奖励权重
    )


    # feet_contact_forces = RewTerm(
    #     func=mdp.contact_forces,
    #     weight=0.01,  #-0.02效果不好
    #     params={
    #         # "threshold": 100.0,
    #         "threshold": 100.0,  # 单脚 > 45 N 就罚        
    #         "sensor_cfg": SceneEntityCfg("contact_forces", body_names=".*_foot"),
    #     },
    # )

    # -- other
    undesired_contacts = RewTerm(
        func=mdp.undesired_contacts,
        weight=-1.0,
        params={
            "threshold": 1,
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=[".*_hip", ".*_thigh", ".*_calf"]),
        },
    )

    upward: RewTerm = RewTerm(
        func=mdp.rewards.upward,
        weight=1.0,
        params={"asset_cfg": SceneEntityCfg("robot")
        },
    )


    # 跌倒惩罚
    early_terminate: RewTerm = RewTerm(
        func=mdp.is_terminated_term,
        weight=-0.3,                 
        params={"term_keys": ".*"
        },  
    )

    #防绊倒
    feet_stumble = RewTerm(
        func=mdp.rewards.feet_stumble,
        weight=-0.05,  # 建议初始值，可根据训练调整 -0.10  -0.08
        params={
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=".*_foot"),
        },
    )

    # #防外八
    # feet_distance_y_exp: RewTerm = RewTerm(
    #     func=mdp.rewards.feet_distance_y_exp,  # 上面定义的函数
    #     weight=0.05,                # 权重，建议从 0.5-2.0 开始调 0.10 
    #     params={
    #         "stance_width": 0.25,   # 期望站宽 25cm（Go2 典型值约 0.23-0.28m）
    #         "std": 0.05,            # 容忍标准差，越小越严格
    #         "asset_cfg": SceneEntityCfg("robot", body_ids=[6, 9, 12, 15]),  # 4个足端link id
    #     },
    # )

    # #外翻抬腿
    # hip_abduction_in_swing: RewTerm = RewTerm(
    #     func=mdp.rewards.hip_abduction_during_swing,
    #     weight=0.10,  #0.10
    #     params={
    #         "swing_threshold": 0.05,
    #         "target_abduction": 0.12,
    #         "tanh_mult": 3.0,
    #         "feet_body_names": ["FR_foot", "FL_foot", "RR_foot", "RL_foot"],  # 明确指定
    #         "joint_names": ["FR_hip_joint", "FL_hip_joint", "RR_hip_joint", "RL_hip_joint"],  # 明确指定
    #     },
    # )

@configclass
class TerminationsCfg:
    """Termination terms for the MDP."""

    time_out = DoneTerm(func=mdp.time_out, time_out=True)
    base_contact = DoneTerm(
        func=mdp.illegal_contact,
        params={"sensor_cfg": SceneEntityCfg("contact_forces", body_names="base"), "threshold": 1.0},
    )
    bad_orientation = DoneTerm(func=mdp.bad_orientation, params={"limit_angle": 0.8})


@configclass
class CurriculumCfg:
    """Curriculum terms for the MDP."""
    # pass
    terrain_levels = CurrTerm(func=mdp.terrain_levels_vel)
    lin_vel_cmd_levels = CurrTerm(mdp.lin_vel_cmd_levels)


@configclass
class RobotEnvCfg(ManagerBasedRLEnvCfg):
    """Configuration for the locomotion velocity-tracking environment."""

    # Scene settings
    scene: RobotSceneCfg = RobotSceneCfg(num_envs=4096, env_spacing=2.5)
    # Basic settings
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    commands: CommandsCfg = CommandsCfg()
    # MDP settings
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    events: EventCfg = EventCfg()
    curriculum: CurriculumCfg = CurriculumCfg()

    def __post_init__(self):
        """Post initialization."""
        import sys
        print("DEBUG RobotEnvCfg.__post_init__: CALLED", file=sys.stderr, flush=True)
        print(f"DEBUG RobotEnvCfg.__post_init__: terrain_generator.size = {self.scene.terrain.terrain_generator.size}", file=sys.stderr, flush=True)
        import logging
        logging.warning(f"DEBUG RobotEnvCfg.__post_init__: terrain_generator.size = {self.scene.terrain.terrain_generator.size}")
        # general settings
        self.decimation = 4
        self.episode_length_s = 20.0
        # simulation settings
        self.sim.dt = 0.005
        self.sim.render_interval = self.decimation
        self.sim.physics_material = self.scene.terrain.physics_material
        self.sim.physx.gpu_max_rigid_patch_count = 10 * 2**15

        # update sensor update periods
        # we tick all the sensors based on the smallest update period (physics update period)
        self.scene.contact_forces.update_period = self.sim.dt
        self.scene.height_scanner.update_period = self.decimation * self.sim.dt

        # check if terrain levels curriculum is enabled - if so, enable curriculum for terrain generator
        # this generates terrains with increasing difficulty and is useful for training
        if getattr(self.curriculum, "terrain_levels", None) is not None:
            if self.scene.terrain.terrain_generator is not None:
                self.scene.terrain.terrain_generator.curriculum = True
        else:
            if self.scene.terrain.terrain_generator is not None:
                self.scene.terrain.terrain_generator.curriculum = False


@configclass
class RobotPlayEnvCfg(RobotEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        self.scene.num_envs = 500
        self.scene.terrain.terrain_generator.num_rows = 2
        self.scene.terrain.terrain_generator.num_cols = 1
        self.commands.base_velocity.ranges = self.commands.base_velocity.limit_ranges