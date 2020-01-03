"""Ragdoll Object

Please note that, this is not a complete feature. Most likely many things
are still quite hard to accomplish using RagDoll. But it will be gradually
better in time. It just needs some time. Therefore, I am going to visit
this file every now and then.
"""
import math
from typing import Any, Dict

from payton.scene.geometry.base import Line
from payton.scene.geometry.cube import Cube
from payton.scene.geometry.sphere import Sphere
from payton.scene.material import WIREFRAME

R_COLLAR = "right_collar"
L_COLLAR = "left_collar"
R_SHOULDER = "right_shoulder"
L_SHOULDER = "left_shoulder"
NECK = "neck"
HEAD = "head"
R_HIP = "right_hip"
L_HIP = "left_hip"
R_ELBOW = "right_elbow"
L_ELBOW = "left_elbow"
R_KNEE = "right_knee"
L_KNEE = "left_knee"
R_WRIST = "right_wrist"
L_WRIST = "left_wrist"
R_FOOT = "right_foot"
L_FOOT = "left_foot"


class Joint(Sphere):
    def __init__(self, name: str, **kwargs):
        super().__init__(**kwargs)
        self.radius = 0.01
        self.material.display = WIREFRAME
        self.build_sphere()
        self.name = name


class Bone(Line):
    def __init__(
        self, root_joint_name: str, end_joint_name: str, length: float, **kwargs,
    ):
        super().__init__(**kwargs)
        root_joint = Joint(name=root_joint_name)
        end_joint = Joint(name=end_joint_name)
        self.add_child(root_joint_name, root_joint)
        self.root = self.children[root_joint_name]
        self.root.add_child(end_joint_name, end_joint)
        self.end = self.root.children[end_joint_name]
        self.end.position = [0.0, 0.0, length]

    def connect_to_root(self, joint: Joint):
        self.root.add_child(joint.name, joint)

    def connect_to_end(self, joint: Joint):
        self.end.add_child(joint.name, joint)


class RagDoll(Bone):
    def __init__(self, **kwargs: Any) -> None:
        kwargs["root_joint_name"] = "lower_spine"
        kwargs["end_joint_name"] = "higher_spine"
        kwargs["length"] = 1.0
        super().__init__(**kwargs)
        self.joints: Dict[str, Joint] = {}
        self.position = [0.0, 0.0, 2.0]
        self.setup_joints()
        self.setup_geometry()

    def setup_geometry(self):
        self.joints[HEAD].add_child(
            "mesh", Cube(from_corner=[-0.2, -0.2, 0], to_corner=[0.2, 0.2, 0.4]),
        )
        self.joints[NECK].add_child(
            "mesh", Cube(from_corner=[-0.1, -0.1, 0], to_corner=[0.1, 0.1, 0.2]),
        )
        self.joints[R_COLLAR].add_child(
            "mesh", Cube(from_corner=[-0.1, -0.1, 0], to_corner=[0.1, 0.1, 0.5]),
        )
        self.joints[L_COLLAR].add_child(
            "mesh", Cube(from_corner=[-0.1, -0.1, 0], to_corner=[0.1, 0.1, 0.5]),
        )
        self.joints[R_SHOULDER].add_child(
            "mesh", Cube(from_corner=[-0.1, -0.1, 0], to_corner=[0.1, 0.1, 0.5]),
        )
        self.joints[L_SHOULDER].add_child(
            "mesh", Cube(from_corner=[-0.1, -0.1, 0], to_corner=[0.1, 0.1, 0.5]),
        )
        self.joints[R_ELBOW].add_child(
            "mesh", Cube(from_corner=[-0.1, -0.1, 0], to_corner=[0.08, 0.08, 0.5]),
        )
        self.joints[L_ELBOW].add_child(
            "mesh", Cube(from_corner=[-0.08, -0.08, 0], to_corner=[0.08, 0.08, 0.5]),
        )
        self.joints[R_WRIST].add_child(
            "mesh", Cube(from_corner=[-0.1, -0.01, 0], to_corner=[0.1, 0.01, 0.2]),
        )
        self.joints[L_WRIST].add_child(
            "mesh", Cube(from_corner=[-0.1, -0.01, 0], to_corner=[0.1, 0.01, 0.2]),
        )
        self.root.add_child(
            "mesh", Cube(from_corner=[-0.3, -0.2, 0], to_corner=[0.3, 0.2, 1.0]),
        )
        self.joints[R_HIP].add_child(
            "mesh", Cube(from_corner=[-0.15, -0.15, 0], to_corner=[0.15, 0.15, 1.0]),
        )
        self.joints[L_HIP].add_child(
            "mesh", Cube(from_corner=[-0.15, -0.15, 0], to_corner=[0.15, 0.15, 1.0]),
        )
        self.joints[R_KNEE].add_child(
            "mesh", Cube(from_corner=[-0.1, -0.1, 0], to_corner=[0.1, 0.1, 1.0]),
        )
        self.joints[L_KNEE].add_child(
            "mesh", Cube(from_corner=[-0.1, -0.1, 0], to_corner=[0.1, 0.1, 1.0]),
        )
        self.joints[R_FOOT].add_child(
            "mesh", Cube(from_corner=[-0.05, -0.05, 0], to_corner=[0.05, 0.05, 0.3]),
        )
        self.joints[L_FOOT].add_child(
            "mesh", Cube(from_corner=[-0.08, -0.08, 0], to_corner=[0.08, 0.08, 0.3]),
        )

    def setup_joints(self):
        neck = Bone(root_joint_name="lower_neck", end_joint_name="head", length=0.2)
        self.connect_to_end(neck.root)
        head = Bone(root_joint_name="head", end_joint_name="head_top", length=0.4)
        neck.connect_to_end(head.root)
        left_collar = Bone(root_joint_name="l_collar_joint", end_joint_name="l_shoulder", length=0.5,)
        self.connect_to_end(left_collar.root)

        right_collar = Bone(root_joint_name="r_collar_joint", end_joint_name="r_shoulder", length=0.5,)
        self.connect_to_end(right_collar.root)

        right_collar.root.rotate_around_y(math.radians(100))
        left_collar.root.rotate_around_y(math.radians(-100))

        right_upper_arm = Bone(root_joint_name="r_shoulder", end_joint_name="r_elbow", length=0.5)
        right_collar.connect_to_end(right_upper_arm.root)
        right_upper_arm.rotate_around_y(math.radians(20))

        left_upper_arm = Bone(root_joint_name="l_shoulder", end_joint_name="l_elbow", length=0.5)
        left_collar.connect_to_end(left_upper_arm.root)
        left_upper_arm.rotate_around_y(math.radians(-20))

        right_lower_arm = Bone(root_joint_name="r_elbow", end_joint_name="r_wrist", length=0.5)
        right_upper_arm.connect_to_end(right_lower_arm.root)
        left_lower_arm = Bone(root_joint_name="l_elbow", end_joint_name="l_wrist", length=0.5)
        left_upper_arm.connect_to_end(left_lower_arm.root)

        right_hand = Bone(root_joint_name="r_wrist", end_joint_name="r_hand", length=0.2)
        right_lower_arm.connect_to_end(right_hand.root)
        left_hand = Bone(root_joint_name="l_wrist", end_joint_name="l_hand", length=0.2)
        left_lower_arm.connect_to_end(left_hand.root)

        right_upper_leg = Bone(root_joint_name="r_hip", end_joint_name="r_knee", length=1.0)
        self.connect_to_root(right_upper_leg.root)
        right_upper_leg.root.rotate_around_y(math.radians(160))

        left_upper_leg = Bone(root_joint_name="l_hip", end_joint_name="l_knee", length=1.0)
        self.connect_to_root(left_upper_leg.root)
        left_upper_leg.root.rotate_around_y(math.radians(-160))

        right_lower_leg = Bone(root_joint_name="r_knee", end_joint_name="r_foot", length=1.0)
        right_upper_leg.connect_to_end(right_lower_leg.root)
        left_lower_leg = Bone(root_joint_name="l_knee", end_joint_name="l_foot", length=1.0)
        left_upper_leg.connect_to_end(left_lower_leg.root)
        right_foot = Bone(root_joint_name="r_foot", end_joint_name="r_toe", length=0.3)
        right_lower_leg.connect_to_end(right_foot.root)
        right_foot.root.rotate_around_x(math.radians(90))

        left_foot = Bone(root_joint_name="l_foot", end_joint_name="l_toe", length=0.3)
        left_lower_leg.connect_to_end(left_foot.root)
        left_foot.root.rotate_around_x(math.radians(90))

        self.joints[NECK] = neck.root
        self.joints[HEAD] = head.root
        self.joints[R_COLLAR] = right_collar.root
        self.joints[L_COLLAR] = left_collar.root
        self.joints[R_SHOULDER] = right_upper_arm.root
        self.joints[L_SHOULDER] = left_upper_arm.root
        self.joints[R_ELBOW] = right_lower_arm.root
        self.joints[L_ELBOW] = left_lower_arm.root
        self.joints[R_WRIST] = right_hand.root
        self.joints[L_WRIST] = left_hand.root
        self.joints[R_HIP] = right_upper_leg.root
        self.joints[L_HIP] = left_upper_leg.root
        self.joints[R_KNEE] = right_lower_leg.root
        self.joints[L_KNEE] = left_lower_leg.root
        self.joints[R_FOOT] = right_foot.root
        self.joints[L_FOOT] = left_foot.root
