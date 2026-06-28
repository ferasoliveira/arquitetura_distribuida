"""Teste visual mínimo: move o UR10e entre duas poses articulares."""

import math
from controller import Robot


JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

POSE_A = [0, -35, 70, -35, 45, 0]
POSE_B = [35, -55, 85, -30, -35, 45]
HOLD_SECONDS = 3.0


def main():
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())
    motors = [robot.getDevice(name) for name in JOINTS]
    if any(motor is None for motor in motors):
        raise RuntimeError("UR10e incompleto: motor ausente")

    for motor in motors:
        motor.setVelocity(0.8)

    poses = [POSE_A, POSE_B]
    pose_index = 0
    next_switch = robot.getTime()
    print("[TWO_POINTS OK] Controlador conectado aos 6 motores do UR10e")

    while robot.step(timestep) != -1:
        if robot.getTime() >= next_switch:
            target = poses[pose_index]
            for motor, degrees in zip(motors, target):
                motor.setPosition(math.radians(degrees))
            print(f"[TARGET] pose={pose_index + 1} q_deg={target}")
            pose_index = (pose_index + 1) % len(poses)
            next_switch = robot.getTime() + HOLD_SECONDS


if __name__ == "__main__":
    main()
