"""Move o UR10e entre duas poses e coleta os seis encoders articulares."""

import csv
import math
from pathlib import Path

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
AS5600_RESOLUTION = 4096


def quantize_as5600(degrees: float) -> int:
    return int(round((degrees % 360.0) * AS5600_RESOLUTION / 360.0)) % AS5600_RESOLUTION


def main():
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())
    motors = [robot.getDevice(name) for name in JOINTS]
    sensors = [robot.getDevice(name + "_sensor") for name in JOINTS]
    if any(device is None for device in motors + sensors):
        raise RuntimeError("UR10e incompleto: motor ou PositionSensor ausente")

    for motor in motors:
        motor.setVelocity(0.8)
    for sensor in sensors:
        sensor.enable(timestep)

    output_dir = Path(__file__).resolve().parents[2] / "output"
    output_dir.mkdir(parents=True, exist_ok=True)
    csv_path = output_dir / "encoder_two_points.csv"
    fields = ["timestamp"]
    for joint in range(1, 7):
        fields += [f"q{joint}_rad", f"q{joint}_deg", f"as5600_j{joint}"]

    poses = [POSE_A, POSE_B]
    pose_index = 0
    next_switch = 0.0
    next_report = 0.0

    with csv_path.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fields)
        writer.writeheader()
        print(f"[ENCODER OK] Coleta iniciada em {csv_path}")

        while robot.step(timestep) != -1:
            now = robot.getTime()
            if now >= next_switch:
                target = poses[pose_index]
                for motor, degrees in zip(motors, target):
                    motor.setPosition(math.radians(degrees))
                print(f"[TARGET] pose={pose_index + 1} q_deg={target}")
                pose_index = (pose_index + 1) % len(poses)
                next_switch = now + HOLD_SECONDS

            row = {"timestamp": f"{now:.6f}"}
            measured_deg = []
            for index, sensor in enumerate(sensors, start=1):
                radians = sensor.getValue()
                degrees = math.degrees(radians)
                measured_deg.append(round(degrees, 3))
                row[f"q{index}_rad"] = f"{radians:.9f}"
                row[f"q{index}_deg"] = f"{degrees:.6f}"
                row[f"as5600_j{index}"] = quantize_as5600(degrees)
            writer.writerow(row)

            if now >= next_report:
                csv_file.flush()
                print(f"[ENCODER] t={now:.2f}s q_deg={measured_deg}")
                next_report = now + 0.5


if __name__ == "__main__":
    main()
