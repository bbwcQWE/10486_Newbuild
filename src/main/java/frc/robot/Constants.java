// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
  /*电梯 */
  public static class ElevatorConstants {
    public static final int ElevatorMotorIdLeft = 11;
    public static final int ElevatorMotorIdRight = 13;

    public static final CurrentLimitsConfigs Amperelimit =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(80)
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimitEnable(true);
    public static final TalonFXConfiguration configs =
        new TalonFXConfiguration()
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(80)
                    .withMotionMagicAcceleration(130))
            /*电梯的PID */
            .withSlot0(
                new Slot0Configs()
                    .withKP(3)
                    .withKI(0.05)
                    .withKD(0)
                    .withKS(.04)
                    .withKV(.128)
                    .withKA(.0038)
                    .withKG(.39));

    public static final double ELEVATOR_POSITION_BOTTOM_ROTATIONS = .7; //
    public static final double ELEVATOR_POSITION_L1_ROTATIONS = 15; //
    public static final double ELEVATOR_POSITION_L2_ROTATIONS = 14.88; //
    public static final double ELEVATOR_POSITION_L3_ROTATIONS = 24.90; //
    public static final double ELEVATOR_POSITION_L4_ROTATIONS = 42.00; //
    public static final double ELEVATOR_POSITION_Max_ROTATIONS = 44.0; //
    public static final double ELEVATOR_POSITION_ALGEA_HIGH = 22.0; //
    public static final double ELEVATOR_POSITION_ALGEA_LOW = 13.0; //

    public static final double ELEVATOR_POSITION_TOLERANCE_ROTATIONS = .02;
  }
  /*地面抓取 */
  public static class GroundIntakeConstants {
    public static final int GroundIntakeKrakenId = 14;
    public static final int GroundIntakeFalconId = 15;

    public static final int gEncoderId = 7;

    public static final double ENCODER_TO_INTAKE_GEAR_RATIO = 2.0;

    public static final double kP = 3.5;
    public static final double kI = 0;
    public static final double kD = 0.01;

    public static final double kPositionToleranceCycles = 0.0005;

    public static final double INTAKE_ANGLE_PREPARE = .30; // 预备位置
    public static final double INTAKE_ANGLE_GROUND = .86; // 地面
    public static final double INTAKE_ANGLE_GRAB = .45; // L1
    public static final double INTAKE_ANGLE_L1 = .45; // L1
    public static final double INTAKE_ANGLE_DRIVERSTATION = .55;

    public static final double WHEEL_SPEED_OUT = .6; // 轮速，正反转
    public static final double WHEEL_SPEED_IN = .3; //
    public static final double WHEEL_SPEED_OUT_CORAL = -.3; //

    public static final double Maxspeed = .15;

    public static final CurrentLimitsConfigs Amperelimit =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(80)
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimitEnable(true);
    public static final TalonFXConfiguration configs = new TalonFXConfiguration();

    public static final Slot0Configs PID_CONFIGS =
        new Slot0Configs().withKP(20).withKI(0.0).withKD(0.0);
    public static final TalonFXConfiguration SBhumconfigs =
        new TalonFXConfiguration().withSlot0(PID_CONFIGS);
  }
  /*前进料器 */
  public static class FeederConstants {
    public static final int FeederTurnerMotorID = 16;
    public static final int FeederWheelMotorId = 17;

    public static final int fEncoderId = 6;

    public static final double ENCODER_TO_INTAKE_GEAR_RATIO = 2.0;

    public static final double kP = 2.7;
    public static final double kI = 0.05;
    public static final double kD = 0.00;

    public static final double kPositionToleranceCycles = 0.003;

    public static final double INTAKE_ANGLE_PREPARE = .77; // 预备位置
    public static final double INTAKE_ANGLE_SHOOT = .55; //
    public static final double INTAKE_ANGLE_SHOOT_L4 = .43; //
    public static final double INTAKE_ANGLE_SHOOT_L1 = .51; //
    public static final double INTAKE_ANGLE_ALGAE = .42; // 海藻

    public static final double WHEEL_SPEED_IN = 800; // 进
    public static final double WHEEL_SPEED_OUT = -3000; // 出
    public static final double Wheel_speed_in = .2; // 进
    public static final double Wheel_speed_out = .2; // 出
    public static final double Wheel_speed_out_Algea = -.5; // 出
    public static final double Wheel_speed_in_Algea = .3; // 出

    /*最大速度 */
    public static final double Maxspeed = -.0; // .15

    public static final CurrentLimitsConfigs Amperelimit =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(80)
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimitEnable(true);
    public static final TalonFXConfiguration configs = new TalonFXConfiguration();
    /*feeder轮子的 */
    public static final Slot0Configs PID_WHEELCONFIGS =
        new Slot0Configs().withKP(2).withKI(0.0).withKD(0.00);
    public static final TalonFXConfiguration WheelConfigs =
        new TalonFXConfiguration().withSlot0(PID_WHEELCONFIGS);
  }

  /*激光传感器参数 */
  public static class LaserSensorConstants {
    public static double OBJECT_DETECT_THRESHOLD_MM = .1;
    public static final int LASER_CAN_ID = 20;
  }
  /*攀爬 */
  public static class ClimberConstants {
    public static final int ClimberMotorID = 19;

    public static final CurrentLimitsConfigs Amperelimit =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(80)
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimitEnable(true);

    public static final TalonFXConfiguration configs =
        new TalonFXConfiguration()
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(40)
                    .withMotionMagicAcceleration(40))
            .withSlot0(new Slot0Configs().withKP(1.3).withKI(0).withKD(0.1).withKS(0))
            .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(66));

    public static TalonFXConfiguration amperageLimit;
  }
  /*中间转的轮子 */
  public static class CollectorConstants {
    public static final int intakecollectorMidID = 18;
    public static final int intakecollectorTopID = 19;
  }

  /*视觉 */
  public static final double ALIGN_X_P = 0.5; // P gain for forward/backward movement (meters)
  public static final double ALIGN_Y_P = 0.5; // P gain for left/right movement (meters)
  public static final double ALIGN_ROT_P = 0.5; // P gain for rotation (radians)

  // Alignment setpoints (desired relative position to the target)
  // These values represent the desired translation (x, y) and rotation (z)
  // of the robot relative to the AprilTag target.
  // Adjust these based on your robot's physical design and scoring mechanism.
  // X_SETPOINT_ALIGN: Desired forward/backward distance from the target (meters).
  //                   A positive value means the target is in front of the robot.
  public static final double ALIGN_X_SETPOINT =
      0.0; // Example: 0.0 meters forward/backward from target
  // Y_SETPOINT_ALIGN: Desired left/right distance from the target (meters).
  //                   A positive value means the target is to the right of the robot.
  //                   This might change based on left/right scoring side.
  public static final double ALIGN_Y_SETPOINT_RIGHT_SCORE =
      -0.5; // Example: 0.5 meters to the left of target for right score
  public static final double ALIGN_Y_SETPOINT_LEFT_SCORE =
      0.5; // Example: 0.5 meters to the right of target for left score
  // ROT_SETPOINT_ALIGN: Desired rotation of the robot relative to the target (radians).
  //                     0.0 means directly facing the target.
  public static final double ALIGN_ROT_SETPOINT =
      0.0; // Example: 0.0 radians (facing target directly)

  // Alignment tolerances (how close the robot needs to be to be considered "aligned")
  public static final double ALIGN_X_TOLERANCE = 0.05; // meters
  public static final double ALIGN_Y_TOLERANCE = 0.05; // meters
  public static final double ALIGN_ROT_TOLERANCE =
      Units.degreesToRadians(2.0); // 2 degrees in radians

  // Timers for alignment stability
  public static final double ALIGN_DONT_SEE_TAG_WAIT_TIME =
      0.5; // seconds (how long to wait if no tag is seen)
  public static final double ALIGN_STABILITY_TIME =
      0.3; // seconds (how long robot must be at setpoint to be finished)

  // Other constants you might need (from your existing VisionConstants)
  // 例如，从 VisionConstants 导入 AprilTagFieldLayout
  // public static final AprilTagFieldLayout APRIL_TAG_LAYOUT =
  // AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  public static final double X_REEF_ALIGNMENT_P = 3.3;
  public static final double Y_REEF_ALIGNMENT_P = 3.3;
  public static final double ROT_REEF_ALIGNMENT_P = 0.058;

  public static final double ROT_SETPOINT_REEF_ALIGNMENT = 0; // Rotation
  public static final double ROT_TOLERANCE_REEF_ALIGNMENT = 1;
  public static final double X_SETPOINT_REEF_ALIGNMENT = -0.34; // Vertical pose
  public static final double X_TOLERANCE_REEF_ALIGNMENT = 0.02;
  public static final double Y_SETPOINT_REEF_ALIGNMENT = 0.16; // Horizontal pose
  public static final double Y_TOLERANCE_REEF_ALIGNMENT = 0.02;

  public static final double DONT_SEE_TAG_WAIT_TIME = 1;
  public static final double POSE_VALIDATION_TIME = 0.3;
}
