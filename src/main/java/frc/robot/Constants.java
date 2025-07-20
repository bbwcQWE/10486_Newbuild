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
    public static final double ELEVATOR_POSITION_L1_ROTATIONS = 12; //
    public static final double ELEVATOR_POSITION_L2_ROTATIONS = 16.9; //
    public static final double ELEVATOR_POSITION_L3_ROTATIONS = 26.0; //
    public static final double ELEVATOR_POSITION_L4_ROTATIONS = 43.4; //
    public static final double ELEVATOR_POSITION_Max_ROTATIONS = 44.0; //
    public static final double ELEVATOR_POSITION_ALGEA_HIGH = 22.0; //
    public static final double ELEVATOR_POSITION_ALGEA_LOW = 12.0; //

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
    public static final double INTAKE_ANGLE_DRIVERSTATION = .55;

    public static final double WHEEL_SPEED_OUT = .3; // 轮速，正反转
    public static final double WHEEL_SPEED_IN = .1; //

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

    public static final int fEncoderId = 9;

    public static final double ENCODER_TO_INTAKE_GEAR_RATIO = 2.0;

    public static final double kP = 2.7;
    public static final double kI = 0.05;
    public static final double kD = 0.00;

    public static final double kPositionToleranceCycles = 0.003;

    public static final double INTAKE_ANGLE_PREPARE = .77; // 预备位置
    public static final double INTAKE_ANGLE_SHOOT = .55; //
    public static final double INTAKE_ANGLE_SHOOT_L4 = .34; //
    public static final double INTAKE_ANGLE_SHOOT_L1 = .51; //
    public static final double INTAKE_ANGLE_ALGAE = .42; // 海藻

    public static final double WHEEL_SPEED_IN = 800; // 进
    public static final double WHEEL_SPEED_OUT = -3000; // 出
    public static final double Wheel_speed_in = .2; // 进
    public static final double Wheel_speed_out = .2; // 出
    public static final double Wheel_speed_out_Algea = -.8; // 出
    public static final double Wheel_speed_in_Algea = .5; // 出

    /*最大速度 */
    public static final double Maxspeed = -.15; // .3

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
}
