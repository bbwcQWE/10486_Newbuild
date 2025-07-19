// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.GroundIntakeConstants;

public class ClimberSubsystem extends SubsystemBase {
  private TalonFX ClimberMotor = new TalonFX(ClimberConstants.ClimberMotorID);
  // private VelocityVoltage ClimberMotorVRequest;
  private MotionMagicVoltage motionmagicRequest = new MotionMagicVoltage(0);
  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    ClimberMotor.getConfigurator().apply(ClimberConstants.configs);
    ClimberMotor.getConfigurator().refresh(ClimberConstants.Amperelimit);
    ClimberMotor.getConfigurator().apply(GroundIntakeConstants.Amperelimit);

    // ClimberMotorVRequest = new VelocityVoltage(0);
  }

  // public void runClimber(double RPM) {
  //  ClimberMotor.setControl(ClimberMotorVRequest.withVelocity(RPM / 60.0));
  // } // <--- 这里加上了 runWheelMotor 方法的结束大括号

  public double getRPM() { //
    // 从轮子电机对象获取当前速度
    // Phoenix 6 的 getVelocity() 方法通常返回 RPS (Rotations Per Second)
    double actualRPS = ClimberMotor.getVelocity().getValueAsDouble();

    // 将 RPS 转换为 RPM (每秒转数 * 60 = 每分钟转数)
    // 只能接受RPS
    return actualRPS * 60.0;
  }

  public void runClimber(double setpoint) {
    ClimberMotor.set(setpoint);
  }

  public void stopMotor() {
    ClimberMotor.set(0);
  }

  public void moveClimber(double Degree) {
    ClimberMotor.setControl(motionmagicRequest.withPosition(Units.degreesToRadians(Degree)));
  }

  public double getClimberPosition() {
    return Units.rotationsToDegrees(ClimberMotor.getPosition().getValueAsDouble());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("ClimberMotor Output ", ClimberMotor.get());
    // SmartDashboard.putNumber("ClimberMotor Position ", getClimberPosition());
  }
}
