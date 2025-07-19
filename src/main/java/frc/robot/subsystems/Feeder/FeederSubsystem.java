// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Feeder;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FeederConstants;

public class FeederSubsystem extends SubsystemBase {
  /** Creates a new FeederSubsystem. */
  private TalonFX FeederTurnerMotor = new TalonFX(FeederConstants.FeederTurnerMotorID);

  private TalonFX FeederWheelMotor = new TalonFX(FeederConstants.FeederWheelMotorId);
  private VelocityVoltage WheelMotorVRequest;

  private DutyCycleEncoder Gencoder;
  private PIDController pidController;
  private VoltageOut voltageRequest = new VoltageOut(0);

  private double targetAngleDegrees = FeederConstants.INTAKE_ANGLE_PREPARE;

  private final double physicalZeroOffsetCycles = 0; // 注意改

  public FeederSubsystem() {
    FeederConstants.configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    FeederTurnerMotor.getConfigurator().apply(FeederConstants.configs);
    FeederTurnerMotor.getConfigurator().apply(FeederConstants.Amperelimit);

    WheelMotorVRequest = new VelocityVoltage(0);

    Gencoder = new DutyCycleEncoder(FeederConstants.fEncoderId);

    pidController = new PIDController(FeederConstants.kP, FeederConstants.kI, FeederConstants.kD);
    pidController.setTolerance(FeederConstants.kPositionToleranceCycles);
  }

  public void setFeederTargetAngle(double angleDegrees) {
    this.targetAngleDegrees = angleDegrees;
  }

  public double getCurrentFeederAngleDegrees() {
    return Gencoder.get() - physicalZeroOffsetCycles;
  }

  public boolean isAtTargetAngle() {
    return pidController.atSetpoint();
  }

  public void runWheelMotor(double RPM) {
    FeederWheelMotor.setControl(WheelMotorVRequest.withVelocity(RPM / 60.0));
  } // <--- 这里加上了 runWheelMotor 方法的结束大括号

  public double getRPM() { //
    // 从轮子电机对象获取当前速度
    // Phoenix 6 的 getVelocity() 方法通常返回 RPS (Rotations Per Second)
    double actualRPS = FeederWheelMotor.getVelocity().getValueAsDouble();

    // 将 RPS 转换为 RPM (每秒转数 * 60 = 每分钟转数)
    // 只能接受RPS
    return actualRPS * 60.0;
  }

  public void runFeederWheel(double speed) {
    FeederWheelMotor.set(speed);
  }

  public void stopMotor() {
    FeederTurnerMotor.set(0);
  }

  public void stopWheelMotor() {
    FeederWheelMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double currentAngle = getCurrentFeederAngleDegrees();
    double pidOutput = pidController.calculate(currentAngle, targetAngleDegrees);
    double motorVoltage = pidOutput * 12.0 * FeederConstants.Maxspeed;
    SmartDashboard.putNumber("fTarget Angle", targetAngleDegrees);
    SmartDashboard.putNumber("fCurrent Angle (Deg)", currentAngle);
    //// SmartDashboard.putNumber("Intake Angle Error (Deg)", pidController.getPositionError());
    // SmartDashboard.putNumber("Intake PID Output", pidOutput); // 这个值现在是钳制后的
    SmartDashboard.putBoolean("fAt Target", isAtTargetAngle());
    //// SmartDashboard.putNumber("Encoder Raw Value (0-1)", getRawEncoderValue());
    SmartDashboard.putNumber("fMotor Output (V)", motorVoltage);
    SmartDashboard.putNumber("fIntake", Gencoder.get());

    SmartDashboard.putNumber("FeederWheelMotor Output ", FeederWheelMotor.get()); // 这个是轮子电机的
    SmartDashboard.putNumber("WHEEL RPM ", getRPM());
    FeederTurnerMotor.setControl(voltageRequest.withOutput(motorVoltage));
  }
}
