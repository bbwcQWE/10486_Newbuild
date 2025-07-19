// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.GroundIntake;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GroundIntakeConstants;

public class GroundIntakeWheelSubsystem extends SubsystemBase {
  /** Creates a new GroundIntakeWheelSubsystem. */
  private TalonFX SBHum = new TalonFX(GroundIntakeConstants.GroundIntakeFalconId);

  private VelocityVoltage SBMotorVRequest;

  public GroundIntakeWheelSubsystem() {

    GroundIntakeConstants.SBhumconfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    SBHum.getConfigurator().apply(GroundIntakeConstants.SBhumconfigs);
    SBHum.getConfigurator().apply(GroundIntakeConstants.Amperelimit);

    SBMotorVRequest = new VelocityVoltage(0);
  }

  public void runIntakeVelocity(double RPM) {
    SBHum.setControl(SBMotorVRequest.withVelocity(RPM));
  }

  public void JustrunIntakeMotor(double Speed) {
    // 直接设置电机速度
    SBHum.set(Speed);
  }

  public double getRPM() {
    // 从 SBHum 电机对象获取当前速度
    // Phoenix 6 的 getVelocity() 方法通常返回 RPS (Rotations Per Second)
    double actualRPS = SBHum.getVelocity().getValueAsDouble();

    // 将 RPS 转换为 RPM (每秒转数 * 60 = 每分钟转数)
    // 只能接受RPS
    return actualRPS * 60.0;
  }

  public void stopSBHumMotor() {
    SBHum.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
