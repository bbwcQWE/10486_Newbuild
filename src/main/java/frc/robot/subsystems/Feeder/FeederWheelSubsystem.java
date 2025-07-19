// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Feeder;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FeederConstants;

public class FeederWheelSubsystem extends SubsystemBase {
  private TalonFX FeederWheelMotor = new TalonFX(FeederConstants.FeederWheelMotorId);
  /** Creates a new FeederWheelSubsystem. */
  public FeederWheelSubsystem() {
    FeederConstants.WheelConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    FeederWheelMotor.getConfigurator().apply(FeederConstants.WheelConfigs);
    FeederWheelMotor.getConfigurator().apply(FeederConstants.Amperelimit);
  }

  public void runFeederWheel(double speed) {
    FeederWheelMotor.set(speed);
  }

  public void stopWheelMotor() {
    FeederWheelMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
