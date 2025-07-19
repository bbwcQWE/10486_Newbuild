// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.GroundIntake;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CollectorConstants;

public class IntakeCollectionSubsystem extends SubsystemBase {
  private final WPI_TalonSRX intakecollectorMidMotor =
      new WPI_TalonSRX(CollectorConstants.intakecollectorMidID);
  private final TalonFX intakecollectorTopMotor =
      new TalonFX(CollectorConstants.intakecollectorTopID);

  /** Creates a new IntakeColtalection. */
  public IntakeCollectionSubsystem() {
    intakecollectorMidMotor.configFactoryDefault();
    intakecollectorMidMotor.setNeutralMode(NeutralMode.Coast);
  }

  public void runIntake(double speedMid, double speedTop) {
    intakecollectorMidMotor.set(speedMid);
    intakecollectorTopMotor.set(speedTop);
  }

  public void stop() {
    intakecollectorMidMotor.stopMotor();
    intakecollectorTopMotor.stopMotor();
  }

  @Override
  public void periodic() {}
}
