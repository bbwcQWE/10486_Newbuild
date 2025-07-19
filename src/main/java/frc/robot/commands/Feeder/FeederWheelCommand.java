// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Feeder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder.FeederSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FeederWheelCommand extends Command {
  /** Creates a new FeederWheelCommand. */
  private FeederSubsystem feederSubsystem;

  private double fWheelSpeed;

  // private final double RPM = 1000;

  public FeederWheelCommand(FeederSubsystem feederSubsystem, double fWheelSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.feederSubsystem = feederSubsystem;
    this.fWheelSpeed = fWheelSpeed;
    addRequirements(this.feederSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    feederSubsystem.runWheelMotor(fWheelSpeed);
    SmartDashboard.putString("FeederWheel Current State", "IMWARD");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    feederSubsystem.stopWheelMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
