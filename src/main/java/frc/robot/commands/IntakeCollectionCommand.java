// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.GroundIntake.IntakeCollectionSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeCollectionCommand extends Command {
  private IntakeCollectionSubsystem intakeCollection;
  /** Creates a new IntakeCollectionCommand. */
  public IntakeCollectionCommand(IntakeCollectionSubsystem intakeCollection) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intakeCollection = intakeCollection;
    addRequirements(this.intakeCollection);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeCollection.runIntake(-0.5, -0.8);
    SmartDashboard.putString("IntakeCollector Current State", "IMWARD");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeCollection.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
