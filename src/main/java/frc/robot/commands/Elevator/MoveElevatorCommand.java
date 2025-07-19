// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveElevatorCommand extends Command {

  private ElevatorSubsystem elevatormove;
  private double targetRotations;
  /** Creates a new MoveElevatorCommand. */
  public MoveElevatorCommand(ElevatorSubsystem elevatormove, double targetRotations) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevatormove = elevatormove;
    this.targetRotations = targetRotations;
    addRequirements(this.elevatormove);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevatormove.moveElevator(targetRotations);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevatormove.stop(.15);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
