// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Feeder;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder.FeederWheelSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunFeederWheel extends Command {
  private FeederWheelSubsystem feederWheelSubsystem;
  private double Wheelspeed;
  /** Creates a new RunFeederWheel. */
  public RunFeederWheel(FeederWheelSubsystem feederSubsystem, double Wheelspeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.feederWheelSubsystem = feederSubsystem;
    this.Wheelspeed = Wheelspeed;
    addRequirements(this.feederWheelSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    feederWheelSubsystem.runFeederWheel(Wheelspeed);
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
