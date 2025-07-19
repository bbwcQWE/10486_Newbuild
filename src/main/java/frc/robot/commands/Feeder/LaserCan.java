// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Feeder;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder.LaserCanSubsystem;
import frc.robot.subsystems.Feeder.i_LaserCanSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LaserCan extends Command {
  /** Creates a new LaserCan. */
  private final LaserCanSubsystem laserCanSubsystem;

  private final i_LaserCanSubsystem i_laserCanSubsystem;

  public LaserCan(LaserCanSubsystem laserCanSubsystem, i_LaserCanSubsystem i_lasCanSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.laserCanSubsystem = laserCanSubsystem;
    this.i_laserCanSubsystem = i_lasCanSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return laserCanSubsystem.isTargetClose() && !i_laserCanSubsystem.isTargetClose();
  }
}
