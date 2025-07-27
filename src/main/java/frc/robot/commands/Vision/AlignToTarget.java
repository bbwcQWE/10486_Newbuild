// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import java.util.List;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToTarget extends Command {
  /** Creates a new AlignToTarget. */
  private final Drive drive;

  private final Vision vision; // 添加 Vision 子系统引用
  // 目标AprilTag的ID

  // 定义机器人相对于AprilTag的目标姿态
  private final Transform2d robotToTagAlignTransform;

  // 实际的场地目标姿态，在execute中实时计算
  private Pose2d calculatedTargetPose;

  // PID 控制器，与之前相同
  private final PIDController xController = new PIDController(2.0, 0.0, 0.0);
  private final PIDController yController = new PIDController(2.0, 0.0, 0.0);
  private final PIDController rotationController = new PIDController(2.5, 0.0, 0.0);

  public AlignToTarget(Drive drive, Vision vision, Transform2d robotToTagAlignTransform) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    this.vision = vision;

    this.robotToTagAlignTransform = robotToTagAlignTransform;
    addRequirements(drive); // 命令需要驱动子系统

    rotationController.enableContinuousInput(-Math.PI, Math.PI);
    xController.setTolerance(0.05);
    yController.setTolerance(0.05);
    rotationController.setTolerance(Units.degreesToRadians(3.0));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    xController.reset();
    yController.reset();
    rotationController.reset();
    calculatedTargetPose = null;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    List<PoseObservation> tagObservations = vision.getLatestTagObservations();

    if (tagObservations.isEmpty()) {
      // 如果没有检测到任何 AprilTag，停止机器人
      drive.stop();
      calculatedTargetPose = null; // 确保目标姿态为空
      return;
    }

    // 获取机器人当前的姿态，用于计算距离
    Pose2d currentRobotPose = drive.getPose(); // 使用里程计融合后的姿态

    // 寻找最近的 AprilTag
    PoseObservation nearestTagObservation = null;
    double minDistanceSq = Double.POSITIVE_INFINITY; // 使用距离的平方避免开方运算

    for (PoseObservation obs : tagObservations) {
      // 确保 AprilTag 的姿态有效

    }

    if (nearestTagObservation == null) {
      // 理论上不会发生，除非所有观测的pose都为null
      drive.stop();
      calculatedTargetPose = null;
      return;
    }

    // 使用最近的 AprilTag 观测来计算目标姿态

    // 以下是你的现有 PID 逻辑，现在使用 calculatedTargetPose
    double xSpeed = xController.calculate(currentRobotPose.getX(), calculatedTargetPose.getX());
    double ySpeed = yController.calculate(currentRobotPose.getY(), calculatedTargetPose.getY());
    double rotationSpeed =
        rotationController.calculate(
            currentRobotPose.getRotation().getRadians(),
            calculatedTargetPose.getRotation().getRadians());

    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed, ySpeed, rotationSpeed, currentRobotPose.getRotation()));
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
