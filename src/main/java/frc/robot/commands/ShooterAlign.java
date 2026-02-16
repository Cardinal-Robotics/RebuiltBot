// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.*;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubstystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.Timer;

import org.littletonrobotics.junction.Logger;

import java.util.List;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShooterAlign extends Command {
  Timer m_timer = new Timer();
  SwerveSubsystem m_swerveSubsystem;
  ShooterSubstystem m_shooterSubsytem;
  private int targetID = -1;
  boolean m_finished = false;

  public ShooterAlign(SwerveSubsystem swerveSubsystem, ShooterSubstystem shooterSubsytem) {
    addRequirements(swerveSubsystem);

    m_swerveSubsystem = swerveSubsystem;
    m_shooterSubsytem = shooterSubsytem;
  }

  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.recordOutput("Shooter", 1);

    Pose2d targetPose = getShootPose();

    ChassisSpeeds speeds = m_swerveSubsystem.calculateAlignSpeeds(m_timer.get(), targetPose);
    m_swerveSubsystem.driveRelative(speeds);
  }

  public Pose2d getShootPose() {
    Pose2d robotPose = m_swerveSubsystem.getPose2d();

    Pose2d[] blueShootPoses = { // the five shooting positions
        new Pose2d(3.257, 7.308, new Rotation2d(Math.toRadians(-68))),
        new Pose2d(2.272, 5.616, new Rotation2d(Math.toRadians(-32.56))),
        new Pose2d(2.345, 4.119, new Rotation2d(Math.toRadians(-3.02))),
        new Pose2d(2.607, 2.313, new Rotation2d(Math.toRadians(41.8))),
        new Pose2d(3.257, 0.809, new Rotation2d(Math.toRadians(65)))
    };

    Pose2d[] redShootPoses = {
        new Pose2d(13.239, 7.209, Rotation2d.fromDegrees(246.2)),
        new Pose2d(14.284, 6.352, new Rotation2d(Math.toRadians(223.3))),
        new Pose2d(14.612, 4.048, new Rotation2d(Math.toRadians(-179.48))),
        new Pose2d(14.368, 2.125, new Rotation2d(Math.toRadians(142.58))),
        new Pose2d(13.249, 0.642, new Rotation2d(Math.toRadians(108.54)))
    };

    Pose2d[] shooterPoses = new Pose2d[] {};
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Red);
    if (alliance == Alliance.Red)
      shooterPoses = redShootPoses;
    else if (alliance == Alliance.Blue)
      shooterPoses = blueShootPoses;

    Pose2d closestPose = shooterPoses[0];
    double minDistance = Double.MAX_VALUE;

    for (Pose2d pose : shooterPoses) { // chooses the best of the 5 poses
      double dx = pose.getX() - robotPose.getX();
      double dy = pose.getY() - robotPose.getY();
      double distance = Math.hypot(dx, dy); // sqrt(dx^2 + dy^2)

      if (distance < minDistance) {
        minDistance = distance;
        closestPose = pose;
      }
    }

    Logger.recordOutput("ShootPose", closestPose);

    return closestPose;
  }

  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
