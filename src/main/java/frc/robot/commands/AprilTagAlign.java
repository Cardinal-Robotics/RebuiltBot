// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsytems.*;

import org.littletonrobotics.junction.Logger;

import java.util.List;

public class AprilTagAlign extends Command {
  Timer m_timer = new Timer();
  SwerveSubsystem m_swerveSubsystem;
  VisionSubsystem m_visionSubsystem;
  boolean m_finished = false;
  private int targetID = -1;

  public AprilTagAlign(SwerveSubsystem swerveSubsystem, VisionSubsystem visionSubsystem) {
    addRequirements(swerveSubsystem);

    m_swerveSubsystem = swerveSubsystem;
    m_visionSubsystem = visionSubsystem;
  }

  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
    targetID = m_visionSubsystem.getBestTargetId();
    if (targetID == -1)
      m_finished = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // get a function that gets the ID of the best tag
    Logger.recordOutput("ID", targetID);

    // this function gets the offset based off of the ID we get
    if (targetID == -1)
      return;

    Pose2d targetPose = getApriltagOffset(targetID); // love hardcoding

    ChassisSpeeds speeds = m_swerveSubsystem.calculateAlignSpeeds(m_timer.get(), targetPose);
    m_swerveSubsystem.driveRelative(speeds);
  }

  public Pose2d getApriltagOffset(int ID) { // IT IS NOT OFFSET ANYMORE ITS A GLOBAL POSITION
    Pose2d offset = new Pose2d(0, 0, Rotation2d.kZero);

    double x = SmartDashboard.getNumber("x", 0);
    double y = SmartDashboard.getNumber("y", 0);

    int targetID = ID;
    switch (targetID) {
      case 9:
      case 10:
        offset = new Pose2d(13.87, 4, Rotation2d.kZero);
        break;
      case 5:
      case 8:
        offset = new Pose2d(13.38, 2.36, new Rotation2d(Math.toRadians(140 + 180)));
        break;
      case 11:
      case 2:
        offset = new Pose2d(x, y, Rotation2d.kZero);
        break;

    }
    return offset;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_finished;
  }
}
