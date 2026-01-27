// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Rotation;

import java.util.List;
import java.util.Optional;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsytems.SwerveSubsystem;
import frc.robot.subsytems.VisionSubsystem;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShooterAlign extends Command {
  Timer m_timer = new Timer();
  SwerveSubsystem m_swerveSubsystem;
  VisionSubsystem m_visionSubsystem;
  private int targetID = -1;
  boolean m_finished = false;

  public ShooterAlign(SwerveSubsystem swerveSubsystem, VisionSubsystem visionSubsystem) {
    addRequirements(swerveSubsystem);

    m_swerveSubsystem = swerveSubsystem;
    m_visionSubsystem = visionSubsystem;
  }

  @Override
  public void initialize() {
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.recordOutput("Shooter", 1);

    Pose2d targetPose = getShootPose();

    driveToShootPose(m_timer.get(), targetPose);

  }

  public Pose2d getShootPose() {
    Pose2d robotPose = m_swerveSubsystem.getPose2d();

    Pose2d[] blueShootPoses = { // the five shooting positions
        new Pose2d(3.257, 7.308, new Rotation2d(Math.toRadians(-69.19 + 180))),
        new Pose2d(2.272, 5.616, new Rotation2d(Math.toRadians(-32.56 + 180))),
        new Pose2d(2.345, 4.119, new Rotation2d(Math.toRadians(-3.02 + 180))),
        new Pose2d(2.607, 2.313, new Rotation2d(Math.toRadians(41.8 + 180))),
        new Pose2d(3.257, 0.809, new Rotation2d(Math.toRadians(72.26 + 180)))
    };

    Pose2d[] redShootPoses = {
      new Pose2d(13.239, 7.209, Rotation2d.fromDegrees(-107.92 + 180)),
      new Pose2d(14.284, 6.352, new Rotation2d(Math.toRadians(215.52 + 180))),
      new Pose2d(14.612, 4.048, new Rotation2d(Math.toRadians(-179.48 + 180))),
      new Pose2d(14.368, 2.125, new Rotation2d(Math.toRadians(142.58 + 180))),
      new Pose2d(13.249, 0.642, new Rotation2d(Math.toRadians(108.54 + 180)))
    };

    Pose2d[] shooterPoses = new Pose2d[] {};
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Red);
    if(alliance == Alliance.Red) shooterPoses = redShootPoses;
    else if(alliance == Alliance.Blue) shooterPoses = blueShootPoses;

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

  public void driveToShootPose(double seconds, Pose2d targetPose) {
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
        m_swerveSubsystem.getPose2d(),
        targetPose);

    PathConstraints constraints = new PathConstraints(
        3.0,
        3.0,
        3.0 * 2 * Math.PI,
        3.0 * 2 * Math.PI); // The constraints for this path.

    PathPlannerPath path = new PathPlannerPath(
        waypoints,
        constraints,
        new IdealStartingState(0, m_swerveSubsystem.getPose2d().getRotation()),
        new GoalEndState(
            0.0,
            targetPose.getRotation()));

    PPHolonomicDriveController controller = new PPHolonomicDriveController(
        new PIDConstants(3, 0, 0),
        new PIDConstants(5, 0, 0));

    // this is just charlie trying to do it so if its wrong delete it
    ChassisSpeeds startingSpeeds = m_swerveSubsystem.getSwerveDrive().getRobotVelocity();
    Rotation2d startingRotation = m_swerveSubsystem.getPose2d().getRotation();

    PathPlannerTrajectory trajectory = new PathPlannerTrajectory(
        path,
        startingSpeeds,
        startingRotation,
        m_swerveSubsystem.config
    // ??? we need some sort of robot config here but I am not completely sure how
    // to do that
    );

    PathPlannerTrajectoryState trajectoryState = trajectory.sample(seconds);

    ChassisSpeeds speeds = controller.calculateRobotRelativeSpeeds(m_swerveSubsystem.getPose2d(), trajectoryState);
    m_swerveSubsystem.driveRelative(speeds);
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
