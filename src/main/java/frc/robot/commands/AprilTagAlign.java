// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

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

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AprilTagAlign extends Command {
  Timer m_timer = new Timer();
  SwerveSubsystem m_swerveSubsystem;
  VisionSubsystem m_visionSubsystem;
  private int targetID = -1;
  boolean m_finished = false;

  public AprilTagAlign(SwerveSubsystem swerveSubsystem, VisionSubsystem visionSubsystem) {
    addRequirements(swerveSubsystem);

    m_swerveSubsystem = swerveSubsystem;
    m_visionSubsystem = visionSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.start();
    targetID = m_visionSubsystem.getBestTargetId();
    if(targetID == -1) m_finished = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // get a function that gets the ID of the best tag
    Logger.recordOutput("ID", targetID);

    // this function gets the offset based off of the ID we get
    if (targetID == -1) return; 

    System.out.println(targetID);

    Pose2d targetPose = m_swerveSubsystem.fieldLayout.getTagPose(targetID).get().toPose2d()
    .plus(getApriltagOffset(targetID)); // love hardcoding
    
    driveToAprilTag(m_timer.get(), targetPose);
  }

  public Transform2d getApriltagOffset(int ID) {
    Transform2d offset = new Transform2d(0, 0, Rotation2d.kZero);
    int targetID = ID;
    switch(targetID) {
  
    }
    return offset;
  }

  public void driveToAprilTag(double seconds, Pose2d targetPose) {
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
      m_swerveSubsystem.getPose2d(),
      targetPose
    );

  PathConstraints constraints = new PathConstraints(
    3.0,
    3.0,
    2 * Math.PI,
    4 * Math.PI); // The constraints for this path.

    PathPlannerPath path = new PathPlannerPath(
      waypoints,
      constraints,
      new IdealStartingState(0, m_swerveSubsystem.getPose2d().getRotation()),
      new GoalEndState(
        0.0,
         targetPose.getRotation().plus(new Rotation2d(Math.PI)))
    );

    PPHolonomicDriveController controller = 
      new PPHolonomicDriveController(
        new PIDConstants(1, 0, 0),
        new PIDConstants(1, 0, 0)
        );

    // this is just charlie trying to do it so if its wrong delete it
    ChassisSpeeds startingSpeeds = m_swerveSubsystem.getSwerveDrive().getRobotVelocity();
    Rotation2d startingRotation = m_swerveSubsystem.getPose2d().getRotation();

    PathPlannerTrajectory trajectory = new PathPlannerTrajectory(
      path,
      startingSpeeds,
      startingRotation,
      m_swerveSubsystem.config
      // ??? we need some sort of robot config here but I am not completely sure how to do that
    );

    PathPlannerTrajectoryState trajectoryState = trajectory.sample(seconds);

    ChassisSpeeds speeds = controller.calculateRobotRelativeSpeeds(m_swerveSubsystem.getPose2d(), trajectoryState);
    m_swerveSubsystem.driveRelative(speeds);
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
