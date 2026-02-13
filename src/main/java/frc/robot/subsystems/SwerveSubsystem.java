// Copyright Charlie Malerich all code is completely original and is not legally sueable} nerds?

package frc.robot.subsystems;


import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.apriltag.*;

import org.littletonrobotics.junction.Logger;

import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.Supplier;
import java.io.IOException;
import java.util.Optional;
import java.util.List;
import java.io.File;

public class SwerveSubsystem extends SubsystemBase {
  public int driveType;
  private SwerveDrive m_swerveDrive; // create variable so the whole class can see it
  
  public RobotConfig config;

  Optional<Pose3d> tag0 = fieldLayout.getTagPose(1);

  public static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(
      AprilTagFields.k2026RebuiltWelded);

  public SwerveSubsystem() {
    driveType = 0;

    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH; // quoth Lil Vu: "tells YAGSL to print a bunch of stuff"

    File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve"); // goes to filesystem, finds deploy
                                                                                    // folder, gets swerve folder, gets
                                                                                    // the JSON

    try {
      m_swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(DriveConstants.kMaxSpeed); // makes the
                                                                                                         // thing to
                                                                                                         // read the
                                                                                                         // JSON, makes
                                                                                                         // a new Swerve
                                                                                                         // drive with
                                                                                                         // the maxspeed
    } catch (IOException e) { // if we get this error then:
      throw new RuntimeException(e);
    }

    Pose2d kInitialRedRobotPose = new Pose2d(12.95, 0.6, Rotation2d.kZero);

    m_swerveDrive.field.setRobotPose(kInitialRedRobotPose);
    m_swerveDrive.resetOdometry(kInitialRedRobotPose);

    setupPathPlanner();
  }

  @Override
  public void periodic() { // Luke Vu is the most Luke Vu guy I know (except for Luke Vu et al.)
    // This method will be called once per scheduler run
    Pose2d pose = m_swerveDrive.getPose();
    Logger.recordOutput("YAGSL", pose);
  }

  @Override
  public void simulationPeriodic() {
    Pose2d simulationPose = m_swerveDrive.getSimulationDriveTrainPose().orElseGet(() -> Pose2d.kZero);
    m_swerveDrive.swerveDrivePoseEstimator.resetPose(simulationPose);
  }

  public void driveRelative(ChassisSpeeds velocity) { // driving relative to the robot

    m_swerveDrive.drive(velocity);
  }

  public Command driveRelative(Supplier<ChassisSpeeds> velocity) {
    return run(() -> {
      driveRelative(velocity.get()); // runs this function
    });
  }

  public void driveFieldOriented(ChassisSpeeds velocity) {

    m_swerveDrive.driveFieldOriented(velocity);

  }

  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
    return run(() -> {
      m_swerveDrive.driveFieldOriented(velocity.get());
    });
  }

  /**
   * @param seconds - the time along the path
   * @param targetPose - the target position
   * @returns {@link ChassisSpeeds} - Robot relative driving speeds.
   */
  public ChassisSpeeds calculateAlignSpeeds(double seconds, Pose2d targetPose) {
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
        getPose2d(),
        targetPose);

    PathConstraints constraints = new PathConstraints(
        3.0,
        3.0,
        2 * Math.PI,
        4 * Math.PI); // The constraints for this path.

    PathPlannerPath path = new PathPlannerPath(
        waypoints,
        constraints,
        new IdealStartingState(0, getPose2d().getRotation()),
        new GoalEndState(
            0.0,
            targetPose.getRotation()));

    // this is just charlie trying to do it so if its wrong delete it
    ChassisSpeeds startingSpeeds = getSwerveDrive().getRobotVelocity();
    Rotation2d startingRotation = getPose2d().getRotation();

    PathPlannerTrajectory trajectory = new PathPlannerTrajectory(
        path,
        startingSpeeds,
        startingRotation,
        config
    );

    PathPlannerTrajectoryState trajectoryState = trajectory.sample(seconds);

    ChassisSpeeds speeds = DriveConstants.kPathDriveController.calculateRobotRelativeSpeeds(getPose2d(), trajectoryState);
    return speeds;
  }

  public void toggleDriveType() { // there you are hahahah
    driveType = (driveType == 0) ? 1 : 0;
    System.out.println("Drive type toggled to " + (driveType == 0 ? "FIELD" : "RELATIVE"));
  }

  public SwerveDrive getSwerveDrive() {
    return m_swerveDrive;
  }

  public Pose2d getPose2d() {
    return m_swerveDrive.getPose();
  }

  public ChassisSpeeds getFieldVelocity() {
    return m_swerveDrive.getFieldVelocity();
  }

  public void setupPathPlanner() {
    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    try {
      config = RobotConfig.fromGUISettings();

      System.out.println("Autobuilder Configured");

      AutoBuilder.configure(
          m_swerveDrive::getPose,
          m_swerveDrive::resetOdometry,
          m_swerveDrive::getRobotVelocity,
          (speedsRobotRelative, moduleFeedForwards) -> {
            m_swerveDrive.drive(
                speedsRobotRelative,
                m_swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                moduleFeedForwards.linearForces());
          },
          DriveConstants.kPathDriveController, // Rotation PID
          config,
          () -> {
            Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
            return alliance == DriverStation.Alliance.Red;
          },
          this);

    } catch (Exception e) {
      e.printStackTrace();
    }
  }

}
