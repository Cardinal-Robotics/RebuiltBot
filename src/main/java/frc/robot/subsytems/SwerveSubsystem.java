// Copyright Charlie Malerich all code is completely original and is not legally sueable} nerds?

package frc.robot.subsytems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.File;
import java.io.IOException;
import java.io.ObjectInputFilter.Config;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import swervelib.parser.Cache;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import swervelib.SwerveDrive;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Pose2d;

import java.util.List;
import java.util.Optional;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsytems.SwerveSubsystem;
import frc.robot.subsytems.VisionSubsystem;

import frc.robot.Constants.DriveConstants;

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

    SmartDashboard.putData(m_swerveDrive.field);
    SmartDashboard.putNumber("x", 0);
    SmartDashboard.putNumber("y", 0);
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

  public void toggleDriveType() { //there you are hahahah
    driveType = (driveType == 0) ? 1 : 0;
    System.out.println("Drive type toggled to " + (driveType == 0 ? "FIELD" : "RELATIVE"));
  }

  public SwerveDrive getSwerveDrive() {
    return m_swerveDrive;
  }

  public Pose2d getPose2d() {
    return m_swerveDrive.getPose();
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
