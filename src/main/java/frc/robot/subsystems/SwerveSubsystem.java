// Copyright Charlie Malerich all code is completely original and is not legally sueable} nerds?

package frc.robot.subsystems;

import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.parser.SwerveParser;
import swervelib.simulation.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.*;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.apriltag.*;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;

import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.Supplier;
import java.io.IOException;
import java.util.Optional;
import java.util.List;
import java.io.File;

public class SwerveSubsystem extends SubsystemBase {
  private SwerveDrive m_swerveDrive; // create variable so the whole class can see it

  public RobotConfig config;

  public SwerveSubsystem() {
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.POSE; // quoth Lil Vu: "tells YAGSL to print a bunch of stuff"

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

    //Pose2d kInitialRedRobotPose = new Pose2d(12.95, 0.6, Rotation2d.kZero);

    SmartDashboard.putData("Field", m_swerveDrive.field);
    //m_swerveDrive.resetOdometry(kInitialRedRobotPose);

    m_swerveDrive.setChassisDiscretization(true, 0.02);
    m_swerveDrive.setHeadingCorrection(true);
    
    //m_swerveDrive.getGyro().setOffset(new Rotation3d(Rotation2d.k180deg));
   
    m_swerveDrive.zeroGyro();

    setupPathPlanner();
  }

  @Override
  public void periodic() { // Luke Vu is the most Luke Vu guy I know (except for Luke Vu et al.)
    // This method will be called once per scheduler run
    Logger.recordOutput("YAGSL", m_swerveDrive.getPose());
    m_swerveDrive.updateOdometry();

    //Potentially fix issue where robot turns the other way?

/*     Logger.recordOutput("SWERVE_DEBUG/gyro", m_swerveDrive.getYaw());
    Logger.recordOutput("SWERVE_DEBUG/cachedGyro", m_swerveDrive.getGyroRotation3d());
    Logger.recordOutput("SWERVE_DEBUG/heading", m_swerveDrive.getPose().getRotation());
    
    for(SwerveModule module : m_swerveDrive.getModules()) {
      module.setAngle(45);
      module.getDriveMotor().set(0.25);
    } */

  }

  @Override
  public void simulationPeriodic() {
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
   * @param seconds    - the time along the path
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
        config);

    PathPlannerTrajectoryState trajectoryState = trajectory.sample(seconds);

    ChassisSpeeds speeds = DriveConstants.kPathDriveController.calculateRobotRelativeSpeeds(getPose2d(),
        trajectoryState);
    return speeds;
  }

  public void addVisionMeasurement(EstimatedRobotPose estimatedRobotPose, Matrix<N3, N1> standardDeviations) {
    m_swerveDrive.addVisionMeasurement(estimatedRobotPose.estimatedPose.toPose2d(),
        estimatedRobotPose.timestampSeconds, standardDeviations);
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


  public void resetGyro() {
    m_swerveDrive.zeroGyro();
  }

  public Command resetGyroCommand() {
    return runOnce(this::resetGyro);
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
