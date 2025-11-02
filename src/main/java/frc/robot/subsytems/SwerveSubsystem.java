// Copyright Charlie Malerich all code is completely original and is not legally sueable} nerds?

package frc.robot.subsytems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.File;
import java.io.IOException;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;

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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Pose2d;

import frc.robot.Constants.DriveConstants;

public class SwerveSubsystem extends SubsystemBase {
    private SwerveDrive m_swerveDrive; // create variable so the whole class can see it
  
  public SwerveSubsystem() {
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH; // quoth Lil Vu: "tells YAGSL to print a bunch of stuff"

    File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve"); // goes to filesystem, finds deploy folder, gets swerve folder, gets the JSON
  
    
    try { 
      m_swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(DriveConstants.kMaxSpeed); // makes the thing to read the JSON, makes a new Swerve drive with the maxspeed
    } catch (IOException e) { // if we get this error then:
      throw new RuntimeException(e);
    }
    
    Pose2d kInitialBlueRobotPose = new Pose2d(7.469, 7.457, Rotation2d.k180deg);
    
    m_swerveDrive.field.setRobotPose(kInitialBlueRobotPose);
    m_swerveDrive.resetOdometry(kInitialBlueRobotPose);

    setupPathPlanner();

    SmartDashboard.putData(m_swerveDrive.field);

  }

  @Override
  public void periodic() { // Luke Vu is the most Luke Vu guy I know (except for Luke Vu et al.)
    // This method will be called once per scheduler run
    Pose2d position = m_swerveDrive.getPose();
    Logger.recordOutput("YAGSL", position);
  
    
  }


  public void driveRelative(double velocity_x, double velocity_y, double rotation) { // driving relative to the robot
    ChassisSpeeds velocity = new ChassisSpeeds(velocity_x, velocity_y, Units.degreesToRadians(rotation)); // see luke vu's wonderful lecture
    Logger.recordOutput("Sigma", "67");
    m_swerveDrive.drive(velocity);
  }
  
  public Command driveRelativeCommand(Translation2d translation) {
    Logger.recordOutput("getName()", "null");
    return run(() -> {
      driveRelative(translation.getX(), translation.getY(), 0); // runs this function
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

  public SwerveDrive getSwerveDrive() {
    return m_swerveDrive;
  }

  public void setupPathPlanner() {
        // Load the RobotConfig from the GUI settings. You should probably
        // store this in your Constants file
        RobotConfig config;
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
