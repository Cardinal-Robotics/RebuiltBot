// Copyright Charlie Malerich all code is completely original and is not legally sueable} nerds?

package frc.robot.subsytems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.File;
import java.io.IOException;

import org.littletonrobotics.junction.Logger;

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
import edu.wpi.first.math.geometry.Pose2d;

public class SwerveSubsystem extends SubsystemBase {
    private SwerveDrive m_swerveDrive; // create variable so the whole class can see it
  
  public SwerveSubsystem() {
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH; // quoth Lil Vu: "tells YAGSL to print a bunch of stuff"

    double maximumSpeed = Units.feetToMeters(Integer.MAX_VALUE);
    File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve"); // goes to filesystem, finds deploy folder, gets swerve folder, gets the JSON
  
    
    try { 
      m_swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed); // makes the thing to read the JSON, makes a new Swerve drive with the maxspeed
    } catch (IOException e) { // if we get this error then:
      throw new RuntimeException(e);
    }
    
    Pose2d kInitialBlueRobotPose = new Pose2d(7.469, 7.457, Rotation2d.k180deg);
    
    m_swerveDrive.field.setRobotPose(kInitialBlueRobotPose);
    m_swerveDrive.resetOdometry(kInitialBlueRobotPose);

    
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
    m_swerveDrive.drive(velocity);
  }
  
  public void driveFieldOriented(double velocity_x, double velocity_y, double rotation) {

    ChassisSpeeds velocity = new ChassisSpeeds(velocity_x, velocity_y, Units.degreesToRadians(rotation));

    m_swerveDrive.driveFieldOriented(velocity);

  }

}
