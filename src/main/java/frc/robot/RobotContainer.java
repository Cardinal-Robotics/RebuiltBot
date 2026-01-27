package frc.robot;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import swervelib.SwerveInputStream;

import frc.robot.subsytems.*;
import frc.robot.Constants.*;
import frc.robot.commands.*;

public class RobotContainer {

  // SUBSYSTEMS
  // -----------------------------------------------------------------------------------
  private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem(); // awesome
  private final ShooterSubstystem m_shooterSubsystem = new ShooterSubstystem(); // awesome
  private final SimulationSubsystem m_smulationSubsystem = new SimulationSubsystem();
  private final VisionSubsystem m_visionSubsystem = new VisionSubsystem(m_swerveSubsystem);
  // -----------------------------------------------------------------------------------

  // COMMANDS
  // -----------------------------------------------------------------------------------

  // -----------------------------------------------------------------------------------

  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);

  public RobotContainer() {
    configureBindings();
  }

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(m_swerveSubsystem.getSwerveDrive(),
      () -> m_driverController.getLeftY() * -1,
      () -> m_driverController.getLeftX() * -1)
      .headingWhile(true)
      .deadband(OperatorConstants.kDEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);

  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
      .withControllerHeadingAxis(
          () -> m_driverController.getRightX()
              * (DriverStation.getAlliance().orElse(Alliance.Red).equals(Alliance.Blue) ? -1 : 1)
              * (SmartDashboard.getBoolean("Invert Rotation", false) ? -1 : 1),
          () -> m_driverController.getRightY()
              * (DriverStation.getAlliance().orElse(Alliance.Red).equals(Alliance.Blue) ? -1 : 1)
              * (SmartDashboard.getBoolean("Invert Rotation", false) ? -1 : 1))
      .headingWhile(true);

  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
      .allianceRelativeControl(false)
      .withControllerRotationAxis(() -> m_driverController.getRightX() * -1);

  Command driveFieldOritentedDirectAngle = m_swerveSubsystem.driveFieldOriented(driveDirectAngle);

  Command driveFieldOrientedAngularVelocity = m_swerveSubsystem.driveFieldOriented(driveAngularVelocity);

  Command driveRobotOrientedAngularVelocity = m_swerveSubsystem.driveFieldOriented(driveRobotOriented);

  Command driveAutoAlign = new AprilTagAlign(m_swerveSubsystem, m_visionSubsystem);
  Command driveShooterAlign = new ShooterAlign(m_swerveSubsystem, m_visionSubsystem);
  Command shootyBoi = new Shoot(m_shooterSubsystem, m_swerveSubsystem);

  private void configureBindings() {
    m_swerveSubsystem.setDefaultCommand(driveFieldOritentedDirectAngle);

    m_driverController.a().toggleOnTrue(driveRobotOrientedAngularVelocity
        .beforeStarting(() -> {
          Logger.recordOutput("Robot Relative", true);
        }).finallyDo(() -> {
          Logger.recordOutput("Robot Relative", false);
        }));

    m_driverController.y().toggleOnTrue(driveAutoAlign);
    m_driverController.b().toggleOnTrue(driveShooterAlign); // temporary button
    m_driverController.rightTrigger().toggleOnTrue(shootyBoi);

    /*
     * m_driverController.x().onTrue(Commands.runOnce(() -> {
     * m_swerveSubsystem.toggleDriveType();
     * 
     * System.out.println(m_swerveSubsystem.getDefaultCommand());
     * 
     * if (m_swerveSubsystem.driveType == 0) {
     * m_swerveSubsystem.setDefaultCommand(driveFieldOrientedAngularVelocity);
     * System.out.println("Drive mode: FIELD ORIENTED");
     * } else {
     * m_swerveSubsystem.setDefaultCommand(driveRobotOrientedAngularVelocity);
     * System.out.println("Drive mode: ROBOT RELATIVE");
     * }
     * }));
     */
  }

  public Command getAutonomousCommand() {
    return new PathPlannerAuto("Lucas' Auto");
  }
}
