package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.math.geometry.Pose2d;

import com.pathplanner.lib.commands.PathPlannerAuto;

import org.littletonrobotics.junction.Logger;

import swervelib.SwerveInputStream;
import frc.robot.Constants.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.vision.VisionSubsystem;

public class RobotContainer {

  // SUBSYSTEMS
  // -----------------------------------------------------------------------------------
  private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem(); // awesome
  private final ShooterSubstystem m_shooterSubsystem = new ShooterSubstystem(m_swerveSubsystem); // awesome
  private final SimulationSubsystem m_simulationSubsystem = new SimulationSubsystem();
  private final VisionSubsystem m_visionSubsystem = new VisionSubsystem(m_swerveSubsystem);
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem(m_swerveSubsystem);
  private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
  // -----------------------------------------------------------------------------------

  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);

  public RobotContainer() {
    configureBindings();
  }

  // COMMANDS
  // -----------------------------------------------------------------------------------

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
              * (SmartDashboard.getBoolean("Invert Rotation", false) ? -1 : 1));

    SwerveInputStream driveHubLocked = driveAngularVelocity.copy()
      .withControllerHeadingAxis(
         () -> Math.sin(m_shooterSubsystem.getIdealShooterConditions()[1]),
         () -> Math.cos(m_shooterSubsystem.getIdealShooterConditions()[1]))
      .headingWhile(true);

  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
      .allianceRelativeControl(false)
      .withControllerRotationAxis(() -> m_driverController.getRightX() * -1);

      

  Command driveFieldOritentedDirectAngle = m_swerveSubsystem.driveFieldOriented(driveDirectAngle);

  Command driveFieldOrientedAngularVelocity = m_swerveSubsystem.driveFieldOriented(driveAngularVelocity);

  Command driveRobotOrientedAngularVelocity = m_swerveSubsystem.driveFieldOriented(driveRobotOriented);
  Command driveRobotOrientedHubLocked = m_swerveSubsystem.driveFieldOriented(driveHubLocked);

  SwerveInputStream driveAimRedHub = driveDirectAngle.copy().aim(new Pose2d(12.1, 4.03, Rotation2d.kZero)).aimWhile(true);
  SwerveInputStream driveAimBlueHub = driveDirectAngle.copy().aim(new Pose2d(4.5, 4.03, Rotation2d.kZero)).aimWhile(true);

  Command driveFieldOrientedRedHub = m_swerveSubsystem.driveFieldOriented(driveAimRedHub);

  Command driveFieldOrientedBlueHub = m_swerveSubsystem.driveFieldOriented(driveAimBlueHub);

  Command driveAutoAlign = new AprilTagAlign(m_swerveSubsystem, m_visionSubsystem);
  Command driveShooterAlign = new ShooterAlign(m_swerveSubsystem, m_visionSubsystem);
  Command shootyBoi = new Shoot(m_shooterSubsystem, m_swerveSubsystem);

  Command riseCommand = m_climberSubsystem.riseCommand();
  Command descendCommand = m_climberSubsystem.descendCommand();

  // -----------------------------------------------------------------------------------

  private void configureBindings() {
    m_swerveSubsystem.setDefaultCommand(driveFieldOritentedDirectAngle);

    m_driverController.a().toggleOnTrue(driveRobotOrientedAngularVelocity
        .beforeStarting(() -> {
          Logger.recordOutput("Robot Relative", true);
        }).finallyDo(() -> {
          Logger.recordOutput("Robot Relative", false);
        }));
    m_driverController.x().toggleOnTrue(driveRobotOrientedHubLocked);

    m_driverController.y().toggleOnTrue(driveAutoAlign);
    m_driverController.b().toggleOnTrue(driveShooterAlign); // temporary button
    m_driverController.rightTrigger().toggleOnTrue(shootyBoi);
    m_driverController.povLeft().whileTrue(m_intakeSubsystem.setIntakePivotCommand(90));
    m_driverController.povRight().whileTrue(m_intakeSubsystem.setIntakePivotCommand(0));
    m_driverController.povUp().whileTrue(descendCommand);
    m_driverController.povDown().whileTrue(riseCommand);

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
