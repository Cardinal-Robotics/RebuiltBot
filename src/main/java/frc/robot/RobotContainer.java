package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsytems.SimulationSubsystem;
import frc.robot.subsytems.SwerveSubsystem;
import frc.robot.subsytems.VisionSubsystem;
import swervelib.SwerveInputStream;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.DriveConstants;;

public class RobotContainer {

  // SUBSYSTEMS
  // -----------------------------------------------------------------------------------
  private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem(); // awesome
  private final SimulationSubsystem m_smulationSubsystem = new SimulationSubsystem();
  private final VisionSubsystem m_visionSubsystem = new VisionSubsystem(m_swerveSubsystem);
  // -----------------------------------------------------------------------------------

  // COMMANDS
  // -----------------------------------------------------------------------------------

  // -----------------------------------------------------------------------------------

  private final CommandXboxController m_driverController = 
    new CommandXboxController(OperatorConstants.kDriverControllerPort);

  public RobotContainer() {
    configureBindings();
    m_swerveSubsystem.setDefaultCommand(driveFieldOrientedAngularVelocity);
  }

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(m_swerveSubsystem.getSwerveDrive(), 
                                                                () -> m_driverController.getLeftY() * -1,
                                                                () -> m_driverController.getLeftX() * -1)
                                                                .withControllerRotationAxis(m_driverController::getRightX)
                                                                .deadband(OperatorConstants.kDEADBAND)
                                                                .scaleTranslation(0.8) // apparently can change to make robot go faster
                                                                .allianceRelativeControl(true);

  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(m_driverController::getRightX,
                                                                                             m_driverController::getRightY)
                                                                                             .headingWhile(true);

  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                                    .allianceRelativeControl(false)
                                                                    .withControllerRotationAxis(() -> m_driverController.getRightX() * -1);
                                
                                  

  Command driveFieldOritentedDirectAngle = m_swerveSubsystem.driveFieldOriented(driveDirectAngle);
  
  Command driveFieldOrientedAngularVelocity = m_swerveSubsystem.driveFieldOriented(driveAngularVelocity);
  
  Command driveRobotOrientedAngularVelocity  = m_swerveSubsystem.driveFieldOriented(driveRobotOriented);

  private void configureBindings() {
    m_driverController.a().toggleOnTrue(driveRobotOrientedAngularVelocity);

/*     m_driverController.x().onTrue(Commands.runOnce(() -> {
      m_swerveSubsystem.toggleDriveType();
  
      System.out.println(m_swerveSubsystem.getDefaultCommand());

      if (m_swerveSubsystem.driveType == 0) {
        m_swerveSubsystem.setDefaultCommand(driveFieldOrientedAngularVelocity);
        System.out.println("Drive mode: FIELD ORIENTED");
      } else {
        m_swerveSubsystem.setDefaultCommand(driveRobotOrientedAngularVelocity);
        System.out.println("Drive mode: ROBOT RELATIVE");
      }
    })); */
  }

  public Command getAutonomousCommand() {
    return new PathPlannerAuto("Lucas' Auto");
  }
}
