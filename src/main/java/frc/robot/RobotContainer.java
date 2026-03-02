package frc.robot;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AprilTagAlign;
import frc.robot.commands.AutoHubLock;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.HubLock;
import frc.robot.commands.Shoot;
import frc.robot.commands.ShooterAlign;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubstystem;
import frc.robot.subsystems.SimulationSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import swervelib.SwerveInputStream;

public class RobotContainer {

    // SUBSYSTEMS
    // -----------------------------------------------------------------------------------
    private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem(); // awesome
    private final ShooterSubstystem m_shooterSubsystem = new ShooterSubstystem(m_swerveSubsystem); // awesome
    private final SimulationSubsystem m_simulationSubsystem = new SimulationSubsystem();
    private final VisionSubsystem m_visionSubsystem = new VisionSubsystem(m_swerveSubsystem);
    private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem(m_swerveSubsystem);
    private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
    private final IndexerSubsystem m_indexerSubsystem = new IndexerSubsystem();
    // -----------------------------------------------------------------------------------

    private final SendableChooser<Command> m_autoChooser;

    private final CommandXboxController m_driverController = new CommandXboxController(
            OperatorConstants.kDriverControllerPort);

    public RobotContainer() {
        CameraServer.startAutomaticCapture();

        configureBindings();
        // Register Named commands first before building chooser...
        m_autoChooser = AutoBuilder.buildAutoChooser("Lucas' Auto");
        SmartDashboard.putData("Auto Chooser", m_autoChooser);
    }

    // COMMANDS
    // -----------------------------------------------------------------------------------

    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(m_swerveSubsystem.getSwerveDrive(),
            () -> m_driverController.getLeftY() * -1
                    * (DriverStation.getAlliance().orElse(Alliance.Red).equals(Alliance.Blue) ? 1 : -1)
                    * (SmartDashboard.getBoolean("Invert Translation", false) ? 1 : -1),
            () -> m_driverController.getLeftX() * -1
                    * (DriverStation.getAlliance().orElse(Alliance.Red).equals(Alliance.Blue) ? 1 : -1)
                    * (SmartDashboard.getBoolean("Invert Translation", false) ? 1 : -1))
            .deadband(OperatorConstants.kDEADBAND)
            .scaleTranslation(0.8)
            .allianceRelativeControl(false);

    SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
            .headingWhile(true)
            .withControllerHeadingAxis(
                    () -> m_driverController.getRightX()
                            * (DriverStation.getAlliance().orElse(Alliance.Red).equals(Alliance.Blue) ? 1 : -1)
                            * (SmartDashboard.getBoolean("Invert Rotation", false) ? 1 : -1),
                    () -> m_driverController.getRightY()
                            * (DriverStation.getAlliance().orElse(Alliance.Red).equals(Alliance.Blue) ? 1 : -1)
                            * (SmartDashboard.getBoolean("Invert Rotation", false) ? 1 : -1));

    SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
            .allianceRelativeControl(false)
            .withControllerRotationAxis(() -> m_driverController.getRightX() * -1);

    Command driveFieldOritentedDirectAngle = m_swerveSubsystem.driveFieldOriented(driveDirectAngle);
    Command driveRobotOrientedAngularVelocity = m_swerveSubsystem.driveRelative(driveRobotOriented);

    SwerveInputStream driveAimRedHub = driveDirectAngle.copy().aim(new Pose2d(12.1, 4.03, Rotation2d.kZero))
            .aimWhile(true);
    SwerveInputStream driveAimBlueHub = driveDirectAngle.copy().aim(new Pose2d(4.5, 4.03, Rotation2d.kZero))
            .aimWhile(true);

    Command driveFieldOrientedRedHub = m_swerveSubsystem.driveFieldOriented(driveAimRedHub);
    Command driveFieldOrientedBlueHub = m_swerveSubsystem.driveFieldOriented(driveAimBlueHub);

    Command driveRobotOrientedHubLocked = new HubLock(m_swerveSubsystem, m_shooterSubsystem, m_driverController);
    Command autoHubLock = new AutoHubLock(m_swerveSubsystem, m_shooterSubsystem, m_driverController);
    Command driveAutoAlign = new AprilTagAlign(m_swerveSubsystem, m_visionSubsystem);
    Command driveShooterAlign = new ShooterAlign(m_swerveSubsystem, m_shooterSubsystem);
    Command shootyBoi = new Shoot(m_shooterSubsystem, m_intakeSubsystem, m_indexerSubsystem);

    Command riseCommand = m_climberSubsystem.riseCommand();
    Command descendCommand = m_climberSubsystem.descendCommand();
    Command indexerCommand = m_indexerSubsystem.spinIndexerCommand();
    Command intakeCommand = m_intakeSubsystem.runIntakeMotor(.5); //temporary (vu postulate)
    Command stopIntakeCommand = m_intakeSubsystem.stopIntakeCommand();

    // -----------------------------------------------------------------------------------

    private void configureBindings() {
        SmartDashboard.putBoolean("Invert Translaiton", true);
        SmartDashboard.putBoolean("Invert Rotation", true);
        m_swerveSubsystem.setDefaultCommand(driveFieldOritentedDirectAngle);
        // m_swerveSubsystem.setDefaultCommand(driveRobotOrientedAngularVelocity);

        Logger.recordOutput("Robot Relative", false);
        /*
         * m_driverController.a().toggleOnTrue(driveRobotOrientedAngularVelocity
         * .beforeStarting(() -> {
         * Logger.recordOutput("Robot Relative", true);
         * }).finallyDo(() -> {
         * Logger.recordOutput("Robot Relative", false);
         * }));
         */

        // m_driverController.x().toggleOnTrue(driveRobotOrientedHubLocked);
        // m_driverController.y().toggleOnTrue(driveAutoAlign);
        // m_driverController.b().toggleOnTrue(driveShooterAlign); // temporary button -
        // great vu postulate
        m_driverController.y().whileTrue(m_swerveSubsystem.resetGyroCommand());
        m_driverController.povLeft().whileTrue(m_intakeSubsystem.setIntakePivotCommand(0));
        m_driverController.povRight().whileTrue(m_intakeSubsystem.setIntakePivotCommand(90));
        m_driverController.povUp().whileTrue(descendCommand);
        m_driverController.povDown().whileTrue(riseCommand);

        m_driverController.rightTrigger().whileTrue(shootyBoi);// .whileTrue(indexerCommand);
        m_driverController.leftTrigger().whileTrue(intakeCommand);// .whileTrue(indexerCommand);
        m_driverController.leftTrigger().whileFalse(stopIntakeCommand);
        // m_driverController.leftStick().whileTrue(SimulationSubsystem.resetFieldCommand());

        NamedCommands.registerCommand("Print",
                Commands.print(
                        "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA"));
        NamedCommands.registerCommand("Shoot", shootyBoi);
        NamedCommands.registerCommand("Intake Up", m_intakeSubsystem.setIntakePivotCommand(90));
        NamedCommands.registerCommand("Intake Down", m_intakeSubsystem.setIntakePivotCommand(0));
        NamedCommands.registerCommand("Intake Start", intakeCommand);
        NamedCommands.registerCommand("Run Indexer", indexerCommand);
        NamedCommands.registerCommand("Intake Stop", stopIntakeCommand);
        NamedCommands.registerCommand("Climber Rise", riseCommand);
        NamedCommands.registerCommand("Climber Descend", descendCommand);
        NamedCommands.registerCommand("Hub Lock", autoHubLock);
        NamedCommands.registerCommand("Shooter Align", driveShooterAlign);
        NamedCommands.registerCommand("Stop",
                Commands.runOnce(m_swerveSubsystem.getSwerveDrive()::lockPose, m_swerveSubsystem));
        NamedCommands.registerCommand("Shoot3s",
                new AutoShoot(m_shooterSubsystem, m_swerveSubsystem, m_intakeSubsystem, 3));
        NamedCommands.registerCommand("Shoot1s",
                new AutoShoot(m_shooterSubsystem, m_swerveSubsystem, m_intakeSubsystem, 1));
        NamedCommands.registerCommand("Shoot5s",
                new AutoShoot(m_shooterSubsystem, m_swerveSubsystem, m_intakeSubsystem, 5));

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
        return m_autoChooser.getSelected();
    }
}
