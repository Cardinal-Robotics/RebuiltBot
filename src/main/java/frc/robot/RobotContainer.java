package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.math.geometry.Pose2d;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.auto.AutoBuilder;

import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.SimulationSubsystem;
import frc.robot.subsystems.ShooterSubstystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import frc.robot.commands.ParallelHubLock;
import frc.robot.commands.AprilTagAlign;
import frc.robot.commands.ShooterAlign;
import frc.robot.commands.AutoHubLock;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.HubLock;
import frc.robot.commands.Shoot;

import org.littletonrobotics.junction.Logger;
import frc.robot.Constants.OperatorConstants;
import swervelib.SwerveInputStream;
import java.util.ArrayList;
import java.util.List;

public class RobotContainer {
    // SUBSYSTEMS
    // -----------------------------------------------------------------------------------
    private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem(); // awesome
    private final ShooterSubstystem m_shooterSubsystem = new ShooterSubstystem(m_swerveSubsystem); // awesome
    private final SimulationSubsystem m_simulationSubsystem = new SimulationSubsystem();
    private final VisionSubsystem m_visionSubsystem = new VisionSubsystem(m_swerveSubsystem);
    private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem(m_swerveSubsystem);
    private final IndexerSubsystem m_indexerSubsystem = new IndexerSubsystem();
    // -----------------------------------------------------------------------------------

    private final SendableChooser<Command> m_autoChooser;
    private final CommandXboxController m_driverController = new CommandXboxController(
            OperatorConstants.kDriverControllerPort);

    public RobotContainer() {
        // Turn on driver camera.
        CameraServer.startAutomaticCapture(1);

        // Register Named commands first before building chooser...
        configureBindings();
        m_autoChooser = AutoBuilder.buildAutoChooser("Lucas' Auto");
        SmartDashboard.putData("Auto Chooser", m_autoChooser);

        m_autoChooser.onChange((Command selectedCommand) -> {
            String commandName = selectedCommand.getName();
            try {
                List<PathPlannerPath> trajectory = PathPlannerAuto.getPathGroupFromAutoFile(commandName);
                List<Pose2d> poses = new ArrayList<>();

                for (PathPlannerPath path : trajectory) {
                    if (DriverStation.getAlliance().get().equals(Alliance.Red))
                        path = path.flipPath();

                    poses.addAll(path.getPathPoses());
                }

                m_swerveSubsystem.getSwerveDrive().field.getObject("trajectory").setPoses(poses);
            } catch (Exception exception) {}
        });
    }

    // COMMANDS
    // -----------------------------------------------------------------------------------

    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(m_swerveSubsystem.getSwerveDrive(),
            () -> m_driverController.getLeftY() * -1
                    * (DriverStation.getAlliance().orElse(Alliance.Red).equals(Alliance.Blue) ? 1
                            : -1)
                    * (Robot.isSimulation() ? -1 : -1)
                    * (SmartDashboard.getBoolean("Invert Translation", false) ? 1 : -1)
                    * (DriverStation.isAutonomous() ? 0 : 1),
            () -> m_driverController.getLeftX() * -1
                    * (DriverStation.getAlliance().orElse(Alliance.Red).equals(Alliance.Blue) ? 1
                            : -1)
                    * (Robot.isSimulation() ? -1 : -1)
                    * (SmartDashboard.getBoolean("Invert Translation", false) ? 1 : -1)
                    * (DriverStation.isAutonomous() ? 0 : 1))
            .deadband(OperatorConstants.kDEADBAND)
            .scaleTranslation(0.8)
            .allianceRelativeControl(false);

    SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
            .headingWhile(true)
            .withControllerHeadingAxis(
                    () -> m_driverController.getRightX()
                            * (DriverStation.getAlliance().orElse(Alliance.Red)
                                    .equals(Alliance.Blue) ? 1 : -1)
                            * (Robot.isSimulation() ? -1 : 1)
                            * (SmartDashboard.getBoolean("Invert Rotation", false) ? -1 : 1)
                            * (DriverStation.isAutonomous() ? 0 : 1),
                    () -> m_driverController.getRightY()
                            * (DriverStation.getAlliance().orElse(Alliance.Red)
                                    .equals(Alliance.Blue) ? 1 : -1)
                            * (Robot.isSimulation() ? -1 : 1)
                            * (SmartDashboard.getBoolean("Invert Rotation", false) ? -1 : 1)
                            * (DriverStation.isAutonomous() ? 0 : 1));

    SwerveInputStream driveRobotOriented = SwerveInputStream.of(m_swerveSubsystem.getSwerveDrive(),
            () -> m_driverController.getLeftY()
                    * (Robot.isSimulation() ? 1 : -1)
                    * (SmartDashboard.getBoolean("Invert Translation", false) ? 1 : -1)
                    * (DriverStation.isAutonomous() ? 0 : 1),
            () -> m_driverController.getLeftX()
                    * (Robot.isSimulation() ? 1 : -1)
                    * (SmartDashboard.getBoolean("Invert Translation", false) ? 1 : -1)
                    * (DriverStation.isAutonomous() ? 0 : 1))
            .robotRelative(true)
            .deadband(OperatorConstants.kDEADBAND)
            .scaleTranslation(0.8)
            .allianceRelativeControl(false)
            .withControllerRotationAxis(() -> -m_driverController.getRightX());

    Command driveFieldOritentedDirectAngle = m_swerveSubsystem.driveFieldOriented(driveDirectAngle);
    Command driveRobotOrientedAngularVelocity = m_swerveSubsystem.driveFieldOriented(driveRobotOriented);

    Command driveRobotOrientedHubLocked = new HubLock(m_swerveSubsystem, m_shooterSubsystem, m_driverController);
    Command autoHubLock = new AutoHubLock(m_swerveSubsystem, m_shooterSubsystem, m_driverController);
    ParallelHubLock parallelHubLock = new ParallelHubLock(m_shooterSubsystem, m_swerveSubsystem);

    Command driveAutoAlign = new AprilTagAlign(m_swerveSubsystem, m_visionSubsystem);
    Command driveShooterAlign = new ShooterAlign(m_swerveSubsystem, m_shooterSubsystem);
    Command shootyBoi = new Shoot(m_shooterSubsystem, m_intakeSubsystem, m_indexerSubsystem);
    Command indexerCommand = m_indexerSubsystem.spinIndexerCommand(0.8);
    Command reverseIndexerCommand = m_indexerSubsystem.spinIndexerCommand(-0.8);
    Command stopIndexerCommand = m_indexerSubsystem.spinIndexerCommand(0);
    Command intakeCommand = m_intakeSubsystem.runIntakeMotor(1); // temporary (vu postulate)
    Command stopIntakeCommand = m_intakeSubsystem.stopIntakeCommand();

    // -----------------------------------------------------------------------------------

    private void configureBindings() {
        SmartDashboard.putBoolean("Invert Translation", false);
        SmartDashboard.putBoolean("Invert Rotation", false);
        m_swerveSubsystem.setDefaultCommand(driveFieldOritentedDirectAngle);
        // m_swerveSubsystem.setDefaultCommand(driveRobotOrientedAngularVelocity);

        Logger.recordOutput("Robot Relative", false);

        m_driverController.a().toggleOnTrue(driveRobotOrientedAngularVelocity
                .beforeStarting(() -> {
                    Logger.recordOutput("Robot Relative", true);
                }).finallyDo(() -> {
                    Logger.recordOutput("Robot Relative", false);
                }));

        m_driverController.x().toggleOnTrue(driveRobotOrientedHubLocked);
        m_driverController.b().whileTrue(new PathPlannerAuto("Climber Align")); // temporary button -
        // great vu postulate
        m_driverController.y().whileTrue(m_swerveSubsystem.resetGyroCommand());

        m_driverController.povLeft().whileTrue(m_intakeSubsystem.setIntakePivotCommand(0));
        m_driverController.povRight().whileTrue(m_intakeSubsystem.setIntakePivotCommand(74));

        m_driverController.rightTrigger().whileTrue(shootyBoi);
        m_driverController.leftTrigger().whileTrue(intakeCommand);
        m_driverController.leftTrigger().whileFalse(stopIntakeCommand);
        m_driverController.leftBumper().whileTrue(reverseIndexerCommand).whileFalse(stopIndexerCommand);

        if (Robot.isSimulation()) {
            m_driverController.leftStick().whileTrue(SimulationSubsystem.resetFieldCommand());
        }

        NamedCommands.registerCommand("Shoot", shootyBoi);
        NamedCommands.registerCommand("Intake Up", m_intakeSubsystem.setIntakePivotCommand(0));
        NamedCommands.registerCommand("Intake Down", m_intakeSubsystem.setIntakePivotCommand(74));
        NamedCommands.registerCommand("Intake Start", intakeCommand);
        NamedCommands.registerCommand("Run Indexer", indexerCommand);
        NamedCommands.registerCommand("Intake Stop", stopIntakeCommand);
        NamedCommands.registerCommand("Intake Nudge", m_intakeSubsystem.nudgeForward());
        // NamedCommands.registerCommand("Climber Rise", riseCommand);
        // NamedCommands.registerCommand("Climber Descend", descendCommand);
        NamedCommands.registerCommand("Hub Lock", autoHubLock);
        NamedCommands.registerCommand("Parallel Hub Lock", parallelHubLock);
        NamedCommands.registerCommand("Stop Hub Lock", parallelHubLock.stop());
        NamedCommands.registerCommand("Shooter Align", driveShooterAlign);
        NamedCommands.registerCommand("Stop",
                Commands.runOnce(m_swerveSubsystem.getSwerveDrive()::lockPose, m_swerveSubsystem));
        NamedCommands.registerCommand("Shoot3s",
                new AutoShoot(m_shooterSubsystem, m_intakeSubsystem, m_indexerSubsystem, 3));
        NamedCommands.registerCommand("Shoot1s",
                new AutoShoot(m_shooterSubsystem, m_intakeSubsystem, m_indexerSubsystem, 1));
        NamedCommands.registerCommand("Shoot5s",
                new AutoShoot(m_shooterSubsystem, m_intakeSubsystem, m_indexerSubsystem, 5));
        NamedCommands.registerCommand("Shoot15s",
                new AutoShoot(m_shooterSubsystem, m_intakeSubsystem, m_indexerSubsystem, 15));
    }

    public Command getAutonomousCommand() {
        return m_autoChooser.getSelected();
    }

}
