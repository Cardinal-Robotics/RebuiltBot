// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import org.apache.commons.math3.analysis.UnivariateFunction;
import org.apache.commons.math3.analysis.solvers.BrentSolver;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Robot;
import frc.robot.subsystems.shooter.FuelPhysicsSim;
import frc.robot.subsystems.shooter.ProjectileSimulator;
import frc.robot.subsystems.shooter.ShotCalculator;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import swervelib.simulation.ironmaple.utils.FieldMirroringUtils;

public class ShooterSubstystem extends SubsystemBase {
  private SparkMax m_shootMotor = new SparkMax(35, MotorType.kBrushless);
  private SparkClosedLoopController shootController = m_shootMotor.getClosedLoopController();

  private SparkMax m_uptakeMotor = new SparkMax(31, MotorType.kBrushless);

  private DCMotor m_neoGearbox = DCMotor.getNEO(1);

  private SparkMaxSim m_shootMotorSim = new SparkMaxSim(m_shootMotor, m_neoGearbox);

  private SwerveSubsystem m_swerveSubstystem;
  private Rotation2d idealShotAngle = Rotation2d.kZero;
  private double setpoint = 0;

  private LinearSystem<N1, N1, N1> m_linearSystemProfile = LinearSystemId.createFlywheelSystem(
      m_neoGearbox,
      4 * 0.00032,
      1);

  private FlywheelSim m_flywheelSim = new FlywheelSim(m_linearSystemProfile, m_neoGearbox);

  private final double theta = Math.toRadians(90 - 25);
  private final double wheelRadiusMeters = Meters.convertFrom(2, Inches);
  private final double flywheelConversionFactor = (2 * Math.PI * wheelRadiusMeters) / 60.0;
  private final Transform3d shooterOffset = new Transform3d(0.140, 0.145, 0.419, Rotation3d.kZero);

  private ProjectileSimulator.SimParameters params;
  private ShotCalculator.Config shotConfig;
  private ShotCalculator shotCalculator;
  private FuelPhysicsSim ballSim;

  private SysIdRoutine routine = new SysIdRoutine(
      new SysIdRoutine.Config(null, null, null, (state) -> Logger.recordOutput("SysIdTestState", state.toString())),
      new SysIdRoutine.Mechanism(
          (voltage) -> m_shootMotor.setVoltage(voltage.in(Volts)),
          null,
          this));

  public ShooterSubstystem(SwerveSubsystem swerveSubsystem) {
    SmartDashboard.putData(routine.dynamic(Direction.kForward).withName("Forward Dynamic"));
    SmartDashboard.putData(routine.dynamic(Direction.kReverse).withName("Reverse Dynamic"));
    SmartDashboard.putData(routine.quasistatic(Direction.kForward).withName("Forward Quasistatic"));
    SmartDashboard.putData(routine.quasistatic(Direction.kReverse).withName("Reverse Quasistatic"));

    params = new ProjectileSimulator.SimParameters(
        0.215,
        0.1501,
        0.47,
        0.2,
        1.225,
        shooterOffset.getZ(), // height offset of where the ball actually zomes out
        2 * wheelRadiusMeters, // diameter; use calipers?
        1.9,
        0.425, // needs real world tuning
        Math.toDegrees(theta), // output angle
        0.001, 
        1500, 6000, 25, 5.0);

    ProjectileSimulator simulator = new ProjectileSimulator(params);
    ProjectileSimulator.GeneratedLUT lut = simulator.generateLUT();

    shotConfig = new ShotCalculator.Config();
    shotConfig.launcherOffsetX = shooterOffset.getX();
    shotConfig.launcherOffsetY = shooterOffset.getY();

    shotCalculator = new ShotCalculator(shotConfig);

    for(var entry : lut.entries()) {
        if(entry.reachable()) {
            shotCalculator.loadLUTEntry(entry.distanceM(), entry.rpm(), entry.tof());
            System.out.println(entry);
        } 
    }


    SparkMaxConfig shooterMotorConfig = new SparkMaxConfig();
    shooterMotorConfig.idleMode(IdleMode.kCoast);
    shooterMotorConfig.closedLoop.pid(0.0006832, 0, 0);
    // shootConfig.closedLoop.allowedClosedLoopError(100, ClosedLoopSlot.kSlot0);
    shooterMotorConfig.inverted(true);
    shooterMotorConfig.encoder.quadratureMeasurementPeriod(10);
    shooterMotorConfig.encoder.quadratureAverageDepth(1);

    SparkMaxConfig uptakeConfig = new SparkMaxConfig();
    uptakeConfig.idleMode(IdleMode.kBrake);
    uptakeConfig.inverted(true);
    m_shootMotor.configure(
        shooterMotorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    m_uptakeMotor.configure(
        uptakeConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    m_shootMotor.getEncoder();
    m_uptakeMotor.getEncoder();

    m_swerveSubstystem = swerveSubsystem;

    // SmartDashboard.putNumber("RPM", 4630);
    SmartDashboard.putNumber("kMultiplier", 1.5);
    SmartDashboard.putNumber("RPM", 0);
    SmartDashboard.putNumber("kF", 0);
    SmartDashboard.putBoolean("Force Enable Shooter", false);

    if(Robot.isSimulation()) {
        this.ballSim = new FuelPhysicsSim("AdvantageKit/RealOutputs/FieldSimulation/Fuel");
        ballSim.enable();
        ballSim.placeFieldBalls();
        ballSim.configureRobot(
            Meters.convertFrom(25, Inches),
            Meters.convertFrom(29, Inches),
            0.127, // This number might not be right, I just copied it from someone else.
            () -> swerveSubsystem.getPose2d(),
            () -> swerveSubsystem.getFieldVelocity());
    }
  }

  @Override
  public void periodic() {
    if (!DriverStation.isEnabled()) {
      m_shootMotor.getClosedLoopController().setIAccum(0);
    }

    ShotCalculator.ShotInputs inputs = new ShotCalculator.ShotInputs(
        m_swerveSubstystem.getPose2d(),
        m_swerveSubstystem.getFieldVelocity(),
        m_swerveSubstystem.getRobotVelocity(),
        getTargetPosition().toPose2d().getTranslation(),
        DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue) ? new Translation2d(1, 0) : new Translation2d(-1, 0),
        0.9);

    ShotCalculator.LaunchParameters shot = shotCalculator.calculate(inputs);
    Logger.recordOutput("Shooter/isValidShot", shot.isValid());
    Logger.recordOutput("Shooter/Confidence", shot.confidence());
    Logger.recordOutput("Shooter/TOF", shot.timeOfFlightSec());

    this.idealShotAngle = shot.driveAngle();
    setTargetSpeedRPM(shot.rpm());

    // This method will be called once per scheduler run
    Logger.recordOutput("Shooter/output", shot.rpm());
    Logger.recordOutput(
        "Shooter/SimRPM",
        m_flywheelSim.getAngularVelocityRPM());
    Logger.recordOutput(
        "Shooter/RPM",
        m_shootMotor.getEncoder().getVelocity());
  }

  public void adjustShooterOffset(double rpmOffset) { shotCalculator.adjustOffset(rpmOffset); }
  public void resetShooterOffset() { shotCalculator.resetOffset(); }

  private Pose3d getTargetPosition() {
    Pose3d targetPosition = new Pose3d(
        4.6,
        4.0,
        Meters.convertFrom(72 + 5, Inches),
        Rotation3d.kZero);
    
    if (FieldMirroringUtils.isSidePresentedAsRed()) {
      targetPosition = new Pose3d(
          11.95,
          4.06,
          Meters.convertFrom(72 + 5, Inches),
          Rotation3d.kZero);
    }

    return targetPosition;
  }

  public Rotation2d getIdealDriveAngle() {
    return this.idealShotAngle;
  }

  // TODO: WILL NEVER DO as prophesized by the Great Vu Postulate
  // ERM ACTUALLY
  public void createSimulatedFuelProjectile() {
    Pose2d robotPose = m_swerveSubstystem.getPose2d();
    Rotation2d robotRotation = robotPose.getRotation();
    Translation3d launchPos = new Pose3d(robotPose).transformBy(shooterOffset).getTranslation();

    double velocityRPM = getVelocityRPM();
    double velocityLinear = ProjectileSimulator.rpmToExitVelocity(velocityRPM, params.wheelDiameterM(), params.slipFactor());

    double vHorizontal = velocityLinear * Math.cos(theta);
    double vVertical = velocityLinear * Math.sin(theta);
    double vx = vHorizontal * robotRotation.getCos();
    double vy = vHorizontal * robotRotation.getSin();
    
    Translation3d launchVelocity = new Translation3d(vx, vy, vVertical);

    ballSim.launchBall(launchPos, launchVelocity, velocityRPM);
  }

  @Override
  public void simulationPeriodic() {
    double timestep = 20e-3;

    ballSim.tick();

    m_flywheelSim.setInputVoltage(
        m_shootMotor.getAppliedOutput()
            * RobotController.getInputVoltage());

    m_flywheelSim.update(timestep);

    m_shootMotorSim.iterate(
        m_flywheelSim.getAngularVelocityRPM(),
        RobotController.getInputVoltage(),
        timestep);

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(
            m_shootMotor.getOutputCurrent()));
  }

  public double getVelocityRPM() {
    if (Robot.isSimulation())
      return m_flywheelSim.getAngularVelocityRPM();
    return m_shootMotor.getEncoder().getVelocity();
  }

  public boolean atTargetSpeed() {
    double error = Math.abs(this.setpoint - getVelocityRPM());
    Logger.recordOutput("Shooter/RPM Error", error);
    return error <= 100;
  }

  public void setTargetSpeedRPM(double targetSpeed) {
    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.092747, 0.0020776, 0.0006831);
    double feedforwardVoltage = feedforward.calculate(targetSpeed);
    this.setpoint = targetSpeed;

    Logger.recordOutput("Shooter/SetpointRPM", targetSpeed);
    shootController
        .setSetpoint(targetSpeed, ControlType.kVelocity, ClosedLoopSlot.kSlot0, feedforwardVoltage,
            ArbFFUnits.kVoltage);

    if (SmartDashboard.getBoolean("Force Enable Shooter", false)) {
      shootController
          .setSetpoint(targetSpeed, ControlType.kVelocity, ClosedLoopSlot.kSlot0, feedforwardVoltage,
              ArbFFUnits.kVoltage);
    }
  }

  public void setUptake(double speed) {
    m_uptakeMotor.set(speed);
  }
}
