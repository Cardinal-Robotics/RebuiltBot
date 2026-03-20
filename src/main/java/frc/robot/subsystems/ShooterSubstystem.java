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
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Robot;
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
  private double setpoint = 0;

  private LinearSystem<N1, N1, N1> m_linearSystemProfile = LinearSystemId.createFlywheelSystem(
      m_neoGearbox,
      4 * 0.00032,
      1);

  private FlywheelSim m_flywheelSim = new FlywheelSim(m_linearSystemProfile, m_neoGearbox);

  private final double theta = Math.toRadians(10);
  private final double wheelRadiusMeters = Meters.convertFrom(2, Inches);
  private final double flywheelConversionFactor = (2 * Math.PI * wheelRadiusMeters) / 60.0;
  private final Transform3d shooterOffset = new Transform3d(0.140, 0.145, 0.419, Rotation3d.kZero);

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

    SparkMaxConfig shootConfig = new SparkMaxConfig();

    shootConfig.idleMode(IdleMode.kCoast);
    shootConfig.closedLoop.pid(0.00034274, 0, 0);
    // shootConfig.closedLoop.allowedClosedLoopError(100, ClosedLoopSlot.kSlot0);
    shootConfig.inverted(true);

    shootConfig.encoder.quadratureMeasurementPeriod(10);
    shootConfig.encoder.quadratureAverageDepth(1);

    SparkMaxConfig uptakeConfig = new SparkMaxConfig();
    uptakeConfig.idleMode(IdleMode.kBrake);
    uptakeConfig.inverted(true);

    m_shootMotor.configure(
        shootConfig,
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
  }

  @Override
  public void periodic() {
    if (!DriverStation.isEnabled()) {
      m_shootMotor.getClosedLoopController().setIAccum(0);

    }

    double[] values = getIdealShooterConditions();
    if (Double.isNaN(values[0]))
      return;

    double targetRPM = values[0] * 1.85;// SmartDashboard.getNumber("kMultiplier", 1.5);
    if (Robot.isSimulation()) {
      targetRPM = values[0];
    }

    setTargetSpeedRPM(targetRPM);

    // This method will be called once per scheduler run

    Logger.recordOutput(
        "Shooter/SimRPM",
        m_flywheelSim.getAngularVelocityRPM());
    Logger.recordOutput(
        "Shooter/RPM",
        m_shootMotor.getEncoder().getVelocity());
  }

  private Pose3d getTargetPosition() {
    Pose3d targetPosition = new Pose3d(
        4.59,
        4.06,
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

  private double[] calculateNetTargetTranslation() {
    Pose3d targetPosition = getTargetPosition();

    final Pose3d currentPosition = new Pose3d(m_swerveSubstystem.getPose2d());
    final Pose3d shooterPosition = currentPosition.plus(shooterOffset);

    double dx = targetPosition.getX() - shooterPosition.getX();
    double dy = targetPosition.getY() - shooterPosition.getY();
    double dz = targetPosition.getZ() - shooterPosition.getZ();

    return new double[] { dx, dy, dz };
  }

  /**
   * @return double[]
   *         Index 0 is Rotational RPMTho
   *         Index 1 is azimuthal angle
   */
  public double[] getIdealShooterConditions() {
    // I hate MapleSim so much bro. They use 11 as their gravity.
    final double g = Robot.isSimulation() ? 11 : 9.8;
    final double theta = this.theta;
    final double t_min = 0.5;
    final double t_max = 5;

    double[] netTranslation = calculateNetTargetTranslation();
    double dx = netTranslation[0], dy = netTranslation[1], dz = netTranslation[2];
    double w_robot = m_swerveSubstystem.getFieldVelocity().omegaRadiansPerSecond;

    Translation2d offsetField = shooterOffset.getTranslation().toTranslation2d()
        .rotateBy(m_swerveSubstystem.getPose2d().getRotation());
    Translation2d velocityDueToRotation = offsetField.rotateBy(Rotation2d.kCCW_90deg).times(w_robot);

    double v_robotX = m_swerveSubstystem.getFieldVelocity().vxMetersPerSecond;// + velocityDueToRotation.getX();
    double v_robotY = m_swerveSubstystem.getFieldVelocity().vyMetersPerSecond;// + velocityDueToRotation.getY();

    // Solves for t = 0 using some special technique called the BrentSolver.
    UnivariateFunction f = t -> {
      double phi = Math.atan2(dy - v_robotY * t, dx - v_robotX * t);

      double v_ySolution = (dy - v_robotY * t) / (Math.sin(theta) * Math.sin(phi) * t);
      double interceptValue = v_ySolution;

      // If dy - v_robotY = 0 for some reason, it will be NaN because it has already
      // reached the optimal target location on the y-axis. Instead focus on the
      // x-axis.
      if (Double.isNaN(v_ySolution)) {
        double v_xSolution = (dx - v_robotX * t) / (Math.sin(theta) * Math.cos(phi) * t);
        interceptValue = v_xSolution;
      }

      double v_zSolution = (dz + (g / 2) * t * t) / (Math.cos(theta) * t);

      return v_zSolution - interceptValue;
    };

    BrentSolver functionSolver = new BrentSolver(1e-6);
    double t = 0;

    // If there are no valid solutions, it throws an error
    try {
      t = functionSolver.solve(100, f, t_min, t_max);
    } catch (Exception e) {
      return new double[] { Double.NaN, Double.NaN };
    }

    double v0 = (dz + (g / 2) * t * t) / (Math.cos(theta) * t);
    double flywheelRPM = v0 / flywheelConversionFactor;
    double phi = Math.atan2(dy - v_robotY * t, dx - v_robotX * t);

    return new double[] { flywheelRPM, phi };
  }

  // TODO: WILL NEVER DO as prophesized by the Great Vu Postulate
  // ERM ACTUALLY
  public void createSimulatedFuelProjectile() {
    double v0 = getVelocityRPM() * flywheelConversionFactor;
    Logger.recordOutput("Shooter/simVelocity", v0);
    RebuiltFuelOnFly fuelOnFly = new RebuiltFuelOnFly(
        m_swerveSubstystem.getPose2d().getTranslation(),
        new Translation2d(shooterOffset.getX(), shooterOffset.getY()),
        m_swerveSubstystem.getFieldVelocity(),
        m_swerveSubstystem.getPose2d().getRotation(),
        Distance.ofBaseUnits(shooterOffset.getZ(), Meters),
        LinearVelocity.ofBaseUnits(v0, MetersPerSecond), // V sub 0 = sqrt(x^2/(2s)^2 + (72 in +
        // ((0.5)(9.8)((2s)^2))^2)/(2s)^2)
        Angle.ofBaseUnits((Math.PI / 2) - theta, Radians));

    Pose3d targetPosition = getTargetPosition();

    fuelOnFly
        .withTargetPosition(() -> targetPosition.getTranslation())
        .withTargetTolerance(new Translation3d(0.5, 0.5, 0.1));

    fuelOnFly
        // Configure callbacks to visualize the flight trajectory of the projectile
        .withProjectileTrajectoryDisplayCallBack(
            // Callback for when the note will eventually hit the target (if configured)
            (pose3ds) -> Logger.recordOutput("Flywheel/FuelProjectileSuccessfulShot", pose3ds.toArray(Pose3d[]::new)),
            // Callback for when the note will eventually miss the target, or if no target
            // is configured
            (pose3ds) -> Logger.recordOutput("Flywheel/FuelProjectileUnsuccessfulShot",
                pose3ds.toArray(Pose3d[]::new)));

    fuelOnFly.enableBecomesGamePieceOnFieldAfterTouchGround();

    SimulatedArena.getInstance().addGamePieceProjectile(fuelOnFly);
  }

  @Override
  public void simulationPeriodic() {
    double timestep = 20e-3;

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
    return error <= 150;
  }

  public void setTargetSpeedRPM(double targetSpeed) {
    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.13445, 0.0020244, 0.00062254);
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
