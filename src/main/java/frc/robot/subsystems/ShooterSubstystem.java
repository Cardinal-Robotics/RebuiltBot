// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose3d;
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

  private SparkMax m_uptakeMotor = new SparkMax(31, MotorType.kBrushless);

  private DCMotor m_neoGearbox = DCMotor.getNEO(1);

  private SparkMaxSim m_shootMotorSim = new SparkMaxSim(m_shootMotor, m_neoGearbox);

  private SwerveSubsystem m_swerveSubstystem;

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
      new SysIdRoutine.Config(),
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
    shootConfig.closedLoop.pid(0.00019762, 0, 0);
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
    SmartDashboard.putNumber("RPM", 0);
    SmartDashboard.putNumber("kF", 0);
    SmartDashboard.putNumber("kMultipier", 0);
  }

  @Override
  public void periodic() {
    if (!DriverStation.isEnabled()) {
      m_shootMotor.getClosedLoopController().setIAccum(0);

    }

    double[] values = getIdealShooterConditions();
    if (Double.isNaN(values[0]))
    return;

    double targetRPM = values[0] * SmartDashboard.getNumber("kMultipier", 0);
    setTargetSpeedRPM(targetRPM);


    // This method will be called once per scheduler run

        
/* 
    Logger.recordOutput(
        "Shooter/SetpointRPM",
        m_shootMotor.getClosedLoopController().getSetpoint()); */

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
        Rotation3d.kZero
      );
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
   *         Index 0 is Rotational RPM
   *         Index 1 is azimuthal angle
   */
  public double[] getIdealShooterConditions() {

    final double g = 9.8;
    final double theta = this.theta;
    final double t_min = 0.5;
    final double t_max = 5;

    double[] netTranslation = calculateNetTargetTranslation();

    double dx = netTranslation[0], dy = netTranslation[1], dz = netTranslation[2];
    double v_robotX = m_swerveSubstystem.getFieldVelocity().vxMetersPerSecond;
    double v_robotY = m_swerveSubstystem.getFieldVelocity().vyMetersPerSecond;
    double w_robot = m_swerveSubstystem.getFieldVelocity().omegaRadiansPerSecond;

    // Solves for t = 0 using some special technique called the BrentSolver.
    UnivariateFunction f = t -> {
      double phi = Math.atan2(dy - v_robotY * t, dx - v_robotX * t);
      
      double v_ySolution = (dy - v_robotY * t) / (Math.sin(theta) * Math.sin(phi));
      double interceptValue = v_ySolution;

      // If dy - v_robotY = 0 for some reason, it will be NaN because it has already reached the optimal target location on the y-axis. Instead focus on the x-axis.
      if(Double.isNaN(v_ySolution)) {
        double v_xSolution = (dx - v_robotX * t) / (Math.sin(theta) * Math.cos(phi));
        interceptValue = v_xSolution;
      }

      double v_zSolution = (dz + 4.9 * t * t) / (Math.cos(theta));

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

    double v0 = (dz + 4.9 * t * t) / (Math.cos(theta));
    double flywheelRPM = v0 / flywheelConversionFactor;
    double phi = Math.atan2(dy - v_robotY * t, dx - v_robotX * t);


    return new double[] { flywheelRPM, phi };
  }

  // TODO: WILL NEVER DO as prophesized by the Great Vu Postulate
  // ERM ACTUALLY
  public void createSimulatedFuelProjectile() {
    double v0 = getVelocityRPM() * flywheelConversionFactor;
    RebuiltFuelOnFly fuelOnFly = new RebuiltFuelOnFly(
        m_swerveSubstystem.getPose2d().getTranslation(),
        new Translation2d(0, 0),
        m_swerveSubstystem.getFieldVelocity(),
        m_swerveSubstystem.getPose2d().getRotation(),
        Distance.ofBaseUnits(0.45, Meters),
        LinearVelocity.ofBaseUnits(v0, MetersPerSecond), // V sub 0 = sqrt(x^2/(2s)^2 + (72 in +
        // ((0.5)(9.8)((2s)^2))^2)/(2s)^2)
        Angle.ofBaseUnits((Math.PI / 2 ) - theta, Radians));

    fuelOnFly
        .withTargetPosition(() -> FieldMirroringUtils.toCurrentAllianceTranslation(new Translation3d(4.5,
            4.03, Meters.convertFrom(72, Inches))))
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

  // Not working, Fix later (vu postulate)
  public boolean isValidShot() {
    final Pose3d currentPosition = new Pose3d(m_swerveSubstystem.getPose2d());
    final Pose3d shooterPosition = currentPosition.plus(shooterOffset);
    final Pose3d targetPosition = getTargetPosition();

    double rpm = getVelocityRPM();
    double v0 = rpm * flywheelConversionFactor;
    double x0 = m_swerveSubstystem.getPose2d().getX();
    double y0 = m_swerveSubstystem.getPose2d().getY();
    double dz = targetPosition.getZ() - shooterPosition.getZ();
    double phi = m_swerveSubstystem.getPose2d().getRotation().getRadians();
    double v_robotX = m_swerveSubstystem.getFieldVelocity().vxMetersPerSecond;
    double v_robotY = m_swerveSubstystem.getFieldVelocity().vyMetersPerSecond;

    double t = (v0 * Math.cos(theta) + Math.sqrt(Math.pow(v0 * Math.cos(theta), 2) - (4 * 4.9 * dz))) / 9.8;

    double px = (v0 * Math.sin(theta) * Math.cos(phi) + v_robotX) * t + x0;
    double py = (v0 * Math.sin(theta) * Math.sin(phi) + v_robotY) * t + y0;

    // Check if blue works.
    double blueX1 = 4.40, blueY1 = 4.40;
    double blueX2 = 4.9, blueY2 = 3.75;
    double minBlueX = Math.min(blueX1, blueX2);
    double maxBlueX = Math.max(blueX1, blueX2);
    double minBlueY = Math.min(blueY1, blueY2);
    double maxBlueY = Math.max(blueY1, blueY2);

    boolean insideBlue = (px >= minBlueX && px <= maxBlueX) &&
        (py >= minBlueY && py <= maxBlueY);

    // check if it goes into red
    double redX1 = 16.54 - blueX1, redY1 = 8.07 - blueY1;
    double redX2 = 16.54 - blueX2, redY2 = 8.07 - blueY2;
    double minRedX = Math.min(redX1, redX2);
    double maxRedX = Math.max(redX1, redX2);
    double minRedY = Math.min(redY1, redY2);
    double maxRedY = Math.max(redY1, redY2);

    boolean insideRed = (px >= minRedX && px <= maxRedX) &&
        (py >= minRedY && py <= maxRedY);

    return insideBlue || insideRed;
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
    double setpoint = m_shootMotor.getClosedLoopController().getSetpoint();
    double error = Math.abs(setpoint - m_shootMotor.getEncoder().getVelocity());
    return error <= 200;
  }

  public void setTargetSpeedRPM(double targetSpeed) {
    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.1148, 0.0020037, 0.00037093);

    double feedforwardVoltage = feedforward.calculate(targetSpeed);

    Logger.recordOutput("Shooter/SetpointRPM", targetSpeed);
     m_shootMotor
        .getClosedLoopController()
        .setSetpoint(targetSpeed, ControlType.kVelocity, ClosedLoopSlot.kSlot0, feedforwardVoltage, ArbFFUnits.kVoltage);
  }

  public void setUptake(double speed) {
    m_uptakeMotor.set(speed);
  }
}
