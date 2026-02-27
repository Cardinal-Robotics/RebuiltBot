// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import org.apache.commons.math3.analysis.UnivariateFunction;
import org.apache.commons.math3.analysis.solvers.BrentSolver;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import swervelib.simulation.ironmaple.utils.FieldMirroringUtils;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.N1;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import com.pathplanner.lib.util.FlippingUtil;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

public class ShooterSubstystem extends SubsystemBase {
  private SparkMax m_shootMotor = new SparkMax(30, MotorType.kBrushless);

  private SparkMax m_uptakeMotor = new SparkMax(31, MotorType.kBrushless);

  private DCMotor m_neoGearbox = DCMotor.getNEO(1);

  private SparkMaxSim m_shootMotorSim = new SparkMaxSim(m_shootMotor, m_neoGearbox);

  private SwerveSubsystem m_swerveSubstystem;

  private LinearSystem<N1, N1, N1> m_linearSystemProfile = LinearSystemId.createFlywheelSystem(
      m_neoGearbox,
      4 * 0.00032,
      1);

  private FlywheelSim m_flywheelSim = new FlywheelSim(m_linearSystemProfile, m_neoGearbox);

  private final double theta = Math.toRadians(65);
  private final double wheelRadiusMeters = Meters.convertFrom(2, Inches);
  private final double flywheelConversionFactor = (2 * Math.PI * wheelRadiusMeters) / 60.0;
  private final Transform3d shooterOffset = new Transform3d(0, 0, 0.45, Rotation3d.kZero);

  public ShooterSubstystem(SwerveSubsystem swerveSubsystem) {
    SparkMaxConfig shootConfig = new SparkMaxConfig();

    shootConfig.idleMode(IdleMode.kBrake);
    shootConfig.closedLoop.pid(0.0005, 0.001, 0);

    SparkMaxConfig uptakeConfig = new SparkMaxConfig();
    uptakeConfig.idleMode(IdleMode.kBrake);
    uptakeConfig.closedLoop.pid(0.0005, 0.001, 0);

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

    SmartDashboard.putNumber("Shooty Speed", 0);
  }

  @Override
  public void periodic() {

    if (!DriverStation.isEnabled()) {
      m_shootMotor.getClosedLoopController().setIAccum(0);
    }

    // This method will be called once per scheduler run

    double[] values = getIdealShooterConditions();
    if (Double.isNaN(values[0]))
      return;

    setTargetSpeedRPM(values[0]);

    Logger.recordOutput(
        "Shooter/SetpointRPM",
        m_shootMotor.getClosedLoopController().getSetpoint());

    Logger.recordOutput(
        "Shooter/SimRPM",
        m_flywheelSim.getAngularVelocityRPM());
  }

  private Pose3d getTargetPosition() {
    Pose3d targetPosition = new Pose3d(
        4.5,
        4.03,
        Meters.convertFrom(72 + 5, Inches),
        Rotation3d.kZero);
    if (FieldMirroringUtils.isSidePresentedAsRed())
      targetPosition = FieldMirroringUtils.flip(targetPosition);

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
    final double t_max = 3.5;

    double[] netTranslation = calculateNetTargetTranslation();

    double dx = netTranslation[0], dy = netTranslation[1], dz = netTranslation[2];
    double v_robotX = m_swerveSubstystem.getFieldVelocity().vxMetersPerSecond;
    double v_robotY = m_swerveSubstystem.getFieldVelocity().vyMetersPerSecond;
    double w_robot = m_swerveSubstystem.getFieldVelocity().omegaRadiansPerSecond;
    double v_rotX = -w_robot * shooterOffset.getY();
    double v_rotY = w_robot * shooterOffset.getX();

    // Solves for t = 0 using some special technique called the BrentSolver.
    UnivariateFunction f = t -> {
      double rx = dx - (v_robotX * t) - (v_rotX * t);
      double ry = dy - (v_robotY * t) - (v_rotY * t);
      double r = Math.hypot(rx, ry);
      return Math.tan(theta) * r - (0.5 * g * t * t) - dz;
    };

    BrentSolver functionSolver = new BrentSolver(1e-6);
    double t = 0;

    // If there are no valid solutions, it throws an error
    try {
      t = functionSolver.solve(100, f, t_min, t_max);
    } catch (Exception e) {
      return new double[] { Double.NaN, Double.NaN };
    }

    // Given t, now solve for v0 and phi.
    double rx = dx - (v_robotX * t) - (v_rotX * t);
    double ry = dy - (v_robotY * t) - (v_rotY * t);
    double r = Math.hypot(rx, ry);

    double v0 = r / (t * Math.cos(theta));
    double phi = Math.atan2(ry, rx);

    // Flywheel RPM conversion (2 inch radius wheel)
    double rotationalVelocityRPM = v0 / flywheelConversionFactor;

    return new double[] { rotationalVelocityRPM, phi };
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
        Angle.ofBaseUnits(theta, Radians));

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

    boolean insideBlue = 
      (px >= minBlueX && px <= maxBlueX) &&
      (py >= minBlueY && py <= maxBlueY);

    // check if it goes into red
    double redX1 = 16.54 - blueX1, redY1 = 8.07 - blueY1;
    double redX2 = 16.54 - blueX2, redY2 = 8.07 - blueY2;
    double minRedX = Math.min(redX1, redX2);
    double maxRedX = Math.max(redX1, redX2);
    double minRedY = Math.min(redY1, redY2);
    double maxRedY = Math.max(redY1, redY2);

    boolean insideRed = 
      (px >= minRedX && px <= maxRedX) &&
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
    return m_flywheelSim.getAngularVelocityRPM();
  }

  public void setTargetSpeedRPM(double targetSpeed) {
    m_shootMotor
        .getClosedLoopController()
        .setSetpoint(targetSpeed, ControlType.kVelocity);
  }
}
