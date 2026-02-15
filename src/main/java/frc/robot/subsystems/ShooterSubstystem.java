// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import org.apache.commons.math3.analysis.UnivariateFunction;
import org.apache.commons.math3.analysis.solvers.BrentSolver;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

public class ShooterSubstystem extends SubsystemBase {
  private SparkMax m_shootMotor = new SparkMax(34, MotorType.kBrushless);

  private SparkMax m_uptakeMotor = new SparkMax(348, MotorType.kBrushless);

  private DCMotor m_neoGearbox = DCMotor.getNEO(1);

  private SparkMaxSim m_shootMotorSim = new SparkMaxSim(m_shootMotor, m_neoGearbox);

  private SwerveSubsystem m_swerveSubstystem;

  private LinearSystem<N1, N1, N1> m_linearSystemProfile = LinearSystemId.createFlywheelSystem(
      m_neoGearbox,
      4 * 0.00032,
      1);

  private FlywheelSim m_flywheelSim = new FlywheelSim(m_linearSystemProfile, m_neoGearbox);

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
    double theta = Math.toRadians(65); // FIXED SHOOTER ANGLE

    double[] values = getIdealShooterConditions();
    if(Double.isNaN(values[0])) return;

    setTargetSpeedRPM(values[0]);

    Logger.recordOutput(
        "Shooter/SetpointRPM",
        m_shootMotor.getClosedLoopController().getSetpoint());

    Logger.recordOutput(
        "Shooter/SimRPM",
        m_flywheelSim.getAngularVelocityRPM());
  }

  /**
   * @return double[]
   *         Index 0 is Rotational RPM
   *         Index 1 is azimuthal angle
   */
  public double[] getIdealShooterConditions() {
    Pose3d currentPosition = new Pose3d(m_swerveSubstystem.getPose2d());

    Pose3d targetPosition = new Pose3d(
        4.5,
        4.03,
        Meters.convertFrom(72, Inches),
        Rotation3d.kZero);

    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Red);

    if (alliance == Alliance.Red) {
      targetPosition = new Pose3d(
          12.1,
          4.03,
          Meters.convertFrom(72, Inches),
          Rotation3d.kZero);
    }

    final double g = 9.8;
    final double theta = Math.toRadians(65.0);
    final double t_min = 0.5;
    final double t_max = 3.5;

    double dx = targetPosition.getX() - currentPosition.getX();
    double dy = targetPosition.getY() - currentPosition.getY();
    double dz = targetPosition.getZ() - currentPosition.getZ();
    double v_robotX = m_swerveSubstystem.getFieldVelocity().vxMetersPerSecond;
    double v_robotY = m_swerveSubstystem.getFieldVelocity().vyMetersPerSecond;

    // Solves for t = 0 using some special technique called the BrentSolver.
    UnivariateFunction f = t -> {
      double rx = dx - (v_robotX * t);
      double ry = dy - (v_robotY * t);
      double r = Math.hypot(rx, ry);
      return Math.tan(theta) * r - (0.5 * g * t * t) - dz;
    };

    BrentSolver functionSolver = new BrentSolver(1e-6);
    double t = 0;

    // If there are no valid solutions, it throws an error
    try {
      t = functionSolver.solve(100, f, t_min, t_max);
    } catch (Exception e) {
      return new double[] {Double.NaN, Double.NaN};
    }

    // Given t, now solve for v0 and phi.
    double rx = dx - (v_robotX * t);
    double ry = dy - (v_robotY * t);
    double r = Math.hypot(rx, ry);

    double v0 = r / (t * Math.cos(theta));
    double phi = Math.atan2(ry, rx);

    // Flywheel RPM conversion (2 inch radius wheel)
    double wheelRadiusMeters = Meters.convertFrom(2, Inches);
    double rotationalVelocityRPM = v0 / (2 * Math.PI * wheelRadiusMeters) * 60.0;

    return new double[] { rotationalVelocityRPM, phi };
  }


  // TODO: WILL NEVER DO as prophesized by the Great Vu Postulate
  public void createSimulatedFuelProjectile() {
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
