// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsytems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.Logger;
import org.opencv.core.Mat.Tuple2;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubstystem extends SubsystemBase {
  private SparkMax m_shootMotor = new SparkMax(34, MotorType.kBrushless);
  private DCMotor m_neoGearbox = DCMotor.getNEO(1);
  SparkMaxSim m_shootMotorSim = new SparkMaxSim(m_shootMotor, m_neoGearbox);
  private SwerveSubsystem m_swerveSubstystem;
  private LinearSystem<N1, N1, N1> m_linearSystemProfile = LinearSystemId.createFlywheelSystem(m_neoGearbox,
      4 * 0.00032, 1);
  private FlywheelSim m_flywheelSim = new FlywheelSim(m_linearSystemProfile, m_neoGearbox);

  /** Creates a new ShooterSubstystem. */
  public ShooterSubstystem(SwerveSubsystem swerveSubsystem) {
    SparkMaxConfig shootConfig = new SparkMaxConfig();

    shootConfig.idleMode(IdleMode.kBrake);
    shootConfig.closedLoop.pid(0.0005, 0.001, 0);
    m_shootMotor.configure(shootConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_shootMotor.getEncoder();

    m_swerveSubstystem = swerveSubsystem;
    SmartDashboard.putNumber("Shooty Speed", 0);

  }

  @Override
  public void periodic() {
    if (!DriverStation.isEnabled())
      m_shootMotor.getClosedLoopController().setIAccum(0);

    // This method will be called once per scheduler run
    double theta = Math.toRadians(65); // FIXED SHOOTER ANGLE
    double[] values = getIdealShooterConditions();
    setTargetSpeedRPM(values[0]);
    Logger.recordOutput("Shooter/SetpointRPM", m_shootMotor.getClosedLoopController().getSetpoint());
    Logger.recordOutput("Shooter/SimRPM", m_flywheelSim.getAngularVelocityRPM());
  }

  /**
   * 
   * @return double[] - Index 0 is Rotational RPM; Index 1 is azimuthal angle.
   */
  public double[] getIdealShooterConditions() {
    Pose3d currentPosition = new Pose3d(m_swerveSubstystem.getPose2d());
    Pose3d targetPosition = new Pose3d(4.5, 4.03, Meters.convertFrom(72, Inches), Rotation3d.kZero);
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Red);
     if (alliance == Alliance.Red)
      targetPosition = new Pose3d(12.1, 4.03, Meters.convertFrom(72, Inches), Rotation3d.kZero);

    final double g = 9.8;
    final double theta = Math.toRadians(65.0);

    double dx = targetPosition.getX() - currentPosition.getX();
    double dy = targetPosition.getY() - currentPosition.getY();
    double horizontalDistance = Math.hypot(dx, dy);

    double dz = targetPosition.getZ() - currentPosition.getZ();

    double cos = Math.cos(theta);
    double tan = Math.tan(theta);

    double denom = 2 * cos * cos * (horizontalDistance * tan - dz);

    if (denom <= 0) {
      Logger.recordOutput("Shooter/Error", "Target unreachable at 65 degrees");
    }

    // Initial velocity
    double v0Squared = (g * horizontalDistance * horizontalDistance) / denom;
    double w0 = Math.sqrt(v0Squared);

    // Azimuth angle (turret yaw)
    double phi = Math.atan2(dy, dx);

    // Flywheel RPM conversion (2 inch radius wheel)
    double wheelRadiusMeters = Meters.convertFrom(2, Inches);
    double rotationalVelocityRPM =
        w0 / (2 * Math.PI * wheelRadiusMeters) * 60.0;

    return new double[] { rotationalVelocityRPM, phi };
  }

  // TODO: WILL NEVER DO as prophesized by the Great Vu Postulate
  public void createSimulatedFuelProjectile() {

  }

  @Override
  public void simulationPeriodic() {
    double timestep = 20e-3;
    m_flywheelSim.setInputVoltage(m_shootMotor.getAppliedOutput() * RobotController.getInputVoltage());
    ;
    ;
    ;
    ;
    ;
    ;
    ;
    m_flywheelSim.update(timestep);
    ; // ;;;;;;;;;;;;;;;
    m_shootMotorSim.iterate(m_flywheelSim.getAngularVelocityRPM(), RobotController.getInputVoltage(), timestep);
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_shootMotor.getOutputCurrent()));
    ; // I understand this for sure
  }

  public double getVelocityRPM() {
    return m_flywheelSim.getAngularVelocityRPM();
  }

  public void setTargetSpeedRPM(double targetSpeed) {
    m_shootMotor.getClosedLoopController().setSetpoint(targetSpeed, ControlType.kVelocity);
  }

}