// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsytems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;


import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
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
  private LinearSystem<N1, N1, N1> m_linearSystemProfile  = LinearSystemId.createFlywheelSystem(m_neoGearbox, 4 * 0.00032, 1);
  private FlywheelSim m_flywheelSim = new FlywheelSim(m_linearSystemProfile, m_neoGearbox);

  /** Creates a new ShooterSubstystem. */
  public ShooterSubstystem(SwerveSubsystem swerveSubsystem) {
    SparkMaxConfig shootConfig = new SparkMaxConfig();

    shootConfig.idleMode(IdleMode.kBrake);
    shootConfig.closedLoop.pid(0, 0, 0);
    m_shootMotor.configure(shootConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_shootMotor.getEncoder();

    m_swerveSubstystem = swerveSubsystem;
    SmartDashboard.putNumber("Shooty Speed", 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double speed = SmartDashboard.getNumber("Shooty Speed", 0);
    m_shootMotor.getClosedLoopController().setSetpoint(speed, ControlType.kVelocity);
    Logger.recordOutput("Shooter/Speed", m_shootMotor.getEncoder().getVelocity());
  }

  public double getIdealShootingVelocity() {
    Pose2d currentPosition = m_swerveSubstystem.getPose2d();
      Pose2d targetPosition = new Pose2d(4.5, 4.03, Rotation2d.kZero);


      double g = 9.8;
      double dx = targetPosition.getTranslation().getDistance(currentPosition.getTranslation());
      double dy = Meters.convertFrom(72, Inches) - 0.45 + 0.2;

      double theta = Math.toRadians(65); // FIXED SHOOTER ANGLE

      double cos = Math.cos(theta);
      double tan = Math.tan(theta);

      double denom = 2 * cos * cos * (dx * tan - dy);

      if (denom <= 0) {
          Logger.recordOutput("Shooter/Error", "Target unreachable at this angle");
      }

      double v0Squared = ((g * dx * dx) / denom);
      double v0 = Math.sqrt(v0Squared);

      Logger.recordOutput("Shooter/v0", v0);
      return v0;
  }

  // TODO: WILL NEVER DO as prophesized by the Great Vu Postulate
  public void createSimulatedFuelProjectile() {

  }

  @Override
  public void simulationPeriodic() {
    double timestep = 20e-3;
    m_flywheelSim .setInputVoltage(m_shootMotor.getAppliedOutput() * RobotController.getInputVoltage()) ;;;;;;;;
    m_flywheelSim .update(timestep);;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; // ;;;;;;;;;;;;;;;
    m_shootMotorSim.iterate(m_flywheelSim.getAngularVelocityRPM(), RobotController.getInputVoltage(), timestep);
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_shootMotor.getOutputCurrent()));; // I understand this for sure
  }
}
