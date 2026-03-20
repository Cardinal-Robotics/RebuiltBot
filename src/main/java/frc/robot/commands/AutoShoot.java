// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import swervelib.simulation.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;

import edu.wpi.first.wpilibj2.command.Command;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.measure.*; // this is peak
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import org.littletonrobotics.junction.Logger;

import frc.robot.Robot;
import frc.robot.subsystems.*;

public class AutoShoot extends Command {
  private ShooterSubstystem m_shooterSubstystem;
  private IntakeSubsystem m_intakeSubstystem;
  private IndexerSubsystem m_indexerSubstystem;
  private double m_fakeStartTime;
  private double m_trueStartTime;
  private final double shootTime;

  public AutoShoot(ShooterSubstystem shooterSubstystem, IntakeSubsystem intakeSubsystem, IndexerSubsystem indexer,
      double shootTime) {
    m_shooterSubstystem = shooterSubstystem;
    m_intakeSubstystem = intakeSubsystem;
    m_indexerSubstystem = indexer;

    this.shootTime = shootTime;

    addRequirements(shooterSubstystem);
  }

  @Override
  public void initialize() {
    m_trueStartTime = Timer.getFPGATimestamp();


    if (Robot.isSimulation()) {
      m_fakeStartTime = Timer.getFPGATimestamp();
      m_trueStartTime = m_fakeStartTime;
    }
  }

  @Override
  public void execute() {
    if (m_shooterSubstystem.atTargetSpeed()) {
      m_shooterSubstystem.setUptake(0.8);
      m_indexerSubstystem.spinIndexer(1);
    } else {
      m_shooterSubstystem.setUptake(0.0);
      m_indexerSubstystem.stopIndexer();
    }

    if (Robot.isSimulation() && (Timer.getFPGATimestamp() - m_fakeStartTime) > 0.1) { // "units are a man's worst enemy"
                                                                                      // - Charlie Malerich 1/26/2026
      m_fakeStartTime = Timer.getFPGATimestamp();

      double[] conditions = m_shooterSubstystem.getIdealShooterConditions();
      double RPM = conditions[0];
      double phi = conditions[1];

      if (Double.isNaN(RPM) || Double.isNaN(phi))
        return;

      // Only shoot if we can actually make it and if we have the ammo for it.
      if (!m_intakeSubstystem.obtainGamePieceFromIntake())
        return;

      m_shooterSubstystem.setTargetSpeedRPM(RPM);
      this.m_shooterSubstystem.createSimulatedFuelProjectile(); // shoot ball
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooterSubstystem.setUptake(0.0);
    m_indexerSubstystem.stopIndexer();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Timer.getFPGATimestamp() - m_trueStartTime) > shootTime;
  }
}
