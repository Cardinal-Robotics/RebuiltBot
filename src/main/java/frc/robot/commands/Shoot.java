// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import swervelib.simulation.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import swervelib.simulation.ironmaple.utils.FieldMirroringUtils;
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

public class Shoot extends Command {
  private ShooterSubstystem m_shooterSubstystem;
  private IntakeSubsystem m_intakeSubstystem;
  private IndexerSubsystem m_indexerSubstystem;
  private Timer simCooldownTimer = new Timer();
  private double m_startTime;


  public Shoot(ShooterSubstystem shooterSubstystem, IntakeSubsystem intakeSubsystem, IndexerSubsystem indexer) {
    m_shooterSubstystem = shooterSubstystem;
    m_intakeSubstystem = intakeSubsystem;
    m_indexerSubstystem = indexer;

    addRequirements(shooterSubstystem);
  }

  @Override
  public void initialize() {
    if (Robot.isSimulation()) {
        m_startTime = Timer.getFPGATimestamp();
        simCooldownTimer.restart();
    }
  }

  @Override
  public void execute() {
    if(m_shooterSubstystem.atTargetSpeed()) {
      m_shooterSubstystem.setUptake(0.8);
      m_indexerSubstystem.spinIndexer(0.8);
    } else { 
      m_shooterSubstystem.setUptake(0.0);
      m_indexerSubstystem.stopIndexer();
    }

    if (Robot.isSimulation() && simCooldownTimer.hasElapsed(0.1)) { // "units are a man's worst enemy" -
                                                                                  // Charlie Malerich 1/26/2026
      this.m_shooterSubstystem.createSimulatedFuelProjectile(); // shoot ball
        simCooldownTimer.restart();
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
    return false;
  }
}
