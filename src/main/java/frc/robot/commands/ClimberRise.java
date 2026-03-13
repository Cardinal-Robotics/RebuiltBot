// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.security.Timestamp;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.ClimberSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimberRise extends Command {
  private ClimberSubsystem m_climberSubsystem;
  private double m_startTime;
  private final double climbTime;

  public ClimberRise(ClimberSubsystem climberSubsystem, double climbTime) {
    m_climberSubsystem = climberSubsystem;
    this.climbTime = climbTime;

    addRequirements(climberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_climberSubsystem.rise();
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Robot.isSimulation() && (Timer.getFPGATimestamp() - m_startTime) > climbTime) return true;
    else return false;
  }
}
