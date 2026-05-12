// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class ThreeSixtyNoScope extends Command {
  SwerveSubsystem swerveSubsystem;
  Timer timer = new Timer();

  public ThreeSixtyNoScope(SwerveSubsystem swerveSubsystem) {
    this.swerveSubsystem = swerveSubsystem;
    addRequirements(swerveSubsystem);
  }

  @Override
  public void initialize() {
    timer.restart();
  }

  @Override
  public void execute() {
    swerveSubsystem.driveRelative(new ChassisSpeeds(0, 0, 2 * 2 * Math.PI));
  }

  @Override
  public void end(boolean interrupted) {
    // Tell the robot to stop moving on x,y, and turn.
    swerveSubsystem.driveFieldOriented(new ChassisSpeeds());
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(0.6);
  }
}
