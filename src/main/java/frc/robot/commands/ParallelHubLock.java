// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ShooterSubstystem;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

public class ParallelHubLock extends Command {
  ShooterSubstystem shooter;
  SwerveSubsystem swerve;

  private double targetAngle;
  private boolean isRotationOverideEnabled = false;
  private boolean isFinished = true;

  public ParallelHubLock(ShooterSubstystem shooter, SwerveSubsystem swerve) {
    this.shooter = shooter;
    this.swerve = swerve;

    this.targetAngle = swerve.getPose2d().getRotation().getRadians();
  }

  @Override
  public void initialize() {
    PPHolonomicDriveController
        .setRotationTargetOverride(() -> isRotationOverideEnabled ? Optional.of(new Rotation2d(getShooterAngle())) : Optional.empty());
    isRotationOverideEnabled = true;
    isFinished = false;
  }

  public double getShooterAngle() {
    double phi = shooter.getIdealShooterConditions()[1];
    if (Double.isNaN(phi))
      targetAngle = swerve.getPose2d().getRotation().getRadians();
    else
      targetAngle = phi;

    return targetAngle;
  }

  @Override
  public void execute() {
    isRotationOverideEnabled = true;
    isFinished = true;
  }

  public Command stop() {
    return Commands.runOnce(() -> isRotationOverideEnabled = false);
  }

  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
