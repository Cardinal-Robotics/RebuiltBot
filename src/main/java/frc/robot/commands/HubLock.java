// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import java.util.Optional;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ShooterSubstystem;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

public class HubLock extends Command {
  CommandXboxController controller;
  ShooterSubstystem shooter;
  SwerveSubsystem swerve;

  
  private final SlewRateLimiter phiRateLimiter = new SlewRateLimiter(Math.toRadians(180));
  private double targetAngle;  
  private SwerveInputStream swerveInput;
  private boolean isEnabled = false;

  public HubLock(SwerveSubsystem swerve, ShooterSubstystem shooter, CommandXboxController driveController) {
    //addRequirements(swerve);

    this.controller = driveController;
    this.shooter = shooter;
    this.swerve = swerve;

    this.targetAngle = swerve.getPose2d().getRotation().getRadians();
    this.swerveInput = SwerveInputStream.of(swerve.getSwerveDrive(),
      () -> controller.getLeftY() * -1,
      () -> controller.getLeftX() * -1)
      .headingWhile(true)
      .deadband(OperatorConstants.kDEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true)
      .withControllerHeadingAxis(
          () -> Math.sin(targetAngle),
          () -> Math.cos(targetAngle));
  }

  @Override
  public void initialize() {
    PPHolonomicDriveController.setRotationTargetOverride(() -> isEnabled ? Optional.of(new Rotation2d(targetAngle)) : Optional.empty());
    isEnabled = true;
  }

  @Override
  public void execute() {
    double phi = shooter.getIdealShooterConditions()[1];
    if (Double.isNaN(phi))
      targetAngle = swerve.getPose2d().getRotation().getRadians();
    else targetAngle = phi;
    ChassisSpeeds input = swerveInput.get();
    input.vxMetersPerSecond = Math.min(1.0, input.vxMetersPerSecond);
    input.vxMetersPerSecond = Math.max(-1.0, input.vxMetersPerSecond);
    
    input.vyMetersPerSecond = Math.min(1.0, input.vyMetersPerSecond);
    input.vyMetersPerSecond = Math.max(-1.0, input.vyMetersPerSecond);

    this.swerve.driveFieldOriented(input);
  }

  @Override
  public void end(boolean interrupted) {
    isEnabled = false;
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
