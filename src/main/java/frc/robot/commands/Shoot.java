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

import org.littletonrobotics.junction.Logger;

import frc.robot.subsytems.*;
import frc.robot.Robot;

public class Shoot extends Command {
  private ShooterSubstystem m_shooterSubstystem;
  private SwerveSubsystem m_swerveSubstystem;

  public Shoot(ShooterSubstystem shooterSubstystem, SwerveSubsystem swerveSubsystem) {
    m_shooterSubstystem = shooterSubstystem;
    m_swerveSubstystem = swerveSubsystem;

    addRequirements(shooterSubstystem);
  }

  @Override
  public void initialize() {
    if (Robot.isSimulation()) { // "units are a man's worst enemy" - Charlie Malerich 1/26/2026
      Pose2d currentPosition = m_swerveSubstystem.getPose2d();
      Pose2d targetPosition = new Pose2d(4.5, 4.03, Rotation2d.kZero);


      double hubHeight = Meters.convertFrom(72, Inches) - 0.45;
      
      double dx = currentPosition.getX() - targetPosition.getX();
      double dy = currentPosition.getY() - targetPosition.getY();
      double hubDistance = Math.hypot(dx, dy) - 0.2; // sqrt(dx^2 + dy^2)
      
      double sinSquared = Math.pow(Math.sin(Math.toRadians(30)), 2);

      double v_0x2 = Math.pow(hubDistance / (2 * Math.cos(Math.toRadians(30))), 2);
      double v_0y2 = (hubHeight + Math.pow((0.5 * 9.8 * 4), 2) / (4 * sinSquared));

      double initialVelocity = Math.sqrt(v_0x2 + v_0y2); // triangle gang YESSIR

      Logger.recordOutput("Initial Velocity", initialVelocity);

      // 0.2x, 0.45y
      RebuiltFuelOnFly fuelOnFly = new RebuiltFuelOnFly(
          m_swerveSubstystem.getPose2d().getTranslation(),
          new Translation2d(0.2, 0),
          m_swerveSubstystem.getFieldVelocity(),
          m_swerveSubstystem.getPose2d().getRotation().plus(Rotation2d.fromDegrees(180)),
          Distance.ofBaseUnits(0.45, Meters),
          LinearVelocity.ofBaseUnits(initialVelocity, MetersPerSecond), // V sub 0 = sqrt(x^2/(2s)^2 + (72 in + ((0.5)(9.8)((2s)^2))^2)/(2s)^2)
          Angle.ofBaseUnits(-30, Degrees));

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
  }

  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
