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
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import org.littletonrobotics.junction.Logger;

import frc.robot.Robot;
import frc.robot.subsystems.*;

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

  }

  @Override
  public void execute() {
        if (Robot.isSimulation()) { // "units are a man's worst enemy" - Charlie Malerich 1/26/2026
      double theta = Math.toRadians(65); // FIXED SHOOTER ANGLE

      double[] conditions = m_shooterSubstystem.getIdealShooterConditions();
      double w0 = conditions[0];
      double phi = conditions[1];

      if(Double.isNaN(w0) || Double.isNaN(phi)) return;

      Logger.recordOutput("Shooter/v0", w0);

      m_shooterSubstystem.setTargetSpeedRPM(w0);

      // 0.2x, 0.45y
      RebuiltFuelOnFly fuelOnFly = new RebuiltFuelOnFly(
          m_swerveSubstystem.getPose2d().getTranslation(),
          new Translation2d(0.2, 0),
          m_swerveSubstystem.getFieldVelocity(),
          new Rotation2d(phi),
          Distance.ofBaseUnits(0.45, Meters),
          LinearVelocity.ofBaseUnits(m_shooterSubstystem.getVelocityRPM() * (2 * Math.PI * Meters.convertFrom(2, Inches)) / 60, MetersPerSecond), // V sub 0 = sqrt(x^2/(2s)^2 + (72 in +
                                                           // ((0.5)(9.8)((2s)^2))^2)/(2s)^2)
          Angle.ofBaseUnits(theta, Radians));

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
