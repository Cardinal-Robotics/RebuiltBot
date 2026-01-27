// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsytems.ShooterSubstystem;
import frc.robot.subsytems.SwerveSubsystem;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.ArrayList;
import java.util.List;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.gamepieces.GamePiece;
import swervelib.simulation.ironmaple.simulation.gamepieces.GamePieceProjectile;
import swervelib.simulation.ironmaple.simulation.seasonspecific.crescendo2024.NoteOnFly;
import swervelib.simulation.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.*; // this is peak
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
    if(Robot.isSimulation()) { // "units are a man's worst enemy" - Charlie Malerich 1/26/2026
      RebuiltFuelOnFly fuelOnFly = new RebuiltFuelOnFly(
        m_swerveSubstystem.getPose2d().getTranslation(),
        new Translation2d(0.2, 0),
        m_swerveSubstystem.getFieldVelocity(),
        m_swerveSubstystem.getPose2d().getRotation().plus(Rotation2d.fromDegrees(180)),
        Distance.ofBaseUnits(0.45, Meters),
        LinearVelocity.ofBaseUnits(9.8 * 1.1, MetersPerSecond),
        Angle.ofBaseUnits(-30, Degrees)
      );

    fuelOnFly
        // Configure callbacks to visualize the flight trajectory of the projectile
        .withProjectileTrajectoryDisplayCallBack(
        // Callback for when the note will eventually hit the target (if configured)
        (pose3ds) -> Logger.recordOutput("Flywheel/FuelProjectileSuccessfulShot", pose3ds.toArray(Pose3d[]::new)),
        // Callback for when the note will eventually miss the target, or if no target is configured
        (pose3ds) -> Logger.recordOutput("Flywheel/FuelProjectileUnsuccessfulShot", pose3ds.toArray(Pose3d[]::new))
        );

      fuelOnFly.enableBecomesGamePieceOnFieldAfterTouchGround();

      SimulatedArena.getInstance().addGamePieceProjectile(fuelOnFly);
    }
  }

  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
