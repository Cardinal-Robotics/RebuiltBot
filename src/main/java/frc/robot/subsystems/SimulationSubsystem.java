// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import swervelib.simulation.ironmaple.simulation.gamepieces.GamePiece;
import swervelib.simulation.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import swervelib.simulation.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnField;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Inches;

import java.util.ArrayList;
import java.util.List;

public class SimulationSubsystem extends SubsystemBase {
  SimulatedArena m_arena = SimulatedArena.getInstance();

  protected static Translation2d blueDepotBottomRightCorner = new Translation2d(0.02, 5.53);
  protected static Translation2d redDepotBottomRightCorner = new Translation2d(16.0274, 1.646936);

  public SimulationSubsystem() {
    if (!Robot.isSimulation())
      return;

    m_arena = SimulatedArena.getInstance();
    m_arena.placeGamePiecesOnField();
    SmartDashboard.putNumber("x", 0);
    SmartDashboard.putNumber("y", 0);
    SmartDashboard.putNumber("z", 0);
  }

  @Override
  public void simulationPeriodic() {
    double x = SmartDashboard.getNumber("x", 0);
    double y = SmartDashboard.getNumber("y", 0);
    double z = SmartDashboard.getNumber("z", 0);

    Logger.recordOutput("Tracker", new Pose3d(x, y, z, Rotation3d.kZero));

    m_arena.simulationPeriodic();
    List<GamePiece> fuelGamePieces = SimulatedArena.getInstance().getGamePiecesByType("Fuel");

    List<Pose3d> poses = new ArrayList<>();

    for (int i = 0; i < fuelGamePieces.size(); i++) {
      poses.add(i, fuelGamePieces.get(i).getPose3d());
    }

    Logger.recordOutput("FieldSimulation/Fuel", poses.toArray(Pose3d[]::new));
  }

  public static void resetField() {
    SimulatedArena.getInstance().clearGamePieces();
    SimulatedArena.getInstance().placeGamePiecesOnField();
    boolean isOnBlue = !DriverStation.getAlliance().isEmpty()
        && DriverStation.getAlliance().get() == Alliance.Blue;
    if (isOnBlue) {
      for (int x = 0; x < 4; x++) {
        for (int y = 0; y < 6; y++) {
          SimulatedArena.getInstance().addGamePiece(new RebuiltFuelOnField(blueDepotBottomRightCorner.plus(
              new Translation2d(Inches.of(5.991 * x), Inches.of(5.95 * y)))));
        }
      }
    }

    if (!isOnBlue) {
      for (int x = 0; x < 4; x++) {
        for (int y = 0; y < 6; y++) {
          SimulatedArena.getInstance().addGamePiece(new RebuiltFuelOnField(redDepotBottomRightCorner.plus(
              new Translation2d(Inches.of(5.991 * x), Inches.of(5.95 * y)))));
        }
      }
    }
  }

  public static Command resetFieldCommand() {
    return Commands.run(SimulationSubsystem::resetField);
  }
}
