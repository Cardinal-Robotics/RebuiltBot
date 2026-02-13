// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import swervelib.simulation.ironmaple.simulation.gamepieces.GamePiece;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import edu.wpi.first.math.geometry.Pose3d;

import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.List;

public class SimulationSubsystem extends SubsystemBase {
  SimulatedArena m_arena = SimulatedArena.getInstance();

  public SimulationSubsystem() {
    if(!Robot.isSimulation()) return;

    m_arena = SimulatedArena.getInstance();
    m_arena.placeGamePiecesOnField();

  }

  @Override
  public void simulationPeriodic() {
    List<GamePiece> fuelGamePieces = SimulatedArena.getInstance().getGamePiecesByType("Fuel");

    List<Pose3d> poses = new ArrayList<>();

    for (int i = 0; i < fuelGamePieces.size(); i++) {
      poses.add(i, fuelGamePieces.get(i).getPose3d());
    }

    Logger.recordOutput("FieldSimulation/Fuel", poses.toArray(Pose3d[]::new));
  }
}
