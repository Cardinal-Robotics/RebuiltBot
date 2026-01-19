// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsytems;

import java.util.List;

import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.gamepieces.GamePiece;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SimulationSubsystem extends SubsystemBase {
  SimulatedArena m_arena = SimulatedArena.getInstance();

  /** Creates a new SimulationSubsystem. */
  public SimulationSubsystem() {

    m_arena = SimulatedArena.getInstance();
    m_arena.placeGamePiecesOnField();

  }

  @Override
  public void periodic() {
  
/*         List<GamePiece> algaeGamePieces = SimulatedArena.getInstance().getGamePiecesByType("Algae");
        List<GamePiece> coralGamePieces = SimulatedArena.getInstance().getGamePiecesByType("Coral");
        Logger.recordOutput("FieldSimulation/Algae", algaeGamePieces.toArray(Pose3d[]::new));
        Logger.recordOutput("FieldSimulation/Coral", coralGamePieces.toArray(Pose3d[]::new)); */

  }
}
