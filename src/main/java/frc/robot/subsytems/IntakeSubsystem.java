// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsytems;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import swervelib.simulation.ironmaple.simulation.IntakeSimulation;
import swervelib.simulation.ironmaple.simulation.IntakeSimulation.IntakeSide;

public class IntakeSubsystem extends SubsystemBase {
  private IntakeSimulation m_intakeSimulation;
  private SwerveSubsystem m_swerveSubsystem;


  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem(SwerveSubsystem swerveSubsystem) {
    this.m_swerveSubsystem = swerveSubsystem;



    // Following only runs in sim:
    if(!Robot.isSimulation()) return;
    this.m_intakeSimulation = IntakeSimulation.OverTheBumperIntake(
      "Fuel",
      m_swerveSubsystem.getSwerveDrive().getMapleSimDrive().get(),
      Meters.of(5),
      Meters.of(5),
      IntakeSide.BACK,
      10000000);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
