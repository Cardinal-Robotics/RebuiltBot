// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerSubsystem extends SubsystemBase {

  private TalonSRX m_indexerMotor = new TalonSRX(2);

  public IndexerSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void spinIndexer() {
    m_indexerMotor.set(TalonSRXControlMode.PercentOutput, .5);
  }

  public Command spinIndexerCommand() {
    return run(() -> spinIndexer());
  }
}
