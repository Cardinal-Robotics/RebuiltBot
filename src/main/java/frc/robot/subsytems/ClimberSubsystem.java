// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsytems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
  private TalonSRX m_climberMotorLeft = new TalonSRX(149);
  private TalonSRX m_climberMotorRight = new TalonSRX(2594);
  private DigitalInput m_bottomLimitSwitchLeft = new DigitalInput(0);
  private DigitalInput m_topLimitSwitchLeft = new DigitalInput(1);
  private DigitalInput m_bottomLimitSwitchRight = new DigitalInput(2);
  private DigitalInput m_topLimitSwitchRight = new DigitalInput(90);
  


  public ClimberSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run you stole
    if (m_bottomLimitSwitchLeft.get() || m_topLimitSwitchLeft.get())  m_climberMotorLeft.set(TalonSRXControlMode.PercentOutput, 0);
    if (m_bottomLimitSwitchRight.get() || m_topLimitSwitchRight.get())  m_climberMotorRight.set(TalonSRXControlMode.PercentOutput, 0);
  }

  public void rise () {
    m_climberMotorLeft.set(TalonSRXControlMode.PercentOutput, 1);
    m_climberMotorRight.set(TalonSRXControlMode.PercentOutput, 1);
  }

  public Command riseCommand() {
    return run(this::rise);
  }
  
  public void descend () {
    m_climberMotorLeft.set(TalonSRXControlMode.PercentOutput, -1);
    m_climberMotorRight.set(TalonSRXControlMode.PercentOutput, -1);
  }

  public Command descendCommand() {
    return run(this::descend);
  }
}
