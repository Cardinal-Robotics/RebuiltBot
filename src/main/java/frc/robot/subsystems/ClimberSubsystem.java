// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
  private TalonSRX m_climberMotorLeft = new TalonSRX(25);
  private TalonSRX m_climberMotorRight = new TalonSRX(26);
  private DigitalInput m_bottomLimitSwitchLeft = new DigitalInput(3);
  private DigitalInput m_topLimitSwitchLeft = new DigitalInput(2);
  private DigitalInput m_bottomLimitSwitchRight = new DigitalInput(1);
  private DigitalInput m_topLimitSwitchRight = new DigitalInput(0);
  private Servo m_leftClimberServo = new Servo(0);
  private Servo m_rightClimberServo = new Servo(1);

  public ClimberSubsystem() {
    m_climberMotorLeft.setInverted(false);
    m_climberMotorRight.setInverted(false);
  }

  public boolean areBottomSwitchesPressed() {
    return m_bottomLimitSwitchLeft.get() && m_bottomLimitSwitchRight.get();
  }

  public boolean areTopSwitchesPressed() {
    return m_topLimitSwitchLeft.get() && m_topLimitSwitchRight.get();
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("TL", m_topLimitSwitchLeft.get());
    SmartDashboard.putBoolean("BL", m_bottomLimitSwitchLeft.get());
    SmartDashboard.putBoolean("TR", m_topLimitSwitchRight.get());
    SmartDashboard.putBoolean("BR", m_bottomLimitSwitchRight.get());

    // This method will be called once per scheduler run you stole
    if (m_bottomLimitSwitchLeft.get() || m_topLimitSwitchLeft.get())
      m_climberMotorLeft.set(TalonSRXControlMode.PercentOutput, 0);
    if (m_bottomLimitSwitchRight.get() || m_topLimitSwitchRight.get())
      m_climberMotorRight.set(TalonSRXControlMode.PercentOutput, 0);
  }

  public void rise() { // NO RATCHET ON LEFT
    if(!m_topLimitSwitchLeft.get()) m_climberMotorLeft.set(TalonSRXControlMode.PercentOutput, 1);
    if(!m_topLimitSwitchRight.get()) m_climberMotorRight.set(TalonSRXControlMode.PercentOutput, 1);
  }
  
  public void descend() {
    if(!m_bottomLimitSwitchLeft.get()) m_climberMotorLeft.set(TalonSRXControlMode.PercentOutput, -1);
    if(!m_bottomLimitSwitchRight.get()) m_climberMotorRight.set(TalonSRXControlMode.PercentOutput, -1);
  }

  public void stop() { // NO RATCHET ON LEFT
    m_climberMotorLeft.set(TalonSRXControlMode.PercentOutput, 0);
    m_climberMotorRight.set(TalonSRXControlMode.PercentOutput, 0);
  }

  public Command unlockServos() { // unlocked/pushed down: climbers can move up - locked/released/default: climbers can go down, not up
    return runOnce(() -> {
      m_rightClimberServo.set(0);
    });
  }

  public Command lockServos() {
    return runOnce(() -> {
      m_rightClimberServo.set(0.4);
    });
  }

}
