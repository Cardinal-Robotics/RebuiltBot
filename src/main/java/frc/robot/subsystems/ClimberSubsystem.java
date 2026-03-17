// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
  private TalonSRX m_climberMotorLeft = new TalonSRX(25);
  private TalonSRX m_climberMotorRight = new TalonSRX(26);
  private DigitalInput m_bottomLimitSwitchLeft = new DigitalInput(3);
  private DigitalInput m_bottomLimitSwitchRight = new DigitalInput(1);
  private Servo m_leftClimberServo = new Servo(0);
  private Servo m_rightClimberServo = new Servo(1);

  public ClimberSubsystem() {
    m_climberMotorLeft.setInverted(false);
    m_climberMotorRight.setInverted(false);
  }

  public boolean areBottomSwitchesPressed() {
    return m_bottomLimitSwitchLeft.get() && m_bottomLimitSwitchRight.get();
  }

  private final Timer leftSpikeTimer = new Timer();
  private double lastLeftCurrent = 0;
  public boolean isLeftClimberToppingOut() {
    double current = m_climberMotorLeft.getStatorCurrent();
    double slope = (current - lastLeftCurrent) / 0.02;
    lastLeftCurrent = current;

    // Find the current threshold the climber at stress uses & find the rate of change.
    if(slope > 0) {
      leftSpikeTimer.start();
    } else {
      leftSpikeTimer.reset();
    }

    // Try to minimize the time so the climber spends less time stressing out
    // while also preventing the initial current spike from false-alarming.
    if(leftSpikeTimer.get() > 0.15) {
      return true;
    }

    return false;
  }

  private final Timer rightSpikeTimer = new Timer();
  private double lastRightCurrent = 0;
  public boolean isRightClimberToppingOut() {
    double current = m_climberMotorRight.getStatorCurrent();
    double slope = current - lastRightCurrent;
    lastRightCurrent = current;

    // Find the current threshold the climber at stress uses & find the rate of change.
    if(slope > 0) {
      rightSpikeTimer.start();
    } else {
      rightSpikeTimer.reset();
    }

    // Try to minimize the time so the climber spends less time stressing out
    // while also preventing the initial current spike from false-alarming.
    if(rightSpikeTimer.get() > 0.15) {
      return true;
    }

    return false;
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Climber/BL", m_bottomLimitSwitchLeft.get());
    Logger.recordOutput("Climber/BR", m_bottomLimitSwitchRight.get());

    Logger.recordOutput("Climber/LeftCurrent", m_climberMotorLeft.getStatorCurrent());
    Logger.recordOutput("Climber/RightCurrent", m_climberMotorRight.getStatorCurrent());
    Logger.recordOutput("Climber/TopOutL", isLeftClimberToppingOut());
    Logger.recordOutput("Climber/TopOutR", isRightClimberToppingOut());
  }

  public void rise() { // NO RATCHET ON LEFT
    // if(!isLeftClimberToppingOut()) { m_climberMotorLeft.set(TalonSRXControlMode.PercentOutput, 1); }
    // else { m_climberMotorLeft.set(TalonSRXControlMode.PercentOutput, 0); }
    // if(!isRightClimberToppingOut()) { m_climberMotorRight.set(TalonSRXControlMode.PercentOutput, 1); }
    // else { m_climberMotorRight.set(TalonSRXControlMode.PercentOutput, 0); }

    m_climberMotorLeft.set(TalonSRXControlMode.PercentOutput, 1);
    m_climberMotorRight.set(TalonSRXControlMode.PercentOutput, 1);
  }

  public void descend() {
    if (!m_bottomLimitSwitchLeft.get()) { m_climberMotorLeft.set(TalonSRXControlMode.PercentOutput, -1); }
    else { m_climberMotorLeft.set(TalonSRXControlMode.PercentOutput, 0); }
    if (!m_bottomLimitSwitchRight.get()) { m_climberMotorRight.set(TalonSRXControlMode.PercentOutput, -1); }
    else { m_climberMotorRight.set(TalonSRXControlMode.PercentOutput, 0);}
  }

  public void stop() { // NO RATCHET ON LEFT
    m_climberMotorLeft.set(TalonSRXControlMode.PercentOutput, 0);
    m_climberMotorRight.set(TalonSRXControlMode.PercentOutput, 0);
  }

  public Command unlockServos() { // unlocked/pushed down: climbers can move up - locked/released/default:
                                  // climbers can go down, not up
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
