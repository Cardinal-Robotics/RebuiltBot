// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsytems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import swervelib.simulation.ironmaple.simulation.IntakeSimulation;
import swervelib.simulation.ironmaple.simulation.IntakeSimulation.IntakeSide;

public class IntakeSubsystem extends SubsystemBase {
  private IntakeSimulation m_intakeSimulation;
  private SwerveSubsystem m_swerveSubsystem;

  private DCMotor m_neoGearbox = DCMotor.getNEO(1);

  private SparkMax m_pivotMotor = new SparkMax(328, MotorType.kBrushless);
  private SparkMaxSim m_pivotMotorSim = new SparkMaxSim(m_pivotMotor, m_neoGearbox);
  private SingleJointedArmSim m_intakeArm = new SingleJointedArmSim(m_neoGearbox,
                15,
                SingleJointedArmSim.estimateMOI(0.5, 10),
                Meters.convertFrom(19, Inches),
                0,
                Math.toRadians(180),
                false,
                0,
                 0, 0);

  
  private TalonSRX m_intakeMotor = new TalonSRX(148);

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem(SwerveSubsystem swerveSubsystem) {
    this.m_swerveSubsystem = swerveSubsystem;

    SparkMaxConfig pivotConfig = new SparkMaxConfig();

    pivotConfig.idleMode(IdleMode.kBrake);
    pivotConfig.closedLoop.pid(2, 0.002, 0.005);
    pivotConfig.encoder.positionConversionFactor(360/36);

    m_pivotMotor.getEncoder().getPosition(); // 1/36 - 1 rotation of the arm is 36 of the motor 4:3:3

    m_pivotMotor.configure(pivotConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);



    // Following only runs in sim:
    if(!Robot.isSimulation()) return;
    this.m_intakeSimulation = IntakeSimulation.OverTheBumperIntake(
      "Fuel",
      m_swerveSubsystem.getSwerveDrive().getMapleSimDrive().get(),
      Meters.of(0.558),
      Meters.of(0.183),
      IntakeSide.BACK,
      100
    );
  }


  public void setTargetAngle(double angle) {
    m_pivotMotor.getClosedLoopController().setSetpoint(angle, ControlType.kPosition);
  }

  public void setIntakeSpeed(double speed) {
    m_intakeMotor.set(TalonSRXControlMode.PercentOutput, speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
