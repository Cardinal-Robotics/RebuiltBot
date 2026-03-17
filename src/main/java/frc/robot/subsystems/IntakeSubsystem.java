// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.ArrayList;
import java.util.function.Function;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import swervelib.simulation.ironmaple.simulation.IntakeSimulation;
import swervelib.simulation.ironmaple.simulation.IntakeSimulation.IntakeSide;

public class IntakeSubsystem extends SubsystemBase {
  private IntakeSimulation m_intakeSimulation;
  private SwerveSubsystem m_swerveSubsystem;

  private DCMotor m_neoGearbox = DCMotor.getNEO(1);
  private DutyCycleEncoder absoluteEncoder = new DutyCycleEncoder(0);

  private SparkMax m_pivotMotor = new SparkMax(32, MotorType.kBrushless);
  private SparkMaxSim m_pivotMotorSim = new SparkMaxSim(m_pivotMotor, m_neoGearbox);
  private SingleJointedArmSim m_intakeArmSim = new SingleJointedArmSim(m_neoGearbox,
      36.0,
      SingleJointedArmSim.estimateMOI(Meters.convertFrom(19, Inches), 1),
      Meters.convertFrom(19, Inches),
      0,
      Math.toRadians(180),
      false,
      0,
      0, 0);

  Mechanism2d mech = new Mechanism2d(0, 0);
  MechanismRoot2d root = mech.getRoot("ArmRoot", 0, .5);
  MechanismLigament2d armLigament = root.append(new MechanismLigament2d(
      "Arm",
      0.5, // length (meters)
      0 // starting angle
  ));

  private TalonSRX m_intakeMotor = new TalonSRX(27);

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem(SwerveSubsystem swerveSubsystem) {
    absoluteEncoder.setDutyCycleRange(0, 360);
    this.m_swerveSubsystem = swerveSubsystem;

    SparkMaxConfig pivotConfig = new SparkMaxConfig();

    pivotConfig.idleMode(IdleMode.kBrake);
    pivotConfig.inverted(false);

    pivotConfig.closedLoop.pid(2, 0, 0.005);
    if(Robot.isSimulation()) pivotConfig.closedLoop.pid(0.005, 0, 0);

    pivotConfig.closedLoop.minOutput(-0.1);
    pivotConfig.closedLoop.maxOutput(0.1);
    pivotConfig.closedLoop.maxMotion.cruiseVelocity(360);

        // 1/36 is 1 rotation, 360 is degrees for every rotation, 60 is seconds for every minute 
    pivotConfig.encoder.positionConversionFactor(360 / 36);
    pivotConfig.encoder.velocityConversionFactor(360.0f / 36.0f / 60.0f);
    m_pivotMotor.getEncoder().getPosition(); // 1/36 - 1 rotation of the arm is 36 of the motor 4:3:3

    m_pivotMotor.configure(pivotConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    m_intakeMotor.setInverted(InvertType.InvertMotorOutput);
    m_pivotMotor.getEncoder().setPosition(0);
  

    // Following only runs in sim:
    if (!Robot.isSimulation())
      return;
    this.m_intakeSimulation = IntakeSimulation.OverTheBumperIntake(
        "Fuel",
        m_swerveSubsystem.getSwerveDrive().getMapleSimDrive().get(),
        Meters.of(0.558),
        Meters.of(0.183),
        IntakeSide.BACK,
        40);

      this.m_intakeSimulation.setGamePiecesCount(8);
  }


  public void setIntakeSpeed(double speed) {
    m_intakeMotor.set(TalonSRXControlMode.PercentOutput, speed);
  }

  @Override
  public void periodic() {
    armLigament.setAngle(m_pivotMotor.getEncoder().getPosition());
    Logger.recordOutput("Intake/AbsoluteEncoder", absoluteEncoder.get());

    SmartDashboard.putData("Arm Sim", mech);
    Logger.recordOutput("Intake/Pivot", m_pivotMotor.getEncoder().getPosition());

    boolean hasRun = false;

    if (DriverStation.isEnabled() && !hasRun) {
      setIntakePivotCommand(0);
      hasRun = false;
    }

  }

  @Override
  public void simulationPeriodic() {
    m_intakeArmSim.setInput(m_pivotMotorSim.getAppliedOutput() * RobotController.getBatteryVoltage());
    m_pivotMotorSim.iterate(m_intakeArmSim.getVelocityRadPerSec(), RoboRioSim.getVInVoltage(), 0.020);
    m_intakeArmSim.update(0.020);

    m_pivotMotorSim.setPosition(Units.radiansToDegrees(m_intakeArmSim.getAngleRads()));

    if(m_pivotMotor.getClosedLoopController().getSetpoint() == 0 && m_intakeMotor.getMotorOutputPercent() > 0) {
      m_intakeSimulation.startIntake();
    } else m_intakeSimulation.stopIntake();

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_intakeArmSim.getCurrentDrawAmps()));
  }

  public void setIntakePivot(double angle) {    
    //m_pivotMotor.getClosedLoopController().setSetpoint(angle, ControlType.kPosition);
    Logger.recordOutput("Intake/IdealPivot", angle);
  }

  public Command nudgeForward() {
    return runEnd(() -> m_pivotMotor.set(0.025), () -> m_pivotMotor.set(0.0));
  }

  public Command setIntakePivotCommand(double angle) {
    SparkClosedLoopController pidController = m_pivotMotor.getClosedLoopController();
    return runOnce(() -> setIntakePivot(angle)/*  , pidController::isAtSetpoint */);
  }

  public Command runIntakeMotor(double speed) {
    return runOnce(() -> setIntakeSpeed(speed));
  }

  public void stopIntakeMotor() {
    setIntakeSpeed(0);
  }

  public Command stopIntakeCommand() {
    return runOnce(() -> stopIntakeMotor());
  }

  public boolean obtainGamePieceFromIntake() {
    return m_intakeSimulation.obtainGamePieceFromIntake();
  }
  
}
