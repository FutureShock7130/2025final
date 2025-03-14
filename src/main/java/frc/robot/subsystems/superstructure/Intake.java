// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.compound.Diff_MotionMagicDutyCycle_Velocity;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Map;


public class Intake extends SubsystemBase {

  private final TalonFX leftAngle;
  private final TalonFX rightAngle;

  private final CANcoder angleEncoder;

  private final TalonFX intakeMotor;

  private final TrapezoidProfile.Constraints constraints =
  new TrapezoidProfile.Constraints(
    0.1,
    0.1
  );

  private final ProfiledPIDController pidController =
  new ProfiledPIDController(
    1.2,
    0.02,
    0.001,
    constraints
  );

  private final ArmFeedforward intakeFF =
  new ArmFeedforward(
    0.01,
    0.2,
    0.1
  );

  private final DynamicMotionMagicVoltage magic =
  new DynamicMotionMagicVoltage(
    0.0,
    0.0,
    0.0,
    0.0
  );


  // Add with other instance variables
  private final ShuffleboardTab intakeTab;


  private static Intake mInstance = null;
  public static synchronized Intake getInstance() {
    if (mInstance == null) {
        mInstance = new Intake();
    }
    return mInstance;
}

  
  /** Creates a new Intake. */
  public Intake() {
    // Create the tab first, before any other initialization
    intakeTab = Shuffleboard.getTab("Intake");
    
    // Then initialize motors
    leftAngle = new TalonFX(17, "GTX7130");
    rightAngle = new TalonFX(18, "GTX7130");
    angleEncoder = new CANcoder(4, "rio");
    intakeMotor = new TalonFX(45, "rio");
    // Configure TalonFX motors
    TalonFXConfiguration leftangleConfig = new TalonFXConfiguration();
    leftangleConfig.Voltage.PeakForwardVoltage = 12.0;
    leftangleConfig.Voltage.PeakReverseVoltage = -12.0;
    leftangleConfig.CurrentLimits.SupplyCurrentLimit = 40;
    leftangleConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    
    TalonFXConfiguration rightangleConfig = new TalonFXConfiguration();
    rightangleConfig.Voltage.PeakForwardVoltage = 12.0;
    rightangleConfig.Voltage.PeakReverseVoltage = -12.0;
    rightangleConfig.CurrentLimits.SupplyCurrentLimit = 40;
    rightangleConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    rightangleConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    

    // var slot0config = angleConfig.Slot0;
    // slot0config.kP = 0.1;
    // slot0config.kI = 0.0;
    // slot0config.kD = 0.0;
    // slot0config.kG = 0.56;
    // slot0config.kS = 0.0;
    // slot0config.kV = 1.62;
    // slot0config.kA = 0.03;

    leftAngle.getConfigurator().apply(leftangleConfig);
    rightAngle.getConfigurator().apply(rightangleConfig);
    intakeMotor.getConfigurator().apply(rightangleConfig);

    leftAngle.setNeutralMode(NeutralModeValue.Brake);
    rightAngle.setNeutralMode(NeutralModeValue.Brake);
    intakeMotor.setNeutralMode(NeutralModeValue.Brake);

    

    // rightAngle.setControl(new Follower(17, true));
    
    // Configure CANcoder
    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0;
    encoderConfig.MagnetSensor.MagnetOffset = -0.2;
    angleEncoder.getConfigurator().apply(encoderConfig);

    // // Configure SparkMax
    // SparkMaxConfig neo550Config = new SparkMaxConfig();


    // neo550Config
    //     .smartCurrentLimit(25)  
    //     .idleMode(IdleMode.kCoast)  
    //     .voltageCompensation(12.0)  
    //     .openLoopRampRate(0.1)
    //     .inverted(true);
    
    // // Apply our configuration with proper timeout
    // intakeMotor.setCANTimeout(250);
    // intakeMotor.configure(neo550Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    pidController.disableContinuousInput();
    pidController.setIntegratorRange(0,0);
    pidController.setGoal(angleEncoder.getAbsolutePosition().getValueAsDouble());
    pidController.calculate(angleEncoder.getAbsolutePosition().getValueAsDouble());
    pidController.setTolerance(0.05);
  }

  @Override
  public void periodic() {
    // System.out.println("intake pos: " + angleEncoder.getAbsolutePosition().getValueAsDouble());
    SmartDashboard.putNumber("intake pid", pidController.calculate(angleEncoder.getAbsolutePosition().getValueAsDouble()));
    SmartDashboard.putNumber("intake pid setpoint", pidController.getSetpoint().position);
  }

  public void moveAngle(double speed) {
    // magic.Velocity = speed;
    // leftAngle.setControl(magic);
    // leftAngle.set(speed + intakeFF.calculate(angleEncoder.getAbsolutePosition().getValueAsDouble(), speed))
    leftAngle.set(speed);
    rightAngle.set(speed);
  }

  public void setIntake(double speed) {
    intakeMotor.set(speed);
  }

  //0.36101cs
  public void setAngle(double position) {
    pidController.setGoal(position);
    double output = MathUtil.clamp(pidController.calculate(angleEncoder.getAbsolutePosition().getValueAsDouble()), -0.1, 0.1);
    // output += intakeFF.calculate(position, output);
    setVoltage(output);
  }

  public void setVoltage(double voltagePercent) {
    double speed = MathUtil.clamp(voltagePercent, -1, 1);
    double output = (speed * 12) + intakeFF.calculate(angleEncoder.getAbsolutePosition().getValueAsDouble(), speed);
    SmartDashboard.putNumber("intake voltage", output);
    
    leftAngle.setVoltage(output);
    rightAngle.setVoltage(output);
  }
}
