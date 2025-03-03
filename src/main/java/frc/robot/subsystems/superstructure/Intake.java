// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
    intakeMotor = new TalonFX(00, "GTX7130"); // i dont know devise id

    // Configure TalonFX motors for angle control
    TalonFXConfiguration leftAngleConfig = new TalonFXConfiguration();
    leftAngleConfig.Voltage.PeakForwardVoltage = 12.0;
    leftAngleConfig.Voltage.PeakReverseVoltage = -12.0;
    leftAngleConfig.CurrentLimits.SupplyCurrentLimit = 40;
    leftAngleConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    
    TalonFXConfiguration rightAngleConfig = new TalonFXConfiguration();
    rightAngleConfig.Voltage.PeakForwardVoltage = 12.0;
    rightAngleConfig.Voltage.PeakReverseVoltage = -12.0;
    rightAngleConfig.CurrentLimits.SupplyCurrentLimit = 40;
    rightAngleConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    rightAngleConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // Configure TalonFX for intake motor
    TalonFXConfiguration intakeMotorConfig = new TalonFXConfiguration();
    intakeMotorConfig.Voltage.PeakForwardVoltage = 12.0;
    intakeMotorConfig.Voltage.PeakReverseVoltage = -12.0;
    intakeMotorConfig.CurrentLimits.SupplyCurrentLimit = 20; // 20A current limit
    intakeMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    intakeMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // 假設與原SparkMax相同方向

    // Apply configurations
    leftAngle.getConfigurator().apply(leftAngleConfig);
    rightAngle.getConfigurator().apply(rightAngleConfig);
    intakeMotor.getConfigurator().apply(intakeMotorConfig);

    // Set neutral modes
    leftAngle.setNeutralMode(NeutralModeValue.Brake);
    rightAngle.setNeutralMode(NeutralModeValue.Brake);
    intakeMotor.setNeutralMode(NeutralModeValue.Coast); // Coast mode for intake

    // Configure CANcoder
    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
    angleEncoder.getConfigurator().apply(encoderConfig);

    // Configure PID controller
    pidController.disableContinuousInput();
    pidController.setIntegratorRange(0,0);
    pidController.setGoal(angleEncoder.getAbsolutePosition().getValueAsDouble());
    pidController.calculate(angleEncoder.getAbsolutePosition().getValueAsDouble());
    pidController.setTolerance(0.05);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("intake pid", pidController.calculate(angleEncoder.getAbsolutePosition().getValueAsDouble()));
    SmartDashboard.putNumber("intake pid setpoint", pidController.getSetpoint().position);
  }

  public void moveAngle(double speed) {
    leftAngle.set(speed);
    rightAngle.set(speed);
  }

  public void setIntake(double speed) {
    intakeMotor.set(speed);
  }

  public void setAngle(double position) {
    pidController.setGoal(position);
    double output = MathUtil.clamp(pidController.calculate(angleEncoder.getAbsolutePosition().getValueAsDouble()), -0.1, 0.1);
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