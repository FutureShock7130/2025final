// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import java.util.Map;

public class Elevator extends SubsystemBase {
  private final SparkMax leftMotor;
  private final SparkMax rightMotor;
  
  private static final double kDownSpeedMultiplier = 1; // Reduces speed when decending

  // Shuffleboard entries
  private final ShuffleboardTab elevatorTab = Shuffleboard.getTab("Elevator");


  // Add these with other instance variables at the top
  private final GenericEntry speedEntry;
  private final GenericEntry leftRotationsEntry;
  private final GenericEntry rightRotationsEntry;


  // Profiled PID Controller for smooth motionS
  private final TrapezoidProfile.Constraints constraints = 
      new TrapezoidProfile.Constraints(
          150,   
          150
      );
  
  private final ProfiledPIDController pidController = 
      new ProfiledPIDController(
          0.06,   // P gain
          0.0,   // I gain
          0.0,   // D gain
          constraints
      );
  
  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.5, 2.307, 0.05);

  private static Elevator mInstance = null;

  public static synchronized Elevator getInstance() {
    if (mInstance == null) {
      mInstance = new Elevator();
    }
    return mInstance;
  }

  /** Creates a new Elevator. */
  public Elevator() {
    leftMotor = new SparkMax(25, MotorType.kBrushless);  // Update ID as needed
    rightMotor = new SparkMax(26, MotorType.kBrushless); // Update ID as needed
    
    
    configureNEO(leftMotor, false,true);  //master ccw positive
    configureNEO(rightMotor, true,true);  //slave cw positive
    
    

    // Configure PID Controller
    pidController.setTolerance(2); 
    pidController.setIZone(Double.POSITIVE_INFINITY);
    pidController.setIntegratorRange(-0.5, 0.5);
    pidController.setGoal(leftMotor.getEncoder().getPosition());
    pidController.calculate(leftMotor.getEncoder().getPosition());
    pidController.reset(leftMotor.getEncoder().getPosition());

    //widgets
    speedEntry = elevatorTab.add("Elevator Speed", 0.0)
        .withPosition(0, 1)
        .withSize(2, 1)
        .getEntry();
    leftRotationsEntry = elevatorTab.add("Left Motor Rotations", 0.0)
        .withPosition(1, 1)
        .withSize(2, 1)
        .getEntry();
    rightRotationsEntry = elevatorTab.add("Right Motor Rotations", 0.0)
        .withPosition(1, 2)
        .withSize(2, 1)
        .getEntry();
  }

  private void configureNEO(SparkMax motor, boolean inverted, boolean softLimit) {
    SparkMaxConfig neoConfig = new SparkMaxConfig();
    
    // Create soft limit config for elevator
    SoftLimitConfig softLimitConfig = new SoftLimitConfig();
    softLimitConfig
        .forwardSoftLimit(198 * 0.6)     // in rotations
        .forwardSoftLimitEnabled(softLimit)
        .reverseSoftLimit(0.0)     
        .reverseSoftLimitEnabled(softLimit);
    
    neoConfig
        .smartCurrentLimit(50)
        .secondaryCurrentLimit(70)
        .idleMode(IdleMode.kBrake)  
        .voltageCompensation(12.0)
        .openLoopRampRate(0.1)
        .apply(softLimitConfig)
        .inverted(inverted)
        .disableFollowerMode();

    // if (motor == rightMotor) {
    //     neoConfig.follow(leftMotor, true);
    // }
    
    motor.setCANTimeout(250);
    motor.configure(neoConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    motor.getEncoder().setPosition(0.0);  // Reset encoder to zero
  }



  /** 
   * Sets the elevator speed. Positive values move up, negative values move down.
   * Includes gravity compensation when moving up and speed reduction when moving down! ^w^
   * @param speed Speed from -1.0 to 1.0
   */
  public void setElevatorSpeed(double speed) {
    double gravityCompensation = 0;
    
    // Reduce speed when moving down
    if (speed < 0) {
      speed *= kDownSpeedMultiplier;
    }
    
    leftMotor.set(speed + gravityCompensation);
    rightMotor.set(speed + gravityCompensation);
  }

  public void setVoltage(double voltagePercent) {
    double speed = MathUtil.clamp(voltagePercent, -1, 1);
    double ff = feedforward.calculate(speed);
    double output = (speed * 12) + ff;

    // Reduce speed when moving down
    if (output < 0) {
      output *= kDownSpeedMultiplier;
    }
    leftMotor.setVoltage(output);
    rightMotor.setVoltage(output);
  }

  public void setleftVoltage(double voltagePercent) {
    double speed = MathUtil.clamp(voltagePercent, -1, 1);
    double ff = feedforward.calculate(speed);
    double output = (speed * 12) + ff;

    // Reduce speed when moving down
    if (output < 0) {
      output *= kDownSpeedMultiplier;
    }
    leftMotor.setVoltage(output);
  }

  public void setrightVoltage(double voltagePercent) {
    double speed = MathUtil.clamp(voltagePercent, -1, 1);
    double ff = feedforward.calculate(speed);
    double output = (speed * 12) + ff;

    // Reduce speed when moving down
    if (output < 0) {
      output *= kDownSpeedMultiplier;
    }
    rightMotor.setVoltage(output);
  }

  /**
   * Stop the elevator uwu
   */
  public void stop() {
    setElevatorSpeed(0.0);
  }

    public void setPosition(double position) {
    pidController.setGoal(position);
    setleftVoltage(MathUtil.clamp(pidController.calculate(leftMotor.getEncoder().getPosition()) * 1.0, -0.9, 0.9));
    setrightVoltage(MathUtil.clamp(pidController.calculate(rightMotor.getEncoder().getPosition()) * 1.0, -0.9, 0.9));
  }


  public double getElevatorPosition() {
    return leftMotor.getEncoder().getPosition();
  }


  public boolean atTargetPosition() {
    return pidController.atGoal();
  }

  @Override
  public void periodic() {
    // Check for negative position and reset if needed
    if (leftMotor.getEncoder().getPosition() < 0) {
      leftMotor.getEncoder().setPosition(0.0);
    }
    if (rightMotor.getEncoder().getPosition() < 0) {
      rightMotor.getEncoder().setPosition(0.0);
    }


    // Update values instead of creating new widgets
    speedEntry.setDouble(leftMotor.get());
    SmartDashboard.putNumber("elevator applied output", leftMotor.getAppliedOutput());
    SmartDashboard.putNumber("elevator get", leftMotor.get());
    SmartDashboard.putNumber("elevator volts", leftMotor.getBusVoltage());
    SmartDashboard.putNumber("Elevator pid", pidController.calculate(rightMotor.getEncoder().getPosition()));
    SmartDashboard.putNumber("pid setpont le", pidController.getSetpoint().position);
    leftRotationsEntry.setDouble(leftMotor.getEncoder().getPosition());
    rightRotationsEntry.setDouble(rightMotor.getEncoder().getPosition());

    // Track maximum rotations
    double leftRotations = Math.abs(leftMotor.getEncoder().getPosition());
    double rightRotations = Math.abs(rightMotor.getEncoder().getPosition());

  }
}


    
    

