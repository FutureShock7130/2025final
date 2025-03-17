// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.Map;

public class AlgaeRemover extends SubsystemBase {
  // Motor constants
  private static final int LEFT_MOTOR_CAN_ID = 61; // Left motor CAN ID
  private static final int RIGHT_MOTOR_CAN_ID = 62; // Right motor CAN ID
  private static final double FORWARD_SOFT_LIMIT = 14.5; // Maximum forward rotation limit
  private static final double REVERSE_SOFT_LIMIT = 0.87; // Minimum reverse rotation limit
  private static final double DEFAULT_SPEED = 0.2; // Default speed for button control
  private static final double DEFAULT_DURATION = 2.0; // Default duration in seconds
  
  // Motors
  private final SparkMax leftMotor;
  private final SparkMax rightMotor;
  
  // Dashboard
  private final ShuffleboardTab algaeTab = Shuffleboard.getTab("AlgaeRemover");
  private final GenericEntry leftPositionEntry;
  private final GenericEntry rightPositionEntry;
  private final GenericEntry leftSpeedEntry;
  private final GenericEntry rightSpeedEntry;
  private final GenericEntry customSpeedEntry;
  private final GenericEntry timeDurationEntry;
  private static AlgaeRemover mInstance = null;

  public static synchronized AlgaeRemover getInstance() {
    if (mInstance == null) {
      mInstance = new AlgaeRemover();
    }
    return mInstance;
  }
  
  /** Creates a new AlgaeRemover. */
  public AlgaeRemover() {
    // Initialize motors
    leftMotor = new SparkMax(LEFT_MOTOR_CAN_ID, MotorType.kBrushless);
    rightMotor = new SparkMax(RIGHT_MOTOR_CAN_ID, MotorType.kBrushless);
    
    // Configure motors
    
    configureNEO(leftMotor, true, true);
    configureNEO(rightMotor, false, true); // Right motor is inverted
    
    // Setup dashboard entries for both motors
    algaeTab.addString("Motor Status", () -> "Left: " + LEFT_MOTOR_CAN_ID + " | Right: " + RIGHT_MOTOR_CAN_ID)
        .withPosition(0, 0)
        .withSize(4, 1);
    
    leftPositionEntry = algaeTab.add("Left Position", 0.0)
        .withPosition(0, 1)
        .withSize(2, 1)
        .getEntry();
    
    rightPositionEntry = algaeTab.add("Right Position", 0.0)
        .withPosition(2, 1)
        .withSize(2, 1)
        .getEntry();
    
    leftSpeedEntry = algaeTab.add("Left Speed", 0.0)
        .withPosition(0, 2)
        .withSize(2, 1)
        .getEntry();
    
    rightSpeedEntry = algaeTab.add("Right Speed", 0.0)
        .withPosition(2, 2)
        .withSize(2, 1)
        .getEntry();
    
    // Add a slider to set custom speed (0.0 to 1.0)
    customSpeedEntry = algaeTab.add("Custom Speed", DEFAULT_SPEED)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0.0, "max", 1.0, "block increment", 0.05))
        .withPosition(0, 3)
        .withSize(4, 1)
        .getEntry();
    
    // Add time duration slider (0.5 to 10.0 seconds)
    timeDurationEntry = algaeTab.add("Duration (s)", DEFAULT_DURATION)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0.5, "max", 10.0, "block increment", 0.5))
        .withPosition(0, 4)
        .withSize(4, 1)
        .getEntry();
    
    // Left motor controls
    algaeTab.add("Left Forward", getLeftForwardCommand())
        .withWidget(BuiltInWidgets.kCommand)
        .withProperties(Map.of("Label", "Left Forward ▶"))
        .withPosition(0, 5)
        .withSize(1, 1);
    
    algaeTab.add("Left Backward", getLeftBackwardCommand())
        .withWidget(BuiltInWidgets.kCommand)
        .withProperties(Map.of("Label", "Left Backward ◀"))
        .withPosition(1, 5)
        .withSize(1, 1);
    
    // Right motor controls
    algaeTab.add("Right Forward", getRightForwardCommand())
        .withWidget(BuiltInWidgets.kCommand)
        .withProperties(Map.of("Label", "Right Forward ▶"))
        .withPosition(2, 5)
        .withSize(1, 1);
    
    algaeTab.add("Right Backward", getRightBackwardCommand())
        .withWidget(BuiltInWidgets.kCommand)
        .withProperties(Map.of("Label", "Right Backward ◀"))
        .withPosition(3, 5)
        .withSize(1, 1);
    
    // Combined controls
    algaeTab.add("Both Forward", getBothForwardCommand())
        .withWidget(BuiltInWidgets.kCommand)
        .withProperties(Map.of("Label", "Both Forward ▶▶"))
        .withPosition(0, 6)
        .withSize(2, 1);
    
    algaeTab.add("Both Backward", getBothBackwardCommand())
        .withWidget(BuiltInWidgets.kCommand)
        .withProperties(Map.of("Label", "Both Backward ◀◀"))
        .withPosition(2, 6)
        .withSize(2, 1);
    
    algaeTab.add("Stop All", getStopCommand())
        .withWidget(BuiltInWidgets.kCommand)
        .withProperties(Map.of("Label", "STOP ALL ■"))
        .withPosition(0, 7)
        .withSize(4, 1);
    
    algaeTab.add("Reset Position", getResetCommand())
        .withWidget(BuiltInWidgets.kCommand)
        .withProperties(Map.of("Label", "Reset Encoders"))
        .withPosition(0, 8)
        .withSize(4, 1);
    
    // Add status indicators
    algaeTab.addBoolean("Left Running", () -> Math.abs(leftMotor.get()) > 0.01)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("colorWhenTrue", "Lime", "colorWhenFalse", "Red"))
        .withPosition(4, 1)
        .withSize(1, 1);
    
    algaeTab.addBoolean("Right Running", () -> Math.abs(rightMotor.get()) > 0.01)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("colorWhenTrue", "Lime", "colorWhenFalse", "Red"))
        .withPosition(4, 2)
        .withSize(1, 1);
  }
  
  /**
   * Command to run the left motor forward
   */
  public Command getLeftForwardCommand() {
    return Commands.runEnd(
        // Run action
        () -> setLeftSpeed(customSpeedEntry.getDouble(DEFAULT_SPEED)),
        // End action
        this::stopLeft,
        // Requirements
        this
    );
  }
  
  /**
   * Command to run the left motor backward
   */
  public Command getLeftBackwardCommand() {
    return Commands.runEnd(
        // Run action
        () -> setLeftSpeed(-customSpeedEntry.getDouble(DEFAULT_SPEED)),
        // End action
        this::stopLeft,
        // Requirements
        this
    );
  }
  
  /**
   * Command to run the right motor forward
   */
  public Command getRightForwardCommand() {
    return Commands.runEnd(
        // Run action
        () -> setRightSpeed(customSpeedEntry.getDouble(DEFAULT_SPEED)),
        // End action
        this::stopRight,
        // Requirements
        this
    );
  }
  
  /**
   * Command to run the right motor backward
   */
  public Command getRightBackwardCommand() {
    return Commands.runEnd(
        // Run action
        () -> setRightSpeed(-customSpeedEntry.getDouble(DEFAULT_SPEED)),
        // End action
        this::stopRight,
        // Requirements
        this
    );
  }
  
  /**
   * Command to run both motors forward
   */
  public Command getBothForwardCommand() {
    return Commands.runEnd(
        // Run action
        () -> setSpeed(customSpeedEntry.getDouble(DEFAULT_SPEED)),
        // End action
        this::stop,
        // Requirements
        this
    );
  }
  
  /**
   * Command to run both motors backward
   */
  public Command getBothBackwardCommand() {
    return Commands.runEnd(
        // Run action
        () -> setSpeed(-customSpeedEntry.getDouble(DEFAULT_SPEED)),
        // End action
        this::stop,
        // Requirements
        this
    );
  }
  
  /**
   * Command to stop both motors
   */
  private Command getStopCommand() {
    return Commands.runOnce(this::stop, this);
  }
  
  /**
   * Command to reset both encoder positions
   */
  private Command getResetCommand() {
    return Commands.runOnce(this::resetPosition, this);
  }
  
  /**
   * Configures a NEO motor with soft limits
   * 
   * @param motor The SparkMax motor controller to configure
   * @param inverted Whether the motor should be inverted
   * @param softLimitEnabled Whether soft limits should be enabled
   */
  private void configureNEO(SparkMax motor, boolean inverted, boolean softLimitEnabled) {
    SparkMaxConfig neoConfig = new SparkMaxConfig();
    
    // Create soft limit configuration
    SoftLimitConfig softLimitConfig = new SoftLimitConfig();
    softLimitConfig
        .forwardSoftLimit(FORWARD_SOFT_LIMIT)
        .forwardSoftLimitEnabled(softLimitEnabled)
        .reverseSoftLimit(REVERSE_SOFT_LIMIT)
        .reverseSoftLimitEnabled(softLimitEnabled);
    
    // Apply all motor configurations
    neoConfig
        .smartCurrentLimit(30) 
        .secondaryCurrentLimit(50)
        .idleMode(IdleMode.kBrake)  
        .voltageCompensation(12.0)
        .openLoopRampRate(0.1) // Seconds from 0 to full throttle
        .apply(softLimitConfig)
        .inverted(inverted)
        .disableFollowerMode();
    
    // Apply configuration to motor
    motor.setCANTimeout(250);
    motor.configure(neoConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    motor.getEncoder().setPosition(0.0);  // Reset encoder to zero
  }
  
  /**
   * Sets the left motor speed
   * 
   * @param speed Speed from -1.0 to 1.0
   */
  public void setLeftSpeed(double speed) {
    leftMotor.set(speed);
  }
  
  /**
   * Sets the right motor speed
   * 
   * @param speed Speed from -1.0 to 1.0
   */
  public void setRightSpeed(double speed) {
    rightMotor.set(speed);
  }
  
  /**
   * Sets both algae remover motor speeds to the same value
   * 
   * @param speed Speed from -1.0 to 1.0
   */
  public void setSpeed(double speed) {
    setLeftSpeed(speed);
    setRightSpeed(speed);
  }
  
  /**
   * Set left motor voltage directly
   * 
   * @param voltage Voltage to apply
   */
  public void setLeftVoltage(double voltage) {
    leftMotor.setVoltage(voltage);
  }
  
  /**
   * Set right motor voltage directly
   * 
   * @param voltage Voltage to apply
   */
  public void setRightVoltage(double voltage) {
    rightMotor.setVoltage(voltage);
  }
  
  /**
   * Set both motor voltages to the same value
   * 
   * @param voltage Voltage to apply
   */
  public void setVoltage(double voltage) {
    setLeftVoltage(voltage);
    setRightVoltage(voltage);
  }
  
  /**
   * Get the current position of the left motor in rotations
   * 
   * @return Current position in rotations
   */
  public double getLeftPosition() {
    return leftMotor.getEncoder().getPosition();
  }
  
  /**
   * Get the current position of the right motor in rotations
   * 
   * @return Current position in rotations
   */
  public double getRightPosition() {
    return rightMotor.getEncoder().getPosition();
  }
  
  /**
   * Reset the encoder positions to zero
   */
  public void resetPosition() {
    leftMotor.getEncoder().setPosition(0.0);
    rightMotor.getEncoder().setPosition(0.0);
  }
  
  /**
   * Stop the left motor
   */
  public void stopLeft() {
    leftMotor.set(0);
  }
  
  /**
   * Stop the right motor
   */
  public void stopRight() {
    rightMotor.set(0);
  }
  
  /**
   * Stop both motors
   */
  public void stop() {
    stopLeft();
    stopRight();
  }

  @Override
  public void periodic() {
    // Update dashboard with both motor info
    leftPositionEntry.setDouble(getLeftPosition());
    rightPositionEntry.setDouble(getRightPosition());
    leftSpeedEntry.setDouble(leftMotor.get());
    rightSpeedEntry.setDouble(rightMotor.get());
  }

  /**
   * Runs both motors at the specified speed for a set duration
   * 
   * @param speed Speed to run the motors (-1.0 to 1.0)
   * @param durationSeconds Time to run in seconds
   * @return Command that can be scheduled
   */
  public Command runForTimeCommand(double speed, double durationSeconds) {
    return Commands.sequence(
        // First start the motors
        Commands.runOnce(() -> setSpeed(speed), this),
        // Wait for the specified duration
        Commands.waitSeconds(durationSeconds),
        // Then stop the motors
        Commands.runOnce(this::stop, this)
    );
  }
  
  /**
   * Runs a specific motor for the specified duration
   * 
   * @param isLeft Whether to run the left motor (true) or right motor (false)
   * @param speed Speed to run (-1.0 to 1.0)
   * @param durationSeconds Duration in seconds
   * @return Command that can be scheduled
   */
  public Command runMotorForTimeCommand(boolean isLeft, double speed, double durationSeconds) {
    return Commands.sequence(
        // First start the specified motor
        Commands.runOnce(() -> {
          if (isLeft) {
            setLeftSpeed(speed);
          } else {
            setRightSpeed(speed);
          }
        }, this),
        // Wait for the specified duration
        Commands.waitSeconds(durationSeconds),
        // Then stop the motor
        Commands.runOnce(() -> {
          if (isLeft) {
            stopLeft();
          } else {
            stopRight();
          }
        }, this)
    );
  }
  
  /**
   * Runs both motors for the specified duration and automatically schedules the command
   * 
   * @param speed Speed to run (-1.0 to 1.0)
   * @param durationSeconds Duration in seconds
   */
  public void runForTime(double speed, double durationSeconds) {
    runForTimeCommand(speed, durationSeconds).schedule();
  }
}
