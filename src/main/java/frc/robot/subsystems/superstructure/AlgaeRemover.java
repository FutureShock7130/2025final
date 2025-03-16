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
  private static final int MOTOR_CAN_ID = 62; // Update this to match your CAN ID
  private static final double FORWARD_SOFT_LIMIT = 14; // Maximum forward rotation limit
  private static final double REVERSE_SOFT_LIMIT = 1; // Minimum reverse rotation limit
  private static final double DEFAULT_SPEED = 0.02; // Default speed for button control
  private static final double DEFAULT_DURATION = 2.0; // Default duration in seconds
  
  // Motor
  private final SparkMax motor;
  
  // Dashboard
  private final ShuffleboardTab algaeTab = Shuffleboard.getTab("AlgaeRemover");
  private final GenericEntry motorPositionEntry;
  private final GenericEntry motorSpeedEntry;
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
    // Initialize motor
    motor = new SparkMax(MOTOR_CAN_ID, MotorType.kBrushless);
    
    // Configure motor
    configureNEO(motor, false, true);
    
    // Setup dashboard entries
    motorPositionEntry = algaeTab.add("Motor Position", 0.0)
        .withPosition(0, 0)
        .withSize(2, 1)
        .getEntry();
    
    motorSpeedEntry = algaeTab.add("Motor Speed", 0.0)
        .withPosition(0, 1)
        .withSize(2, 1)
        .getEntry();
    
    // Add a slider to set custom speed (0.0 to 1.0)
    customSpeedEntry = algaeTab.add("Custom Speed", DEFAULT_SPEED)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0.0, "max", 1.0, "block increment", 0.05))
        .withPosition(2, 0)
        .withSize(2, 1)
        .getEntry();
    
    // Add time duration slider (0.5 to 10.0 seconds)
    timeDurationEntry = algaeTab.add("Duration (s)", DEFAULT_DURATION)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0.5, "max", 10.0, "block increment", 0.5))
        .withPosition(0, 2)
        .withSize(2, 1)
        .getEntry();
    
    // Add control buttons
    algaeTab.add("Forward", getForwardCommand())
        .withWidget(BuiltInWidgets.kCommand)
        .withProperties(Map.of("Label", "Run Forward ▶"))
        .withPosition(2, 1)
        .withSize(1, 1);
    
    algaeTab.add("Backward", getBackwardCommand())
        .withWidget(BuiltInWidgets.kCommand)
        .withProperties(Map.of("Label", "Run Backward ◀"))
        .withPosition(3, 1)
        .withSize(1, 1);
    
    algaeTab.add("Stop", getStopCommand())
        .withWidget(BuiltInWidgets.kCommand)
        .withProperties(Map.of("Label", "STOP ■"))
        .withPosition(2, 2)
        .withSize(2, 1);
    
    algaeTab.add("Reset Position", getResetCommand())
        .withWidget(BuiltInWidgets.kCommand)
        .withProperties(Map.of("Label", "Reset Encoder"))
        .withPosition(4, 0)
        .withSize(1, 1);
    
    // Add status indicator
    algaeTab.addBoolean("Motor Running", () -> Math.abs(motor.get()) > 0.01)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("colorWhenTrue", "Lime", "colorWhenFalse", "Red"))
        .withPosition(4, 1)
        .withSize(1, 1);
    
    // Add timed operation buttons
    algaeTab.add("Timed Forward", getTimedForwardCommand())
        .withWidget(BuiltInWidgets.kCommand)
        .withProperties(Map.of("Label", "Forward ⏱"))
        .withPosition(2, 3)
        .withSize(1, 1);
    
    algaeTab.add("Timed Backward", getTimedBackwardCommand())
        .withWidget(BuiltInWidgets.kCommand)
        .withProperties(Map.of("Label", "Backward ⏱"))
        .withPosition(3, 3)
        .withSize(1, 1);
  }
  
  /**
   * Command to run the motor forward at the speed set by the slider
   */
  private Command getForwardCommand() {
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
   * Command to run the motor backward at the speed set by the slider
   */
  private Command getBackwardCommand() {
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
   * Command to stop the motor
   */
  private Command getStopCommand() {
    return Commands.runOnce(this::stop, this);
  }
  
  /**
   * Command to reset the encoder position
   */
  private Command getResetCommand() {
    return Commands.runOnce(this::resetPosition, this);
  }
  
  /**
   * Command to run the motor forward for a specific time duration
   */
  private Command getTimedForwardCommand() {
    return Commands.runOnce(() -> {
      // Get the current duration from slider
      double duration = timeDurationEntry.getDouble(DEFAULT_DURATION);
      double speed = customSpeedEntry.getDouble(DEFAULT_SPEED);
      runForTime(speed, duration);
    }, this);
  }
  
  /**
   * Command to run the motor backward for a specific time duration
   */
  private Command getTimedBackwardCommand() {
    return Commands.runOnce(() -> {
      // Get the current duration from slider
      double duration = timeDurationEntry.getDouble(DEFAULT_DURATION);
      double speed = customSpeedEntry.getDouble(DEFAULT_SPEED);
      runForTime(-speed, duration);
    }, this);
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
   * Sets the algae remover motor speed
   * 
   * @param speed Speed from -1.0 to 1.0
   */
  public void setSpeed(double speed) {
    motor.set(speed);
  }
  
  /**
   * Set motor voltage directly
   * 
   * @param voltage Voltage to apply
   */
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }
  
  /**
   * Get the current position of the motor in rotations
   * 
   * @return Current position in rotations
   */
  public double getPosition() {
    return motor.getEncoder().getPosition();
  }
  
  /**
   * Reset the encoder position to zero
   */
  public void resetPosition() {
    motor.getEncoder().setPosition(0.0);
  }
  
  /**
   * Stop the motor
   */
  public void stop() {
    motor.set(0);
  }

  @Override
  public void periodic() {
    // Update dashboard
    motorPositionEntry.setDouble(getPosition());
    motorSpeedEntry.setDouble(motor.get());
  }

  /**
   * Runs the motor at the specified speed for a set duration
   * 
   * @param speed Speed to run the motor (-1.0 to 1.0)
   * @param durationSeconds Time to run in seconds
   * @return Command that can be scheduled
   */
  public Command runForTimeCommand(double speed, double durationSeconds) {
    return Commands.sequence(
        // First start the motor
        Commands.runOnce(() -> setSpeed(speed), this),
        // Wait for the specified duration
        Commands.waitSeconds(durationSeconds),
        // Then stop the motor
        Commands.runOnce(this::stop, this)
    );
  }
  
  /**
   * Runs the motor for the specified duration and automatically schedules the command
   * 
   * @param speed Speed to run (-1.0 to 1.0)
   * @param durationSeconds Duration in seconds
   */
  public void runForTime(double speed, double durationSeconds) {
    runForTimeCommand(speed, durationSeconds).schedule();
  }
}
