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

    // private final TalonFX leftAngle;
    private final TalonFX rightAngle;

    private final CANcoder angleEncoder;

    private final TalonFX intakeMotor;

    private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(
            0.2,
            0.2);

    private final ProfiledPIDController pidController = new ProfiledPIDController(
            1,
            0.02,
            0.001,
            constraints);

    private final ArmFeedforward intakeFF = new ArmFeedforward(
            0.1,
            0.2,
            0.5);

    private final DynamicMotionMagicVoltage magic = new DynamicMotionMagicVoltage(
            0.0,
            0.0,
            0.0,
            0.0);

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
        // leftAngle = new TalonFX(17, "GTX7130");
        rightAngle = new TalonFX(18, "rio");
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

        // leftAngle.getConfigurator().apply(leftangleConfig);
        rightAngle.getConfigurator().apply(rightangleConfig);
        intakeMotor.getConfigurator().apply(rightangleConfig);

        // leftAngle.setNeutralMode(NeutralModeValue.Brake);
        rightAngle.setNeutralMode(NeutralModeValue.Brake);
        intakeMotor.setNeutralMode(NeutralModeValue.Brake);

        // Configure CANcoder
        CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
        encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
        encoderConfig.MagnetSensor.MagnetOffset = -0.8566796875;
        angleEncoder.getConfigurator().apply(encoderConfig);

        pidController.disableContinuousInput();
        pidController.setIntegratorRange(0, 0);
        pidController.setGoal(angleEncoder.getAbsolutePosition().getValueAsDouble());
        pidController.calculate(angleEncoder.getAbsolutePosition().getValueAsDouble());
        pidController.setTolerance(0.05);
    }

    public void moveAngle(double speed) {
        // magic.Velocity = speed;
        // leftAngle.setControl(magic);
        // leftAngle.set(speed +
        // intakeFF.calculate(angleEncoder.getAbsolutePosition().getValueAsDouble(),
        // speed))
        // leftAngle.set(speed);
        rightAngle.set(speed);
    }

    public void setIntake(double speed) {
        intakeMotor.set(speed);
    }

    // 0.36101cs
    public void setAngle(double position) {
        pidController.setGoal(position);
        double output = MathUtil.clamp(pidController.calculate(angleEncoder.getAbsolutePosition().getValueAsDouble()),
                -0.1,
                0.1);
        // output += intakeFF.calculate(position, output);
        setVoltage(output);
    }

    public void setVoltage(double voltagePercent) {
        double speed = MathUtil.clamp(voltagePercent, -1, 1);
        double output = (speed * 12) + intakeFF.calculate(angleEncoder.getAbsolutePosition().getValueAsDouble(), speed);
        SmartDashboard.putNumber("intake voltage", output);

        // leftAngle.setVoltage(output);
        rightAngle.setVoltage(output);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("intake pid",
                pidController.calculate(angleEncoder.getAbsolutePosition().getValueAsDouble()));
        SmartDashboard.putNumber("intake pid setpoint", pidController.getSetpoint().position);
    }
}
