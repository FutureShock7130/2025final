package frc.robot.subsystems.superstructure;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.GenericEntry;
import java.util.Map;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;

public class Grabber extends SubsystemBase {
    private final SparkMax rightIntake;
    private final SparkMax leftIntake;
    private final SparkMax rightangle;
    private final SparkMax leftangle;
    private final CANcoder grabberEncoder;
    private static final double DEFAULT_KG = 0.00;
    private static final double cancderoffset = 0.2;
    private int startupCounter = 0;
    private int stallCounter = 0;

    // Shuffleboard entries
    private final ShuffleboardTab grabberTab = Shuffleboard.getTab("Grabber");

    // private final ShuffleboardTab motorTab = Shuffleboard.getTab("Motor
    // Controls");

    private final ProfiledPIDController pidController;
    // Add with other instance variables
    private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(
            1, // Max velocity in rotations per second
            1 // Max acceleration in rotations per second squared
    );

    private final ArmFeedforward grabberFF = new ArmFeedforward(
            0.0,
            0.02,
            0.0);

    private final DigitalInput intakeLimitSwitch;
    private boolean hasCoral;

    private static Grabber mInstance = null;

    public static synchronized Grabber getInstance() {
        if (mInstance == null) {
            mInstance = new Grabber();
        }
        return mInstance;
    }

    public Grabber() {
        rightIntake = new SparkMax(28, MotorType.kBrushless);
        leftIntake = new SparkMax(29, MotorType.kBrushless);
        leftangle = new SparkMax(35, MotorType.kBrushless);
        rightangle = new SparkMax(36, MotorType.kBrushless);
        grabberEncoder = new CANcoder(27, "rio");

        intakeLimitSwitch = new DigitalInput(9);

        // Initialize PID controller
        pidController = new ProfiledPIDController(
                4.0, // kP
                0.0, // kI
                0.005, // kD
                constraints // Motion constraints
        );

        CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
        encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0;
        encoderConfig.MagnetSensor.MagnetOffset = 0.605469;
        grabberEncoder.getConfigurator().apply(encoderConfig);

        pidController.reset(grabberEncoder.getAbsolutePosition().getValueAsDouble());
        pidController.setTolerance(0.0); // Degrees of acceptable error
        pidController.setIZone(0.05);
        pidController.disableContinuousInput();
        pidController.setIntegratorRange(0, 0);
        pidController.setGoal(grabberEncoder.getAbsolutePosition().getValueAsDouble());
        pidController.calculate(grabberEncoder.getAbsolutePosition().getValueAsDouble());
        pidController.setTolerance(0.1);

        configureNEO550(rightIntake);
        configureNEO550(leftIntake);
        configureNEO(leftangle); // Configure leader first

        // Configure follower
        SparkMaxConfig followerConfig = new SparkMaxConfig();
        followerConfig
                .smartCurrentLimit(30)
                .idleMode(IdleMode.kBrake)
                .voltageCompensation(12.0)
                .follow(leftangle, true); // Set to follow leftangle

        rightangle.setCANTimeout(250);
        rightangle.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        leftangle.getEncoder().setPosition(0.0);

        hasCoral = false;
    }

    @Override
    public void periodic() {
        double baseKG = DEFAULT_KG;
        double currentAngle = grabberEncoder.getAbsolutePosition().getValueAsDouble();

        // Calculate kG based on angle (now in volts)
        double kG = (currentAngle <= 0) ? -baseKG * 12.0 : baseKG * 12.0;
    }

    // Copy your configuration methods
    private void configureNEO550(SparkMax motor) {
        SparkMaxConfig neo550Config = new SparkMaxConfig();

        neo550Config
                .smartCurrentLimit(30)
                .idleMode(IdleMode.kCoast)
                .voltageCompensation(12.0)
                .openLoopRampRate(0.1);

        // Apply our configuration with proper timeout
        motor.setCANTimeout(250);
        motor.configure(neo550Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private void configureNEO(SparkMax motor) {
        SparkMaxConfig neoConfig = new SparkMaxConfig();

        // Create soft limit config
        SoftLimitConfig softLimitConfig = new SoftLimitConfig();
        softLimitConfig
                .forwardSoftLimit(0.0)
                .forwardSoftLimitEnabled(false)
                .reverseSoftLimit(0)
                .reverseSoftLimitEnabled(false);

        neoConfig
                .smartCurrentLimit(30)
                .idleMode(IdleMode.kBrake)
                .voltageCompensation(12.0)
                // .openLoopRampRate(0.1)
                .apply(softLimitConfig)
                .inverted(false);

        motor.setCANTimeout(250);
        motor.configure(neoConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Sets the target position for the grabber
     * 
     * @param position Target angle in degrees
     * @return true if position is within valid range
     */
    public void setPosition(double position) {
        // pidController.reset(grabberEncoder.getAbsolutePosition().getValueAsDouble());
        pidController.setGoal(position);
        set(MathUtil.clamp(pidController.calculate(grabberEncoder.getAbsolutePosition().getValueAsDouble()), -0.3,
                0.3));
    }

    /**
     * Sets the angle motor output based on voltage
     * clamped between -1 ~ 1
     * 
     * @param output Target voltage percentage
     */
    public void set(double output) {
        double currentAngle = grabberEncoder.getAbsolutePosition().getValueAsDouble();

        // Convert position to radians for ArmFeedforward
        double positionRadians = (currentAngle - cancderoffset) * Math.PI * 2; // Adjust scaling as needed

        // Calculate feedforward voltage
        double ffVolts = grabberFF.calculate(positionRadians, output);

        // Combine feedforward with commanded output
        double totalVoltage = (MathUtil.clamp(output + ffVolts, -1, 1) * 12.0);

        leftangle.setVoltage(totalVoltage);
    }

    public double calculateKG(double position) {
        double kG = Math.cos((position - 0.2) * 15.5) * 0.01 + 0.01;
        return kG;
    }

    /**
     * @return true if grabber is at the target position
     */
    public boolean atTargetPosition() {
        return pidController.atSetpoint();
    }

    /**
     * @return current angle of the grabber in degrees
     */
    public double getCurrentAngle() {
        return grabberEncoder.getAbsolutePosition().getValueAsDouble();
    }

    public void resetcounter() {
        startupCounter = 0;
        stallCounter = 0;
    }

    public void intake() {
        // if (!intakeLimitSwitch.get()) {
        // rightIntake.set(0);
        // leftIntake.set(0);

        // }else{
        // rightIntake.set(-0.5);
        // leftIntake.set(0.5);
        // }
        double rightRPM = Math.abs(rightIntake.getEncoder().getVelocity());
        double leftRPM = Math.abs(leftIntake.getEncoder().getVelocity());

        if (startupCounter < 20) { // Startup delay
            rightIntake.set(-0.5);
            leftIntake.set(0.5);
            startupCounter++;
        } else if (rightRPM < 100 || leftRPM < 100) {
            if (stallCounter < 75) { // Wait ~0.5 seconds (25 * 20ms) before stopping
                stallCounter++;
                rightIntake.set(-0.5);
                leftIntake.set(0.5);
                hasCoral = false;
            } else {
                rightIntake.set(0);
                leftIntake.set(0);
                hasCoral = true;
                return;
            }
        }

    }

    public boolean isIntakeStopped() {
        return intakeLimitSwitch.get();
    }

    public void placeCoral() {
        rightIntake.set(0.2);
        leftIntake.set(-0.2);
        
        double rightRPM = Math.abs(rightIntake.getEncoder().getVelocity());
        double leftRPM = Math.abs(leftIntake.getEncoder().getVelocity());

        if (rightRPM > 200 || leftRPM > 200) {
            hasCoral = false;
        }
    }

    public void hitAlgea() {
        rightIntake.set(0.4);
        leftIntake.set(-0.4);

    }

    public void stop() {
        rightIntake.set(0);
        leftIntake.set(0);
    }

    public boolean hasCoral() {
        return hasCoral;
    }
}