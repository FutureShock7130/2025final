package frc.robot.subsystems.superstructure;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
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

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.ArmFeedforward;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;

public class Grabber extends SubsystemBase {
    private final SparkMax rightIntake;
    private final SparkMax leftIntake;
    private int startupCounter = 0;
    private int stallCounter = 0;

    // Shuffleboard entries
    private final ShuffleboardTab grabberTab = Shuffleboard.getTab("Grabber");

    // private final ShuffleboardTab motorTab = Shuffleboard.getTab("Motor
    // Controls");


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

        intakeLimitSwitch = new DigitalInput(9);

      
        CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
        encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
        encoderConfig.MagnetSensor.MagnetOffset = 0.3;

        configureNEO550(rightIntake);
        configureNEO550(leftIntake);


        hasCoral = false;
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("limitswitch", intakeLimitSwitch.get());
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

    

    public void resetcounter() {
        startupCounter = 0;
        stallCounter = 0;
    }

    public void intake() {
        if (!intakeLimitSwitch.get()) {
        rightIntake.set(0);
        leftIntake.set(0);
        return;
        }else{
        rightIntake.set(-0.2);
        leftIntake.set(0.2);
        
        }
        // double rightRPM = Math.abs(rightIntake.getEncoder().getVelocity());
        // double leftRPM = Math.abs(leftIntake.getEncoder().getVelocity());

        // if (startupCounter < 20) { // Startup delay
        //     rightIntake.set(0.4);
        //     leftIntake.set(-0.4);
        //     startupCounter++;
        // } else if (rightRPM < 200 || leftRPM < 200) {
        //     if (stallCounter < 100) { // Wait ~0.5 seconds (25 * 20ms) before stopping
        //         stallCounter++;
        //         rightIntake.set(0.45);
        //         leftIntake.set(-0.45);
        //         // hasCoral = false;
        //     } else {
        //         rightIntake.set(-0.0);
        //         leftIntake.set(0.0);
        //         // hasCoral = true;
        //         return;
        //     }
        // }

    }

    public boolean isIntakeStopped() {
        return intakeLimitSwitch.get();
    }

    public void placeCoral() {
        rightIntake.set(-0.4);
        leftIntake.set(0.4);
        
        double rightRPM = Math.abs(rightIntake.getEncoder().getVelocity());
        double leftRPM = Math.abs(leftIntake.getEncoder().getVelocity());

        // if (rightRPM > 200 || leftRPM > 200) {
        //     hasCoral = false;
        // }
    }

    public void forceCoralIntake() {
        rightIntake.set(0.45);
        leftIntake.set(-0.45);
    }

    public void placeL1() {
        rightIntake.set(-0.1);
        leftIntake.set(0.4);
    }

    public void hitAlgea() {
        rightIntake.set(0.3);
        leftIntake.set(-0.3);

    }

    public void stop() {
        rightIntake.set(0);
        leftIntake.set(0);
    }

    public boolean hasCoral() {
        return intakeLimitSwitch.get();
    }
}