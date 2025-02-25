package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;




/**
 * Module IO implementation for Talon FX drive motor controller, Talon FX turn motor controller, and
 * CANcoder
 *
 * <p>NOTE: This implementation should be used as a starting point and adapted to different hardware
 * configurations (e.g. If using an analog encoder, copy from "ModuleIOSparkMax")
 *
 * <p>To calibrate the absolute encoder offsets, point the modules straight (such that forward
 * motion on the drive motor will propel the robot forward) and copy the reported values from the
 * absolute encoders using AdvantageScope. These values are logged under
 * "/Drive/ModuleX/TurnAbsolutePositionRad"
 */
public class Swerve implements ModuleIO {
  private final TalonFX driveTalon;
  // private final TalonFX turnTalon;
  private final SparkMax turnSparkMax;
  private final CANcoder cancoder;

  private final StatusSignal<Angle> drivePosition;
  private final StatusSignal<AngularVelocity> driveVelocity;
  private final StatusSignal<Voltage> driveAppliedVolts;
  private final StatusSignal<Current> driveCurrent;

  private final StatusSignal<Angle> turnAbsolutePosition;
  // private final StatusSignal<Double> turnPosition;
  // private final StatusSignal<Double> turnVelocity;
  // private final StatusSignal<Double> turnAppliedVolts;
  // private final StatusSignal<Double> turnCurrent;

  // Gear ratios for SDS MK4i L2, adjust as necessary
  private final double DRIVE_GEAR_RATIO = 6.122449;
  private final double TURN_GEAR_RATIO = 150.0 / 7.0;

  private final boolean isTurnMotorInverted = true;
  private final Rotation2d absoluteEncoderOffset;

  public Swerve(int index) {
    switch (index) {
      case 0:
        driveTalon = new TalonFX(1,"GTX7130"); //lf
        // turnTalon = new TalonFX(1);
        turnSparkMax = new SparkMax(2, MotorType.kBrushless);
        cancoder = new CANcoder(3,"GTX7130");
        absoluteEncoderOffset = new Rotation2d(Units.rotationsToRadians(-0.132080)); // MUST BE CALIBRATED
        break;
      case 1:
        driveTalon = new TalonFX(31,"GTX7130");  //rf
        // turnTalon = new TalonFX(4);
        
        turnSparkMax = new SparkMax(32, MotorType.kBrushless);
        cancoder = new CANcoder(0,"GTX7130");
        absoluteEncoderOffset = new Rotation2d(Units.rotationsToRadians(0.472412)); // MUST BE CALIBRATED
        break;
      case 2:
        driveTalon = new TalonFX(21,"GTX7130"); //lr
        // turnTalon = new TalonFX(7);
        turnSparkMax = new SparkMax(22, MotorType.kBrushless);
        
        cancoder = new CANcoder(2,"GTX7130");
        absoluteEncoderOffset = new Rotation2d(Units.rotationsToRadians(-0.28833)); // MUST BE CALIBRATED
        break;
      case 3:
        driveTalon = new TalonFX(11,"GTX7130"); //rr
        // turnTalon = new TalonFX(10);
        turnSparkMax = new SparkMax(12, MotorType.kBrushless);
        
        cancoder = new CANcoder(1,"GTX7130");
        absoluteEncoderOffset = new Rotation2d(Units.rotationsToRadians(0.457275)); // MUST BE CALIBRATED
        break;
      default:
        throw new RuntimeException("Invalid module index");
    }

    var driveConfig = new TalonFXConfiguration();
    driveConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    driveTalon.getConfigurator().apply(driveConfig);
    setDriveBrakeMode(true);

    // var turnConfig = new TalonFXConfiguration();
    // turnConfig.CurrentLimits.SupplyCurrentLimit = 30.0;
    // turnConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    // turnTalon.getConfigurator().apply(turnConfig);
    // setTurnBrakeMode(true);
    SparkMaxConfig turnSparkConfig = new SparkMaxConfig();
    
    turnSparkConfig
          .smartCurrentLimit(30)
          .voltageCompensation(12.0)
          .inverted(isTurnMotorInverted)
          .idleMode(IdleMode.kBrake);
    

    turnSparkMax.setCANTimeout(250);

    turnSparkMax.configure(turnSparkConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    

    cancoder.getConfigurator().apply(new CANcoderConfiguration());

    drivePosition = driveTalon.getPosition();
    driveVelocity = driveTalon.getVelocity();
    driveAppliedVolts = driveTalon.getMotorVoltage();
    driveCurrent = driveTalon.getSupplyCurrent();

    turnAbsolutePosition = cancoder.getAbsolutePosition();
    // turnPosition = turnTalon.getPosition();
    // turnVelocity = turnTalon.getVelocity();
    // turnAppliedVolts = turnTalon.getMotorVoltage();
    // turnCurrent = turnTalon.getSupplyCurrent();

    // BaseStatusSignal.setUpdateFrequencyForAll(100.0, drivePosition, turnPosition);
    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0, drivePosition); // Required for odometry, use faster rate
    // BaseStatusSignal.setUpdateFrequencyForAll(
    //     50.0,
    //     driveVelocity,
    //     driveAppliedVolts,
    //     driveCurrent,
    //     turnAbsolutePosition,
    //     turnVelocity,
    //     turnAppliedVolts,
    //     turnCurrent);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, driveVelocity, driveAppliedVolts, driveCurrent, turnAbsolutePosition);
    driveTalon.optimizeBusUtilization();
    // turnTalon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // BaseStatusSignal.refreshAll(
    //     drivePosition,
    //     driveVelocity,
    //     driveAppliedVolts,
    //     driveCurrent,
    //     turnAbsolutePosition,
    //     turnPosition,
    //     turnVelocity,
    //     turnAppliedVolts,
    //     turnCurrent);
    BaseStatusSignal.refreshAll(
        drivePosition, driveVelocity, driveAppliedVolts, driveCurrent, turnAbsolutePosition);

    inputs.drivePositionRad =
        Units.rotationsToRadians(drivePosition.getValueAsDouble()) / DRIVE_GEAR_RATIO;
    inputs.driveVelocityRadPerSec =
        Units.rotationsToRadians(driveVelocity.getValueAsDouble()) / DRIVE_GEAR_RATIO;
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveCurrentAmps = new double[] {driveCurrent.getValueAsDouble()};

    // inputs.turnAbsolutePosition =
    //     Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble())
    //         .minus(absoluteEncoderOffset);
    // inputs.turnPosition =
    //     Rotation2d.fromRotations(turnPosition.getValueAsDouble() / TURN_GEAR_RATIO);
    // inputs.turnVelocityRadPerSec =
    //     Units.rotationsToRadians(turnVelocity.getValueAsDouble()) / TURN_GEAR_RATIO;
    // inputs.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
    // inputs.turnCurrentAmps = new double[] {turnCurrent.getValueAsDouble()};

    inputs.turnAbsolutePosition =
        new Rotation2d(Units.rotationsToRadians(cancoder.getAbsolutePosition().getValueAsDouble()))
            // turnAbsoluteEncoder.getVoltage() / RobotController.getVoltage5V() * 2.0 * Math.PI)
            .plus(absoluteEncoderOffset);
    inputs.turnPosition =
        Rotation2d.fromRotations(turnSparkMax.getEncoder().getPosition() / TURN_GEAR_RATIO);
    inputs.turnVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(turnSparkMax.getEncoder().getVelocity())
            / TURN_GEAR_RATIO;
    inputs.turnAppliedVolts = turnSparkMax.getAppliedOutput() * turnSparkMax.getBusVoltage();
    inputs.turnCurrentAmps = new double[] {turnSparkMax.getOutputCurrent()};
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveTalon.setControl(new VoltageOut(volts));
  }

  @Override
  public void setTurnVoltage(double volts) {
    // turnTalon.setControl(new VoltageOut(volts));
    turnSparkMax.setVoltage(volts);
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    var config = new MotorOutputConfigs();
    config.Inverted = InvertedValue.CounterClockwise_Positive;
    config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    driveTalon.getConfigurator().apply(config);
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    // var config = new MotorOutputConfigs();
    // config.Inverted =
    //     isTurnMotorInverted
    //         ? InvertedValue.Clockwise_Positive
    //         : InvertedValue.CounterClockwise_Positive;
    // config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    // turnTalon.getConfigurator().apply(config);
    // turnSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    
  }
}
