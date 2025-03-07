// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.SuperStructState;
import frc.robot.subsystems.superstructure.Elevator;
import frc.robot.subsystems.superstructure.Grabber;
import frc.robot.subsystems.superstructure.Intake;
import edu.wpi.first.wpilibj.XboxController;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ObjectDetection;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import java.util.Map;

public class SuperStruct extends SubsystemBase {
    Elevator mElevator;
    Grabber mGrabber;
    Intake mIntake;
    StateMachine mStateMachine;
    ObjectDetection mObjectDetection;
    public SuperStructState mCommandedState;
    LED mled;

    private final CommandXboxController driver;
    private final CommandJoystick buttonBoard1;
    private final CommandJoystick buttonBoard2;
    private static final int buttonBoard1Port = 1;
    private static final int buttonBoard2Port = 2;

    private static SuperStruct mInstance = null;

    private final PathConstraints constraints = new PathConstraints(3, 3, 2 * Math.PI, 4 * Math.PI);

    // Add a field to track the previous state
    private SuperStructState mPreviousState = SuperStructState.DEFAULT;

    private double savedElevatorPos = 0.0;
    private boolean hasSetSafeHeight = false;
    private boolean isMovingToDefault = false;
    private double targetUpPosition = 0.0;

    public static synchronized SuperStruct getInstance() {
        if (mInstance == null) {
            mInstance = new SuperStruct();
        }
        return mInstance;
    }

    private void configureButtonBindings() {
        new CommandJoystick(1).button(12)
                .onTrue(Commands.runOnce(
                        () -> setState(SuperStructState.L1),
                        this));

        new CommandJoystick(1).axisLessThan(1, -0.5)
                .whileTrue(Commands.runOnce(
                        () -> setState(SuperStructState.L2),
                        this));

        new CommandJoystick(2).button(4)
                .onTrue(Commands.runOnce(
                        () -> setState(SuperStructState.L3),
                        this));

        new CommandJoystick(2).button(5)
                .onTrue(Commands.runOnce(
                        () -> setState(SuperStructState.L4),
                        this));

        new JoystickButton(driver.getHID(), 5)
                .onTrue(Commands.runOnce(
                        () -> setState(SuperStructState.CS),
                        this));

        // new CommandXboxController(0).axisGreaterThan(3, 0.05)
        // .toggleOnTrue(Commands.runOnce(
        // () -> setState(SuperStructState.PLACEMENT),
        // this));

        new CommandJoystick(2).button(2)
                .onTrue(Commands.runOnce(
                        () -> setState(SuperStructState.PLACEMENT),
                        this));

        new CommandJoystick(2).button(10)
                .onTrue(Commands.runOnce(
                        () -> setState(SuperStructState.DEFAULT),
                        this));

        new CommandJoystick(2).button(7)
                .onTrue(Commands.runOnce(
                        () -> setState(SuperStructState.PAUSE),
                        this));

        // new CommandJoystick(1).button(11)
        // .onTrue(Commands.runOnce(
        // () -> setState(SuperStructState.ABORT),
        // this));

        new CommandJoystick(2).button(6)
                .onTrue(Commands.runOnce(
                        () -> setState(SuperStructState.IVECHANGEDMYMIND),
                        this));

        new CommandJoystick(2).button(9)
                .onTrue(Commands.runOnce(
                        () -> setState(SuperStructState.HIT_ALGAE),
                        this));

        new CommandJoystick(1).button(11)
                .onTrue(Commands.runOnce(
                        () -> setState(SuperStructState.ELEDROP),
                        this));

        new JoystickButton(driver.getHID(), 1)
                .onTrue(Commands.runOnce(
                        () -> setState(SuperStructState.ALGAE_INTAKE),
                        this));

        new JoystickButton(driver.getHID(), 2)
                .onTrue(Commands.runOnce(
                        () -> setState(SuperStructState.ALGAE_STOWAGE),
                        this));

        new JoystickButton(driver.getHID(), 4)
                .onTrue(Commands.runOnce(
                        () -> setState(SuperStructState.DEFAULT),
                        this));

        new CommandJoystick(2).button(8)
                .onTrue(Commands.runOnce(
                        () -> setState(SuperStructState.ALGAE_PLACEMENT),
                        this));

        new CommandJoystick(2).button(6)
                .onTrue(Commands.runOnce(
                        () -> setState(SuperStructState.RESET),
                        this));


        // Object detection - Follow target (Xbox controller Y button)
        // new JoystickButton(driver.getHID(), XboxController.Button.kY.value)
        // .onTrue(Commands.runOnce(
        // () -> {
        // setState(SuperStructState.FOLLOW_TARGET);
        // SmartDashboard.putString("Button Press", "Y Button - Start Following");
        // },
        // this));

        // // Object detection - Stop following (Xbox controller B button)
        // new JoystickButton(driver.getHID(), XboxController.Button.kB.value)
        // .onTrue(Commands.runOnce(
        // () -> {
        // setState(SuperStructState.STOP_FOLLOWING);
        // SmartDashboard.putString("Button Press", "B Button - Stop Following");
        // },
        // this));
    }

    /** Creates a new StateMachine. */
    public SuperStruct() {
        mElevator = Elevator.getInstance();
        mGrabber = Grabber.getInstance();
        mIntake = Intake.getInstance();
        mStateMachine = StateMachine.getInstance();
        mObjectDetection = ObjectDetection.getInstance();
        mled = LED.getInstance();
        mCommandedState = SuperStructState.DEFAULT;
        driver = new CommandXboxController(0);
        buttonBoard1 = new CommandJoystick(buttonBoard1Port); // First port
        buttonBoard2 = new CommandJoystick(buttonBoard2Port); // Second port

        // Add Shuffleboard controls for object following
        var tab = Shuffleboard.getTab("Controls");

        // Current following status display with indicator color
        tab.addBoolean("Following Status", () -> {
            return mCommandedState == SuperStructState.FOLLOW_TARGET;
        })
                .withSize(2, 1)
                .withPosition(0, 5)
                .withProperties(Map.of("colorWhenTrue", "blue", "colorWhenFalse", "gray"));

        // Current state display
        tab.addString("Current State", () -> {
            return mCommandedState.toString();
        })
                .withSize(2, 1)
                .withPosition(2, 5);

        // Debug panel for SmartDashboard values
        tab.addBoolean("Y Button (Follow)", () -> driver.getHID().getRawButton(XboxController.Button.kY.value))
                .withSize(1, 1)
                .withPosition(4, 5)
                .withProperties(Map.of("colorWhenTrue", "blue"));

        tab.addBoolean("B Button (Stop)", () -> driver.getHID().getRawButton(XboxController.Button.kB.value))
                .withSize(1, 1)
                .withPosition(5, 5)
                .withProperties(Map.of("colorWhenTrue", "red"));

        configureButtonBindings();
    }

    public void L1() {
        mElevator.setPosition(-0.001 * 0.6);
        if (mElevator.atTargetPosition()) {
            mGrabber.setPosition(0.460205);
        } else {
            mGrabber.setPosition(0.618896);
        }
        mIntake.setAngle(-0.390137);

    }

    public void L2() {
        mElevator.setPosition(18.69420 * 0.6);
        if (mElevator.atTargetPosition()) {
            mGrabber.setPosition(0.460205);
        } else {
            mGrabber.setPosition(0.618896);
        }
        mIntake.setAngle(-0.390137);

    }

    public void L3() {
        mElevator.setPosition(68.620 * 0.6);
        if (mElevator.atTargetPosition()) {
            mGrabber.setPosition(0.460205);
        } else {
            mGrabber.setPosition(0.618896);
        }
        mIntake.setAngle(-0.390137);

    }

    public void L4() {
        mElevator.setPosition(165 * 0.6);
        if (mElevator.atTargetPosition()) {
            mGrabber.setPosition(0.511719);
        } else {
            mGrabber.setPosition(0.618896);
        }
        mIntake.setAngle(-0.390137);

    }

    public void TRAVEL() {
        mElevator.setPosition(-0.2 * 0.6); // ground
        mGrabber.setPosition(0.618896); // default
        mIntake.setAngle(-0.390137);

    }

    public void RESET() {
        mElevator.stop();
        mElevator.resetPosition();
    }

    public void CS() {
        mElevator.setPosition(-0.001 * 0.6);
        if (mElevator.atTargetPosition()) {
            mGrabber.setPosition(0.289307);
        }
        // mGrabber.setPosition(0.289307);
        mGrabber.intake();
        mIntake.setAngle(-0.390137);

    }

    public void PLACEMENT() {
        if (mPreviousState == SuperStructState.L1) {
            mGrabber.placeL1();
        } else {
            mGrabber.placeCoral();
        }
    }

    /**
     * Sets a new state and updates the previous state tracker
     * 
     * @param state The new state to transition to
     */
    public void setState(SuperStructState state) {
        // save previos state
        mPreviousState = mCommandedState;

        // Set the new state
        mStateMachine.setCommandedState(state);
    }

    /**
     * Checks if the given state is one of the L-levels
     */
    private boolean isLLevel(SuperStructState state) {
        return state == SuperStructState.L1 ||
                state == SuperStructState.L2 ||
                state == SuperStructState.L3 ||
                state == SuperStructState.L4 ||
                state == SuperStructState.CS ||
                state == SuperStructState.PLACEMENT ||
                state == SuperStructState.HIT_ALGAE;
    }

    public void DEFAULT() {
        // Debug current state
        SmartDashboard.putNumber("Current Elevator Position", mElevator.getElevatorPosition());

        // Check if coming from an L-level
        boolean comingFromLLevel = isLLevel(mPreviousState);
        SmartDashboard.putBoolean("Coming From L-Level", comingFromLLevel);

        if (comingFromLLevel) {
            if (!hasSetSafeHeight && !isMovingToDefault) {
                // Only set target position once
                savedElevatorPos = mElevator.getElevatorPosition();
                int raiseDistance = mPreviousState == SuperStructState.L4 ? 30 : 15;
                targetUpPosition = savedElevatorPos + raiseDistance;
                mElevator.setPosition(targetUpPosition);
                mGrabber.setPosition(0.618896);
                hasSetSafeHeight = true;
                SmartDashboard.putString("Movement Phase", "Moving Up");
            } else if (hasSetSafeHeight && mElevator.atTargetPosition() && !isMovingToDefault) {
                // Once we reach the up position, start moving down
                mElevator.setPosition(-0.02 * 0.6);
                isMovingToDefault = true;
                SmartDashboard.putString("Movement Phase", "Moving to Default");
            } else if (isMovingToDefault && mElevator.atTargetPosition()) {
                // Reset flags once we reach default
                hasSetSafeHeight = false;
                isMovingToDefault = false;
                SmartDashboard.putString("Movement Phase", "At Default");
            }

            // Debug info
            SmartDashboard.putNumber("Target Up Position", targetUpPosition);
            SmartDashboard.putBoolean("At Target Position", mElevator.atTargetPosition());
        } else {
            // Direct to default if not from L-level
            hasSetSafeHeight = false;
            isMovingToDefault = false;
            mGrabber.setPosition(0.618896);
            mElevator.setPosition(-0.02 * 0.6);
            SmartDashboard.putString("Movement Phase", "Direct to Default");
        }

        // Common actions
        mGrabber.stop();
        mGrabber.resetcounter();
        mIntake.setAngle(-0.390137);
        mIntake.setIntake(0);
        mled.rainbowmarquee();
        mObjectDetection.stopFollowing();
    }

    public void grabberDefault() {
        mGrabber.setPosition(0.618896);
        mGrabber.stop();
        mGrabber.resetcounter();
    }

    public void ALGAE_STOWAGE() {
        mIntake.setAngle(-0.234619);
        mIntake.setIntake(0.01);
        mObjectDetection.stopFollowing();
    }

    public void ALGAE_INTAKE() {
        mIntake.setAngle(-0.234619);
        mIntake.setIntake(0.5);
        // mObjectDetection.startFollowing();
    }

    public void ALGAE_PLACEMENT() {

        mIntake.setIntake(-0.6);
    }

    public void HIT_ALGAE() {
        mGrabber.hitAlgea();
        mGrabber.setPosition(0.412295);

        mIntake.setAngle(-0.390137);
    }

    public void GENSHINIMPACT() {
        mElevator.setPosition(129);
        if (mElevator.atTargetPosition()) {
            mGrabber.setPosition(0.618896);
        }
    }

    public void ELEDROP() {
        mElevator.setVoltage(-0.3);
    }

    /**
     * Start following a target
     * Uses the ObjectDetection subsystem to follow targets
     */
    public void FOLLOW_TARGET() {
        mled.color(0, 0, 255); // Blue color to indicate following

        // Debug output
        SmartDashboard.putString("SuperStruct State", "FOLLOW_TARGET");
        SmartDashboard.putBoolean("Following Active", true);

        // Schedule the follow command through ObjectDetection
        mObjectDetection.startFollowing();

        // Stop other systems when we're following
        mGrabber.stop();
    }

    /**
     * Stop following a target
     */
    public void STOP_FOLLOWING() {
        // Debug output
        SmartDashboard.putString("SuperStruct State", "STOP_FOLLOWING");
        SmartDashboard.putBoolean("Following Active", false);

        // Stop following command through ObjectDetection
        mObjectDetection.stopFollowing();

        // After stopping, go to default state
        setState(SuperStructState.DEFAULT);
    }

    public void updateState() {
        switch (mCommandedState) {
            case L1:
                L1();
                break;
            case L2:
                L2();
                break;
            case L3:
                L3();
                break;
            case L4:
                L4();
                break;
            case TRAVEL:
                TRAVEL();
                break;
            case CS:
                CS();
                break;
            case PLACEMENT:
                PLACEMENT();
                break;
            case DEFAULT:
                DEFAULT();
                break;
            case ALGAE_STOWAGE:
                ALGAE_STOWAGE();
                break;
            case ALGAE_INTAKE:
                ALGAE_INTAKE();
                break;
            case ALGAE_PLACEMENT:
                ALGAE_PLACEMENT();
                break;
            case FOLLOW_TARGET:
                FOLLOW_TARGET();
                break;
            case STOP_FOLLOWING:
                STOP_FOLLOWING();
                break;
            case HIT_ALGAE:
                HIT_ALGAE();
                break;
            case GRABBER_DEFAULT:
                grabberDefault();
                break;
            case GENSHINIMPACT:
                GENSHINIMPACT();
                break;
            case RESET:
                RESET();
                break;
            case ELEDROP:
                ELEDROP();
                break;
        }
    }

    @Override
    public void periodic() {
        // Get current state
        mCommandedState = mStateMachine.getCommandedState();

        // Update state
        updateState();

        SmartDashboard.putString("Commanded State", mCommandedState.toString());
        SmartDashboard.putString("Previous State", mPreviousState.toString());
        SmartDashboard.putBoolean("From L-Level",
                isLLevel(mPreviousState) && mCommandedState == SuperStructState.DEFAULT);
    }
}
