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
import frc.robot.subsystems.superstructure.AlgaeRemover;
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
    AlgaeRemover mAlgaeRemover;
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

    // Add this field to the class
    private final frc.robot.subsystems.superstructure.AlgaeRemover algaeRemover = frc.robot.subsystems.superstructure.AlgaeRemover
            .getInstance();
    private static final double ALGAE_SPEED = 0.3; // Speed for the algae remover
    private static final double ALGAE_DURATION = 0.2; // Duration in seconds for the algae remover to run
    private boolean algaeCommandSent = false; // Track if we've already sent the command
    private double algaeStartTime = 0; // Track when the algae command was sent
    private int algaeButtonPressCount = 0; // Counter for button presses

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

        new CommandJoystick(2).button(9)
                .onTrue(Commands.runOnce(
                        () -> setState(SuperStructState.PLACEMENT),
                        this));

        new CommandJoystick(2).button(10)
                .onTrue(Commands.runOnce(
                        () -> setState(SuperStructState.DEFAULT),
                        this));

        new CommandJoystick(2).button(2)
                .onTrue(Commands.runOnce(
                        () -> setState(SuperStructState.PAUSE),
                        this));

        // new CommandJoystick(1).button(11)
        // .onTrue(Commands.runOnce(
        // () -> setState(SuperStructState.ABORT),
        // this));

        // new CommandJoystick(2).button(6)
        // .onTrue(Commands.runOnce(
        // () -> setState(SuperStructState.IVECHANGEDMYMIND),
        // this));

        // new CommandJoystick(2).button(9)
        // .onTrue(Commands.runOnce(
        // () -> setState(SuperStructState.HIT_ALGAE),
        // this));

        new CommandJoystick(1).button(11)
                .onTrue(Commands.runOnce(
                        () -> setState(SuperStructState.ELEDROP),
                        this));

        new JoystickButton(driver.getHID(), 1)
                .onTrue(Commands.runOnce(
                        () -> {
                            // Increment press counter when the button is pressed
                            algaeButtonPressCount++;
                            setState(SuperStructState.SMACK_ALGAE);
                        },
                        this));

        new JoystickButton(driver.getHID(), 2)
                .onTrue(Commands.runOnce(
                        () -> setState(SuperStructState.ALGAE_PLACEMENT),
                        this));

        new JoystickButton(driver.getHID(), 3)
                .onTrue(Commands.runOnce(
                        () -> setState(SuperStructState.ALGAE_INTAKE),
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
        mAlgaeRemover = AlgaeRemover.getInstance();
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
        mElevator.setPosition(-0.0);
        if (mElevator.atTargetPosition()) {
            mGrabber.setPosition(0.227539);
        } else {
            mGrabber.setPosition(0.396729);
        }
        mIntake.setAngle(0.818359);
    }

    public void L2() {
        mElevator.setPosition(18.69420 * 0.6);
        if (mElevator.atTargetPosition()) {
            mGrabber.setPosition(0.227539);
        } else {
            mGrabber.setPosition(0.396729);
        }
        mIntake.setAngle(0.818359);
    }

    public void L3() {
        mElevator.setPosition(69.420 * 0.6);
        if (mElevator.atTargetPosition()) {
            mGrabber.setPosition(0.227539);
        } else {
            mGrabber.setPosition(0.396729);
        }
        mIntake.setAngle(0.818359);
    }

    public void L4() {
        mGrabber.setPosition(0.396729);
        if (mGrabber.atTargetPosition()) {
            mElevator.setPosition(165 * 0.6);
            if (mElevator.atTargetPosition()) {
                mGrabber.setPosition(0.297539);
            }
        }
        mIntake.setAngle(0.818359);
    }

    public void TRAVEL() {
        mElevator.setPosition(-0.2 * 0.6); // ground
        // mIntake.setAngle(-0.390137);

    }

    public void RESET() {
        mElevator.stop();
        mElevator.resetPosition();
    }

    public void CS() {
        mElevator.setPosition(-0.001 * 0.6);
        mGrabber.intake();
        if (mElevator.atTargetPosition()) {
            if (!mGrabber.hasCoral()) {
                mGrabber.setPosition(0.687162);
            } else if (mGrabber.hasCoral()) {
                mGrabber.setPosition(0.396729);
            }
        }
        mIntake.setAngle(0.818359);
        // mIntake.setIntake(-0.2);
        // Check if coral is detected and update LEDs accordingly
        if (mGrabber.hasCoral()) {
            // Set LED to green when coral is detected
            mled.color(0, 0, 255); // RGB values for green
        } else {
            mled.blink(255, 50, 50);
        }
    }

    public void PLACEMENT() {
        if (mPreviousState == SuperStructState.L1) {
            mGrabber.placeL1();
        } else {
            mGrabber.placeCoral();
        }
        mled.blinkSection1(255, 0, 255, 1.5);
    }

    public void CORALFORCEINTAKE() {
        // mGrabber.forceCoralIntake();
    }

    /**
     * Sets a new state and updates the previous state tracker
     * 
     * @param state The new state to transition to
     */
    public void setState(SuperStructState state) {
        // save previos state
        mPreviousState = mCommandedState;

        // if (!state.equals(SuperStructState.DEFAULT)) {
        // If not going to DEFAULT (which has its own LED pattern)
        mled.nocolor();
        // }

        // Reset algaeCommandSent when changing to a different state
        if (state != SuperStructState.SMACK_ALGAE) {
            algaeCommandSent = false;
        }

        // Set the new state
        mStateMachine.setCommandedState(state);
    }

    /**
     * Checks if the given state is one of the L-levels
     */
    private boolean isLLevel(SuperStructState state) {
        return state == SuperStructState.L2 ||
                state == SuperStructState.L3 ||
                state == SuperStructState.L4 ||
                state == SuperStructState.PLACEMENT;
    }

    public void DEFAULT() {
        // Check if coming from an L-level
        boolean comingFromLLevel = isLLevel(mPreviousState);
        SmartDashboard.putBoolean("Coming From L-Level", comingFromLLevel);

        if (comingFromLLevel) {
            if (!hasSetSafeHeight && !isMovingToDefault) {
                // Only set target position once
                savedElevatorPos = mElevator.getElevatorPosition();
                int raiseDistance = mPreviousState == SuperStructState.L4 ? 25 : 35;
                targetUpPosition = savedElevatorPos + raiseDistance;
                mElevator.setPosition(targetUpPosition);
                mGrabber.setPosition(0.396729);
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
            mGrabber.setPosition(0.396729);
            mElevator.setPosition(0);
            SmartDashboard.putString("Movement Phase", "Direct to Default");
        }

        // Common actions
        mGrabber.stop();
        mIntake.setAngle(0.818359);
        mIntake.setIntake(0.0);
        mled.rainbowmarquee();
        mObjectDetection.stopFollowing();
    }

    public void grabberDefault() {
        mGrabber.stop();
        mGrabber.setPosition(0.396729);
        // mGrabber.resetcounter();
    }

    public void ALGAE_STOWAGE() {
        mIntake.setAngle(0.932861);
        mIntake.setIntake(0.01);
        mObjectDetection.stopFollowing();
        // mAlgaeRemover.setSpeed(0.6);
    }

    public void ALGAE_INTAKE() {
        mIntake.setAngle(0.932861);
        mIntake.setIntake(0.3);
        // mObjectDetection.startFollowing();

        // mAlgaeRemover.setSpeed(-0.6);
    }

    public void ALGAE_PLACEMENT() {

        mIntake.setAngle(0.818359);
        mIntake.setIntake(-0.2);
    }

    public void HIT_ALGAE() {
        mGrabber.hitAlgea();

        // mIntake.setAngle(-0.390137);
    }

    public void GENSHINIMPACT() {
        // mElevator.setPosition(129);
    }

    public void ELEDROP() {
        mElevator.setVoltage(-0.3);
    }

    public void SMACK_ALGAE() {
        // Determine direction based on button press count (odd = up, even = down)
        boolean directionUp = (algaeButtonPressCount % 2 == 1); // Odd = up, Even = down
        double speed = directionUp ? -ALGAE_SPEED : ALGAE_SPEED;

        // Run for a specific time duration
        algaeRemover.runForTime(speed, ALGAE_DURATION);

    }

    public void SMACK_DOWN() {
        algaeRemover.runForTime(-ALGAE_SPEED, ALGAE_DURATION);
    }

    public void SMACK_UP() {
        algaeRemover.runForTime(ALGAE_SPEED, ALGAE_DURATION);
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

    public void DISABLE() {
        // mled.blinkSection1(255, 165, 0, 1.5);
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
            case SMACK_ALGAE:
                SMACK_ALGAE();
                break;
            case SMACK_DOWN:
                SMACK_DOWN();
                break;
            case SMACK_UP:
                SMACK_UP();
                break;
            case DISABLE:
                DISABLE();
                break;
        }
    }

    @Override
    public void periodic() {
        // Get current state
        mCommandedState = mStateMachine.getCommandedState();

        // Update state
        updateState();

        // Only update LED indicators every 100ms (10 times per second) to save
        // resources
        // This is fast enough for visual feedback but reduces CPU usage
        if ((System.currentTimeMillis() % 100) < 20) { // Only run ~20% of the time
            // Track elevator position progress on LED sections 2 and 3
            if (mCommandedState == SuperStructState.L1 ||
                    mCommandedState == SuperStructState.L2 ||
                    mCommandedState == SuperStructState.L3 ||
                    mCommandedState == SuperStructState.CS ||
                    mCommandedState == SuperStructState.DEFAULT) { // Also track when going to default position

                double currentPosition = mElevator.getElevatorPosition();
                double targetPosition = getTargetElevatorPosition();

                // Handle both upward and downward movement
                boolean isMovingDown = currentPosition > targetPosition
                        && Math.abs(currentPosition - targetPosition) > 1.0;

                // Calculate completion percentage differently based on direction
                double percentComplete;
                boolean atTarget = Math.abs(currentPosition - targetPosition) <= 1.0;

                if (isMovingDown) {
                    // For downward movement - calculate progress from start to target
                    // We want to show 0% at the start position and 100% when reaching target

                    // Estimate starting position based on the previous state or use current
                    // position
                    double startPosition = 0;

                    // If going to default (0), assume we're coming from one of the levels
                    if (mCommandedState == SuperStructState.DEFAULT) {
                        // Use a reasonable starting height (max of current or 100)
                        startPosition = Math.max(currentPosition, 100.0);
                    } else {
                        // For other downward movements, assume we started at position 130
                        // (slightly higher than highest target)
                        startPosition = 130.0;
                    }

                    if (startPosition > targetPosition) {
                        // Map from [startPosition, targetPosition] to [0, 1]
                        // This gives us 0% at start position and 100% at target position
                        percentComplete = Math.min(
                                1.0 - ((currentPosition - targetPosition) / (startPosition - targetPosition)), 1.0);
                    } else {
                        percentComplete = 1.0; // Already at or below target
                    }

                    // Use color based on height for downward movement
                    // For downward motion: section 2 fills right-to-left
                    mled.sectionHeightColor(2, currentPosition, 130.0, percentComplete, atTarget, true); // Fill
                                                                                                         // right-to-left
                } else {
                    // For upward movement, color based on height
                    // For upward motion: section 2 fills left-to-right
                    if (targetPosition > 0.1) { // Avoid division by zero
                        percentComplete = Math.min(currentPosition / targetPosition, 1.0);
                        mled.sectionHeightColor(2, currentPosition, 130.0, percentComplete, atTarget, false); // Fill
                                                                                                              // left-to-right
                    }
                }
            }
        }
    }

    /**
     * Get the target elevator position based on current state
     * 
     * @return The target position for the current state
     */
    private double getTargetElevatorPosition() {
        switch (mCommandedState) {
            case L1:
                return 25.0;
            case L2:
                return 43.7;
            case L3:
                return 76.0;
            case L4:
                return 113.0;
            default:
                return 0.0;
        }
    }
}
