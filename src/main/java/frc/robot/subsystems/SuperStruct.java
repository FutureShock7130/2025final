// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class SuperStruct extends SubsystemBase {
    Elevator mElevator;
    Grabber mGrabber;
    Intake mIntake;
    StateMachine mStateMachine;
    public SuperStructState mCommandedState;
    LED mled;

    private final CommandXboxController driver;
    private final Joystick buttonBoard1;
    private final Joystick buttonBoard2;

    private static SuperStruct mInstance = null;

    private final PathConstraints constraints = new PathConstraints(3, 3, 2 * Math.PI, 4 * Math.PI);

    // Add a field to track the previous state
    private SuperStructState mPreviousState = SuperStructState.DEFAULT;

    public static synchronized SuperStruct getInstance() {
        if (mInstance == null) {
            mInstance = new SuperStruct();
        }
        return mInstance;
    }

    private void configureButtonBindings() {
        new JoystickButton(buttonBoard2, 4)
                .onTrue(Commands.runOnce(
                        () -> setState(SuperStructState.L2),
                        this));

        new JoystickButton(buttonBoard2, 3)
                .onTrue(Commands.runOnce(
                        () -> setState(SuperStructState.L3),
                        this));

        new JoystickButton(buttonBoard2, 2)
                .onTrue(Commands.runOnce(
                        () -> setState(SuperStructState.L4),
                        this));

        new JoystickButton(buttonBoard1, 9)
                .onTrue(Commands.runOnce(
                        () -> setState(SuperStructState.CS),
                        this));

        new JoystickButton(buttonBoard2, 12)
                .onTrue(Commands.runOnce(
                        () -> setState(SuperStructState.PLACEMENT),
                        this));

        new JoystickButton(buttonBoard1, 10)
                .onTrue(Commands.runOnce(
                        () -> setState(SuperStructState.DEFAULT),
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
            () -> setState(SuperStructState.ALGAE_PLACEMENT), 
            this));
        // new JoystickButton(buttonBoard2, 7)
        // .onTrue(AutoBuilder.pathfindToPose(FieldConstants.A,
        // constraints).schedule());

        
    }

    /** Creates a new StateMachine. */
    public SuperStruct() {
        mElevator = Elevator.getInstance();
        mGrabber = Grabber.getInstance();
        mIntake = Intake.getInstance();
        mStateMachine = StateMachine.getInstance();
        mled = LED.getInstance();
        mCommandedState = SuperStructState.DEFAULT;
        driver = new CommandXboxController(0);
        buttonBoard1 = new Joystick(1); // First port
        buttonBoard2 = new Joystick(2); // Second port
        configureButtonBindings();
    }

    public void L1() {
        mElevator.setPosition(-0.001);
        if (mElevator.atTargetPosition()) {
            mGrabber.setPosition(-0.587637);
        }
    }

    public void L2() {
        mElevator.setPosition(17.5);
        if (mElevator.atTargetPosition()) {
            mGrabber.setPosition(-0.587637);
        }   
    }

    public void L3() {
        mElevator.setPosition(69.420);
        if (mElevator.atTargetPosition()) {
            mGrabber.setPosition(-0.587637);
        }
    }

    public void L4() {
        mElevator.setPosition(165);
        if (mElevator.atTargetPosition()) {
            mGrabber.setPosition(-0.5406969);
        }
    }

    public void TRAVEL() {
        mElevator.setPosition(-0.2); // ground
        mGrabber.setPosition(-0.453613); // default
    }

    public void CS() {
        mElevator.setPosition(-0.001);
        mGrabber.setPosition(-0.753154);
        mGrabber.intake();
    }

    public void PLACEMENT() {
        mGrabber.placeCoral();
    }

    /**
     * Sets a new state and updates the previous state tracker
     * @param state The new state to transition to
     */
    public void setState(SuperStructState state) {
        //save previos state
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
               state == SuperStructState.L4;
    }

    public void DEFAULT() {
        // Check if coming from an L-level
        boolean comingFromLLevel = isLLevel(mPreviousState);

        if (comingFromLLevel) {
            if (mPreviousState == SuperStructState.L2) {
                mElevator.setPosition(23.5);
                if (mElevator.atTargetPosition()) {
                    mGrabber.setPosition(-0.453613);
                    mElevator.setPosition(-0.02);
                }
            } else if (mPreviousState == SuperStructState.L3) {
                mElevator.setPosition(75.420);
                if (mElevator.atTargetPosition()) {
                    mGrabber.setPosition(-0.453613);
                    mElevator.setPosition(-0.02);
                }
            } else if (mPreviousState == SuperStructState.L4) {
                mled.rainbowmarquee();
                mGrabber.setPosition(-0.453613);
                if (mGrabber.atTargetPosition()) {
                    mElevator.setPosition(-0.02);
                }
            }
        } else {
            mGrabber.setPosition(-0.453613);
            mElevator.setPosition(-0.02);
        }
        mGrabber.stop();
        mGrabber.resetcounter();
        mIntake.setAngle(0.700439);
        mled.rainbowmarquee();
    }


    public void ALGAE_STOWAGE() {
        mIntake.setAngle(0.866211);
        mIntake.setIntake(0.0);
    }

    public void ALGAE_INTAKE() {
        mIntake.setAngle(0.866211);
        mIntake.setIntake(0.6);
    }

    public void ALGAE_PLACEMENT() {
        
        mIntake.setIntake(-0.6);
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
        SmartDashboard.putBoolean("From L-Level", isLLevel(mPreviousState) && mCommandedState == SuperStructState.DEFAULT);

        // Add button state monitoring to SmartDashboard
        SmartDashboard.putBoolean("Board 1 Button 1 (L1)", buttonBoard1.getRawButton(1));
        SmartDashboard.putBoolean("Board 1 Button 2 (L2)", buttonBoard1.getRawButton(2));
    }
}
