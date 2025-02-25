// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.SuperStructState;
import frc.robot.subsystems.superstructure.Elevator;
import frc.robot.subsystems.superstructure.Grabber;
import frc.robot.subsystems.superstructure.Intake;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Commands;


public class SuperStruct extends SubsystemBase {
  Elevator mElevator;
  Grabber mGrabber;
  Intake mIntake;
  StateMachine mStateMachine;
  SuperStructState mCommandedState;
  
  private final Joystick buttonBoard1;
  private final Joystick buttonBoard2;

  private static SuperStruct mInstance = null;

  public static synchronized SuperStruct getInstance() {
      if (mInstance == null) {
          mInstance = new SuperStruct();
      }
      return mInstance;
  }
  /** Creates a new StateMachine. */
  public SuperStruct() {
    mElevator = Elevator.getInstance();
    mGrabber = Grabber.getInstance();
    mIntake = Intake.getInstance();
    mStateMachine = StateMachine.getInstance();
    mCommandedState = SuperStructState.DEFAULT;
    buttonBoard1 = new Joystick(1);  // First port
    buttonBoard2 = new Joystick(2);  // Second port
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    new JoystickButton(buttonBoard2, 4)
        .onTrue(Commands.runOnce(
            () -> setState(SuperStructState.L2),
            this
        ));

    new JoystickButton(buttonBoard2, 3)
        .onTrue(Commands.runOnce(
            () -> setState(SuperStructState.L3),
            this
        ));

    new JoystickButton(buttonBoard2, 2)
        .onTrue(Commands.runOnce(
            () -> setState(SuperStructState.L4),
            this
        ));

    new JoystickButton(buttonBoard1, 1)
        .onTrue(Commands.runOnce(
            () -> setState(SuperStructState.CS),
            this
        ));

    new JoystickButton(buttonBoard1, 2)
        .onTrue(Commands.runOnce(
            () -> setState(SuperStructState.PLACEMENT),
            this
        ));

    new JoystickButton(buttonBoard1, 4)
        .onTrue(Commands.runOnce(
            () -> setState(SuperStructState.DEFAULT),
            this
        ));
  }

  public void L1() {
    mElevator.setPosition(-0.001); //gorund / L1 (it just works
    if (mElevator.atTargetPosition()) {
        mGrabber.setPosition(-0.18); 
    } 
  }

  public void L2() {
    mElevator.setPosition(17.5); //L2
    if (mElevator.atTargetPosition()) {
       mGrabber.setPosition(-0.1884); 
    } 
  }
  public void L3() {
    mElevator.setPosition(63.54888916015625); //L3
    if (mElevator.atTargetPosition()) {
        mGrabber.setPosition(-0.1884); 
    } 
  }

  public void L4() {
    mElevator.setPosition(159); //L4
    if (mElevator.atTargetPosition()) {
        mGrabber.setPosition(-0.1284); 
    } 
  }

  public void TRAVEL() {
    mElevator.setPosition(-0.2); //ground
    mGrabber.setPosition(-0.0432); //default
  }

  public void CS() {
    mElevator.setPosition(-0.001);
    mIntake.setAngle(0.338623);
    mIntake.setIntake(-0.5);
    if (mElevator.atTargetPosition()) {
        mGrabber.setPosition(0.21630859375);  
        mGrabber.intake();
    }
  }

  public void PLACEMENT() {
    mGrabber.placeCoral();
  }

  public void DEFAULT() {
    mElevator.setPosition(-0.2);
    mGrabber.setPosition(-0.0432); //default
    mGrabber.stop();
    mIntake.setIntake(0);
  }

  public void ALGAE_STOWAGE() {

  }

  public void ALGAE_INTAKE() {

  }

  public void ALGAE_PLACEMENT() {

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

  public void setState(SuperStructState state) {
    mStateMachine.setCommandedState(state);
  }
    

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    mCommandedState = mStateMachine.getCommandedState();
    updateState();

    SmartDashboard.putString("Commanded State", mCommandedState.toString());
    
    // Add button state monitoring to SmartDashboard
    SmartDashboard.putBoolean("Board 1 Button 1 (L1)", buttonBoard1.getRawButton(1));
    SmartDashboard.putBoolean("Board 1 Button 2 (L2)", buttonBoard1.getRawButton(2));
  }
}
