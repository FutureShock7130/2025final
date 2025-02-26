// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

/** Add your docs here. */
public class StateMachine {
    private static StateMachine instance = null;

    private SuperStructState commandedState = SuperStructState.DEFAULT;

    public StateMachine() {}

    public static synchronized StateMachine getInstance() {
        if (instance == null) {
            instance = new StateMachine();
        }
        return instance;
    }

    public void setCommandedState(SuperStructState targetState) {
        commandedState = targetState;
    }

    public SuperStructState getCommandedState() {
        return commandedState;
    }
}