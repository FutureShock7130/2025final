// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.FieldConstants;

/**
 * The methods in this class are called automatically corresponding to each
 * mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the
 * package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private XboxController driver;
  private Joystick ButtonBox1;
  private Joystick ButtonBox2;

  private final RobotContainer m_robotContainer;

  private final PathConstraints constraints = new PathConstraints(3, 3, 2 * Math.PI, 4 * Math.PI);

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  public Robot() {
    m_robotContainer = new RobotContainer();
    driver = new XboxController(0);
    ButtonBox1 = new Joystick(1);
    ButtonBox2 = new Joystick(2);
  }

  public void RobotInit() {

  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    if (ButtonBox2.getRawButtonPressed(7)) {
      AutoBuilder.pathfindToPose(Constants.FieldConstants.A, constraints).until(() -> driverWantsControl()).schedule();
    }
    if (ButtonBox2.getRawButtonPressed(8)) {
      AutoBuilder.pathfindToPose(Constants.FieldConstants.B, constraints).until(() -> driverWantsControl()).schedule();
    }

    if (ButtonBox1.getRawButtonPressed(7)) {
      AutoBuilder.pathfindToPose(Constants.FieldConstants.C, constraints).until(() -> driverWantsControl()).schedule();
    }

    if (ButtonBox1.getRawButtonPressed(12)) {
      AutoBuilder.pathfindToPose(Constants.FieldConstants.D, constraints).until(() -> driverWantsControl()).schedule();
    }

    if (ButtonBox1.getRawButtonPressed(5)) {
      AutoBuilder.pathfindToPose(Constants.FieldConstants.E, constraints).until(() -> driverWantsControl()).schedule();
    }

    if (ButtonBox1.getRawButtonPressed(6)) {
      AutoBuilder.pathfindToPose(Constants.FieldConstants.F, constraints).until(() -> driverWantsControl()).schedule();
    }

    if (ButtonBox1.getRawButtonPressed(3)) {
      AutoBuilder.pathfindToPose(Constants.FieldConstants.G, constraints).until(() -> driverWantsControl()).schedule();
    }

    if (ButtonBox1.getRawButtonPressed(4)) {
      AutoBuilder.pathfindToPose(Constants.FieldConstants.H, constraints).until(() -> driverWantsControl()).schedule();
    }

    if (ButtonBox1.getRawButtonPressed(1)) {
      AutoBuilder.pathfindToPose(Constants.FieldConstants.I, constraints).until(() -> driverWantsControl()).schedule();
    }

    if (ButtonBox1.getRawButtonPressed(2)) {
      AutoBuilder.pathfindToPose(Constants.FieldConstants.J, constraints).until(() -> driverWantsControl()).schedule();
    }

    if (ButtonBox2.getRawButtonPressed(5)) {
      AutoBuilder.pathfindToPose(Constants.FieldConstants.K, constraints).until(() -> driverWantsControl()).schedule();
    }

    if (ButtonBox2.getRawButtonPressed(6)) {
      AutoBuilder.pathfindToPose(Constants.FieldConstants.L, constraints).until(() -> driverWantsControl()).schedule();
    }

    if (driver.getLeftBumperButtonPressed()) {
      AutoBuilder.pathfindToPose(Constants.FieldConstants.CSL, constraints).until(() -> driverWantsControl()).schedule();
    }

    if (driver.getRightBumperButtonPressed()) {
      AutoBuilder.pathfindToPose(Constants.FieldConstants.CSR, constraints).until(() -> driverWantsControl()).schedule();
    }

    if (driver.getLeftBumperButtonPressed()) {
      AutoBuilder.pathfindToPose(Constants.FieldConstants.CSL, constraints).until(() -> driverWantsControl()).schedule();
    }
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }

  public boolean driverWantsControl() {
    return Math.abs(driver.getLeftX()) > 0.3 ||
            Math.abs(driver.getLeftY()) > 0.3 ||
            Math.abs(driver.getRightX()) > 0.3 ||
            Math.abs(driver.getRightY()) > 0.3;
}

}
