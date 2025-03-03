package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.NavigationController;
import frc.robot.subsystems.ObjectDetection;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.SuperStruct;
import frc.robot.subsystems.SuperStructState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.superstructure.Elevator;
import frc.robot.subsystems.superstructure.Grabber;
import frc.robot.Vision;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  public final Vision vision = new Vision();
  private final Elevator m_elevator;
  private final Grabber m_grabber;
  public final SuperStruct m_SuperStruct;
  private final NavigationController m_navigationController;
  private final ObjectDetection m_ObjectDetection = new ObjectDetection();
  
  


  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final SendableChooser<Command> autoChooser;
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    m_elevator = Elevator.getInstance();
    m_grabber = Grabber.getInstance();

    

    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementation
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new Swerve(0),
                new Swerve(1),
                new Swerve(2),
                new Swerve(3),
                vision);
        break;
      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                vision);
        break;
    }
    

    m_SuperStruct = SuperStruct.getInstance();  
    m_navigationController = NavigationController.getInstance();
    m_SuperStruct.setObjectDetection(m_ObjectDetection);
    
    if (drive != null) {
        m_ObjectDetection.setDriveSubsystem(drive);
    }
    


    



    NamedCommands.registerCommand("DEFAULT", Commands.runOnce(() -> StateMachine.getInstance().setCommandedState(SuperStructState.DEFAULT), m_elevator).withTimeout(3));
    NamedCommands.registerCommand("L1", Commands.runOnce(() -> StateMachine.getInstance().setCommandedState(SuperStructState.L1), m_elevator));
    NamedCommands.registerCommand("L2", Commands.runOnce(() -> StateMachine.getInstance().setCommandedState(SuperStructState.L2), m_elevator));
    NamedCommands.registerCommand("L3", Commands.runOnce(() -> StateMachine.getInstance().setCommandedState(SuperStructState.L3), m_elevator));
    NamedCommands.registerCommand("L4", Commands.runOnce(() -> StateMachine.getInstance().setCommandedState(SuperStructState.L4), m_elevator));
    NamedCommands.registerCommand("PLACE", Commands.run(() -> StateMachine.getInstance().setCommandedState(SuperStructState.PLACEMENT), m_grabber).withTimeout(1));
    NamedCommands.registerCommand("INTAKE", Commands.run(() -> StateMachine.getInstance().setCommandedState(SuperStructState.CS), m_grabber).withTimeout(1.5));

    // Set up auto routines
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("auto", autoChooser);

    // Configure the button bindings
    configureButtonBindings();

    SmartDashboard.putData("auto", autoChooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
    controller
        .back()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    drive.setPose(new Pose2d());
    return autoChooser.getSelected();
  }
}