package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
// import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
// import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Vision;
import frc.robot.subsystems.drive.GyroIO.GyroIOInputs;
// import org.littletonrobotics.junction.AutoLogOutput;
// import org.littletonrobotics.junction.AutoLogOutputManager;
// import org.littletonrobotics.junction.Logger;
import org.photonvision.estimation.VisionEstimation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;


public class Drive extends SubsystemBase {
  private static final double MAX_LINEAR_SPEED = 4.5;
  private static final double TRACK_WIDTH_X = Units.inchesToMeters(25.0);
  private static final double TRACK_WIDTH_Y = Units.inchesToMeters(25.0);
  private static final double DRIVE_BASE_RADIUS =
      Math.hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0);
  private static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;
  
  private Field2d odoField2d = new Field2d();

  private final Pose2d photonPose2d = new Pose2d();

  private  GyroIO gyroIO;
  private final GyroIOInputs gyroInputs = new GyroIOInputs();
  
  private  Module[] modules = new Module[4]; // FL, FR, BL, BR
  // private final SysIdRoutine sysId = new SysIdRoutine(
  //   new SysIdRoutine.Config(
  //       null,
  //       null,
  //       null,
  //       (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
  //   new SysIdRoutine.Mechanism(
  //       (voltage) -> {
  //         for (int i = 0; i < 4; i++) {
  //           modules[i].runCharacterization(12);
  //         }
  //       },
  //       null,
  //       this));

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
  private Rotation2d rawGyroRotation = new Rotation2d();
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());
    
  private final Vision vision;

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO,
      Vision vision) {
    this.gyroIO = gyroIO;
    this.vision = vision;
    modules[0] = new Module(flModuleIO, 0);
    modules[1] = new Module(frModuleIO, 1);
    modules[2] = new Module(blModuleIO, 2);
    modules[3] = new Module(brModuleIO, 3);

    // Configure AutoBuilder for PathPlanner
    // AutoBuilder.configure(
    //     this::getPose,
    //     this::setPose,
    //     () -> kinematics.toChassisSpeeds(getModuleStates()),
    //     this::runVelocity,
    //     new HolonomicPathFollowerConfig(
    //         MAX_LINEAR_SPEED, DRIVE_BASE_RADIUS, new ReplanningConfig()),
    //     () ->
    //         DriverStation.getAlliance().isPresent()
    //             && DriverStation.getAlliance().get() == Alliance.Red,
    //     this);
        RobotConfig config;
        try{
        config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
        // Handle exception as needed
        config = null;
        e.printStackTrace();
        }

        AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::setPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> runVelocity(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(3.5, 0.05, 0.15), // Translation PID constants - improved P and added I,D
                    new PIDConstants(4.0, 0.01, 0.2) // Rotation PID constants - improved P and added I,D
            ),
            config, // The robot configurat ion
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
        );}; 
        
        
        
    // Pathfinding.setPathfinder(new LocalADStarAK());
    // PathPlannerLogging.setLogActivePathCallback(
    //     (activePath) -> {
    //       Logger.recordOutput(
    //           "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
    //     });
    // PathPlannerLogging.setLogTargetPoseCallback(
    //     (targetPose) -> {
    //       Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
    //     });

    // Configure SysId
    // Sysid = new SysIdRoutine(
    //         new SysIdRoutine.Config(
    //             null,
    //             null,
    //             null,
    //             (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
    //         new SysIdRoutine.Mechanism(
    //             (voltage) -> {
    //               for (int i = 0; i < 4; i++) {
    //                 modules[i].runCharacterization(12);
    //               }
    //             },
    //             null,
    //             this));
    

  public void periodic() {
    gyroIO.updateInputs(gyroInputs);
    // Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }
    
    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }
    // Log empty setpoint states when disabled
    // if (DriverStation.isDisabled()) {
    //   Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
    //   Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    // }
    
    Runtime runtime = Runtime.getRuntime();
    SmartDashboard.putNumber("Memory/MaxKB", runtime.maxMemory()/1024);
    SmartDashboard.putNumber("Memory/UsedKB", (runtime.totalMemory() - runtime.freeMemory())/1024);
    SmartDashboard.putNumber("Memory/FreeKB", runtime.freeMemory()/1024);

    // Read wheel positions and deltas from each module
    SwerveModulePosition[] modulePositions = getModulePositions();
    SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
    for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
      moduleDeltas[moduleIndex] =
          new SwerveModulePosition(
              modulePositions[moduleIndex].distanceMeters
                  - lastModulePositions[moduleIndex].distanceMeters,
              modulePositions[moduleIndex].angle);
      lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
    }

    // Update gyro angle
    if (gyroInputs.connected) {
      // Use the real gyro angle
      rawGyroRotation = gyroInputs.yawPosition;
    } else {
      // Use the angle delta from the kinematics and module deltas
      Twist2d twist = kinematics.toTwist2d(moduleDeltas);
      rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
    }

    // Apply odometry update
    
    poseEstimator.update(rawGyroRotation, modulePositions);
    odoField2d.setRobotPose(poseEstimator.getEstimatedPosition());
    SmartDashboard.putData("map", odoField2d);

    // Update pose estimator with vision data
    Pose2d visionPose = vision.getLatestPose();
    if (visionPose != null) {
      // Get vision measurement standard deviations (higher values = less trust)
      var stdDevs = vision.getEstimationStdDevs();
      
      // Only use vision measurements that are reasonably trustworthy
      if (stdDevs.get(0, 0) < 1.0) {
        poseEstimator.addVisionMeasurement(
            visionPose,
            Timer.getFPGATimestamp(),
            stdDevs
        );
        
        // Show vision pose on field
        SmartDashboard.putNumber("Vision/X Position (m)", visionPose.getX());
        SmartDashboard.putNumber("Vision/Y Position (m)", visionPose.getY());
        SmartDashboard.putNumber("Vision/Rotation (deg)", visionPose.getRotation().getDegrees());
      }
    }

    // Pose2d currentPose = getPose();
    // SmartDashboard.putNumber("Odometry/X Position (m)", currentPose.getX());
    // SmartDashboard.putNumber("Odometry/Y Position (m)", currentPose.getY());
    // SmartDashboard.putNumber("Odometry/Rotation (deg)", currentPose.getRotation().getDegrees());
    
    // // Get current speeds from module states
    // ChassisSpeeds speeds = kinematics.toChassisSpeeds(getModuleStates());
    // SmartDashboard.putNumber("Robot Speed/X (m/s)", speeds.vxMetersPerSecond);
    // SmartDashboard.putNumber("Robot Speed/Y (m/s)", speeds.vyMetersPerSecond);
    // SmartDashboard.putNumber("Robot Speed/Rotation (rad/s)", speeds.omegaRadiansPerSecond);
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, MAX_LINEAR_SPEED);

    // Send setpoints to modules
    SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      // The module returns the optimized state, useful for logging
      optimizedSetpointStates[i] = modules[i].runSetpoint(setpointStates[i]);
    }

    // // Log setpoint states
    // Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    // Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedSetpointStates);
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  // /** Returns a command to run a quasistatic test in the specified direction. */
  // public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
  //   return sysId.quasistatic(direction);
  // }

  // // /** Returns a command to run a dynamic test in the specified direction. */
  // public Command sysIdDynamic(SysIdRoutine.Direction direction) {
  //   return sysId.dynamic(direction);
  // }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
//   @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the current odometry pose. */
//   @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  /**
   * Adds a vision measurement to the pose estimator.
   *
   * @param visionPose The pose of the robot as measured by the vision camera.
   * @param timestamp The timestamp of the vision measurement in seconds.
   */
  public void addVisionMeasurement(Pose2d visionPose, double timestamp) {
    poseEstimator.addVisionMeasurement(visionPose, timestamp);
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return MAX_LINEAR_SPEED;
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return MAX_ANGULAR_SPEED;
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
      new Translation2d(TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0),
      new Translation2d(-TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
      new Translation2d(-TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0)
    };
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(kinematics.toChassisSpeeds(getModuleStates()), getRotation());
    return chassisSpeeds;
  }

  
  public void end(boolean interrupted) {
      setPose(new Pose2d(new Translation2d() , new Rotation2d()));
  }

  /** Resets the odometry and gyro to zero UwU */
  public void resetPoseToZero() {
    // Reset gyro to zero
    gyroIO.setYaw(0.0);
    // Reset odometry to zero position and rotation
    setPose(new Pose2d(new Translation2d(), new Rotation2d()));
  }

  /**
   * Applies more braking as the robot approaches its target
   * This improves terminal pose accuracy
   */
  public void applyTargetBraking(Pose2d targetPose, double distanceThreshold) {
    Pose2d currentPose = getPose();
    double distance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
    
    // Apply more braking as we get closer to the target
    if (distance < distanceThreshold) {
      // Scale from 0 to 1 based on how close we are to the target
      double brakingFactor = 1.0 - (distance / distanceThreshold);
      
      // Apply braking factor to the modules
      for (var module : modules) {
        module.setBrakeMode(true);
        module.applyBrakingFactor(brakingFactor);
      }
      
      SmartDashboard.putNumber("Path/Braking Factor", brakingFactor);
    }
  }
}
