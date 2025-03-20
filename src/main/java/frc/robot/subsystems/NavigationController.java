package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
// import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Vision;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.LED;

import java.util.Map;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

public class NavigationController extends SubsystemBase {
    
    public enum DestinationState {
        MANUAL_DRIVING,
        PATHFINDING_TO_A,
        PATHFINDING_TO_B,
        PATHFINDING_TO_AB_THEN_A,
        PATHFINDING_TO_AB_THEN_B,
        PATHFINDING_TO_AB,
        PATHFINDING_TO_C,
        PATHFINDING_TO_D,
        PATHFINDING_TO_CD_THEN_C,
        PATHFINDING_TO_CD_THEN_D,
        PATHFINDING_TO_CD,
        PATHFINDING_TO_E,
        PATHFINDING_TO_F,
        PATHFINDING_TO_EF_THEN_E,
        PATHFINDING_TO_EF_THEN_F,
        PATHFINDING_TO_EF,
        PATHFINDING_TO_G,
        PATHFINDING_TO_H,
        PATHFINDING_TO_GH_THEN_G,
        PATHFINDING_TO_GH_THEN_H,
        PATHFINDING_TO_GH,
        PATHFINDING_TO_I,
        PATHFINDING_TO_J,
        PATHFINDING_TO_IJ_THEN_I,
        PATHFINDING_TO_IJ_THEN_J,
        PATHFINDING_TO_IJ,
        PATHFINDING_TO_K,
        PATHFINDING_TO_L,
        PATHFINDING_TO_KL_THEN_K,
        PATHFINDING_TO_KL_THEN_L,
        PATHFINDING_TO_KL,
        PATHFINDING_TO_CSL,
        PATHFINDING_TO_CSR,
        
        // AprilTag navigation states (simplified)
        PATHFINDING_TO_CLOSEST_TAG,      // In front of the closest tag
        PATHFINDING_TO_LEFT_OF_TAG,      // Left of the closest tag
        PATHFINDING_TO_RIGHT_OF_TAG      // Right of the closest tag
    }
    
    // Controllers
    private final XboxController driver;
    private final Joystick buttonBox1;
    private final Joystick buttonBox2;
    
    // Navigation state
    private DestinationState currentDestination = DestinationState.MANUAL_DRIVING;
    private DestinationState nextDestination = null;
    private Command activePathCommand = null;
    private final PathConstraints constraints;
    private final PathConstraints fastConstraints;
    private final PathConstraints slowConstraints;
    
    // Drive subsystem reference
    private Drive driveSubsystem;
    
    // Vision system reference
    private Vision visionSystem;

    private static NavigationController mInstance = null;

    public static synchronized NavigationController getInstance() {
      if (mInstance == null) {
        mInstance = new NavigationController();
      }
      return mInstance;
    }
    
    /**
     * Creates a new navigation controller to manage path planning
     */
    public NavigationController() {
        driver = new XboxController(0);
        buttonBox1 = new Joystick(1);
        buttonBox2 = new Joystick(2);
        
        // Initialize path constraints 
        this.constraints = new PathConstraints(3, 3, 2 * Math.PI, 4 * Math.PI);
        this.fastConstraints = new PathConstraints(4, 8, 2 * Math.PI, 4 * Math.PI);
        this.slowConstraints = new PathConstraints(2, 1, 2 * Math.PI, 4 * Math.PI);
    }
    
    /**
     * Set the drive subsystem reference
     * Must be called before using path following
     */
    public void setDriveSubsystem(Drive drive) {
        this.driveSubsystem = drive;
    }
    
    /**
     * Set the vision system reference
     * Must be called to enable AprilTag-based navigation
     */
    public void setVisionSystem(Vision vision) {
        this.visionSystem = vision;
    }
    
    @Override
    public void periodic() {
        // Check if driver wants manual control
        if (driverWantsControl()) {
            cancelPathfinding();
        }

        // Check if we completed a step in a multi-step navigation
        if (activePathCommand == null && nextDestination != null) {
            // Start the next step - use slow constraints for the second part
            PathConstraints secondStageConstraints = slowConstraints;
            
            // All second-stage paths should use slow constraints for precise positioning
            Pose2d targetPose = getPoseForDestination(nextDestination);
            
            // Use createAccuratePathCommand for better terminal accuracy
            activePathCommand = createAccuratePathCommand(driveSubsystem, targetPose)
                .until(() -> driverWantsControl())
                .finallyDo((interrupted) -> {
                    if (!interrupted) {
                        currentDestination = DestinationState.MANUAL_DRIVING;
                    } else {
                        currentDestination = DestinationState.MANUAL_DRIVING;
                    }
                    activePathCommand = null;
                });
            
            // Schedule the command
            activePathCommand.schedule();
            
            // Update current destination
            currentDestination = nextDestination;
            nextDestination = null;
        }

        // Always check for correct position near AprilTags during manual driving
        // This allows the driver to see feedback while making adjustments
        if (currentDestination == DestinationState.MANUAL_DRIVING) {
            // Check if we're at the correct distance from left or right side of any tag
            if (isAtTargetPosition(0.55, -0.164, 0.1)) {
                // Blink green when at left position of tag
                LED.getInstance().blinkSection1(0, 255, 0, 1.5);
                SmartDashboard.putString("Position", "At left side of tag");
            } else if (isAtTargetPosition(0.55, 0.164, 0.1)) {
                // Blink green when at right position of tag
                LED.getInstance().blinkSection1(0, 255, 0, 1.5);
                SmartDashboard.putString("Position", "At right side of tag");
            } else {
                // Check if we're close but not quite at the target position
                boolean nearLeftPosition = isAtTargetPosition(0.55, -0.164, 0.3);
                boolean nearRightPosition = isAtTargetPosition(0.55, 0.164, 0.3);
                
                if (nearLeftPosition || nearRightPosition) {
                    // Yellow when near but not exactly at position - needs adjustment
                    LED.getInstance().blinkSection1(255, 255, 0, 1.0);
                    SmartDashboard.putString("Position", "Near tag position - adjusting");
                } else {
                    // No special position feedback
                    SmartDashboard.putString("Position", "Not near tag position");
                }
            }
        }

        // Check button presses to set new destinations
        DestinationState newDestination = checkButtonPresses();
        
        if (newDestination != currentDestination) {
            startPathfinding(newDestination);
        }
    }
    
    /**
     * Checks all controller inputs and returns the requested destination
     */
    private DestinationState checkButtonPresses() {
        
        if (buttonBox1.getRawAxis(1) >= 0.5) {
            return DestinationState.PATHFINDING_TO_AB_THEN_A;
        } else if (buttonBox1.getRawButtonPressed(5)) {
            return DestinationState.PATHFINDING_TO_AB_THEN_B;
        } else if (buttonBox1.getRawButtonPressed(6)) {
            return DestinationState.PATHFINDING_TO_CD_THEN_C;
        } else if (buttonBox1.getRawButtonPressed(1)) {
            return DestinationState.PATHFINDING_TO_CD_THEN_D;
        } else if (buttonBox1.getRawButtonPressed(10)) {
            return DestinationState.PATHFINDING_TO_EF_THEN_E;
        } else if (buttonBox1.getRawButtonPressed(9)) {
            return DestinationState.PATHFINDING_TO_EF_THEN_F;
        } else if (buttonBox1.getRawButtonPressed(8)) {
            return DestinationState.PATHFINDING_TO_GH_THEN_G;
        } else if (buttonBox1.getRawButtonPressed(7)) {
            return DestinationState.PATHFINDING_TO_GH_THEN_H;
        } else if (buttonBox1.getRawButtonPressed(4)) {
            return DestinationState.PATHFINDING_TO_IJ_THEN_I;
        } else if (buttonBox1.getRawButtonPressed(3)) {
            return DestinationState.PATHFINDING_TO_IJ_THEN_J;
        } else if (buttonBox1.getRawButtonPressed(2)) {
            return DestinationState.PATHFINDING_TO_KL_THEN_K;
        } else if (buttonBox2.getRawButtonPressed(3)) {
            return DestinationState.PATHFINDING_TO_KL_THEN_L;
        // } else if (driver.getLeftBumperPressed()) {
        //     return DestinationState.PATHFINDING_TO_CSL;
        // } else if (driver.getRightBumperPressed()) {
        //     return DestinationState.PATHFINDING_TO_CSR;
        }// Check for visible AprilTags and show on dashboard, even when not navigating
        if (visionSystem != null) {
            int closestTag = findClosestVisibleTag();
            if (closestTag >= 0) {
                // Found a tag!
                SmartDashboard.putBoolean("Navigation/TagVisible", true);
                SmartDashboard.putNumber("Navigation/ClosestTagID", closestTag);
                
                // If one of the auto-tag buttons is held and we're in manual driving,
                // automatically start the navigation
                if (currentDestination == DestinationState.MANUAL_DRIVING) {
                    if (driver.getYButton()) {
                        startPathfinding(DestinationState.PATHFINDING_TO_CLOSEST_TAG);
                    } else if (driver.getLeftTriggerAxis() > 0.6) {
                        startPathfinding(DestinationState.PATHFINDING_TO_LEFT_OF_TAG);
                    } else if (driver.getRightTriggerAxis() > 0.6) {
                        startPathfinding(DestinationState.PATHFINDING_TO_RIGHT_OF_TAG);
                    }
                }
            } else {
                SmartDashboard.putBoolean("Navigation/TagVisible", false);
                SmartDashboard.putNumber("Navigation/ClosestTagID", -1);
            }
        }
        
       
        
        // Left/Right bumpers - navigate to left/right of the closest tag
        // if (driver.getLeftBumperPressed()) {
        //     return DestinationState.PATHFINDING_TO_LEFT_OF_TAG;
        // } else if (driver.getRightBumperPressed()) {
        //     return DestinationState.PATHFINDING_TO_RIGHT_OF_TAG;
        // }
        
        return currentDestination;
    }
    
    /**
     * Starts pathfinding to the specified destination
     */
    public void startPathfinding(DestinationState destination) {
        // Cancel any existing pathfinding
        cancelPathfinding();
        
        PathConstraints currentConstraints = constraints;
        
        if (destination == DestinationState.PATHFINDING_TO_AB_THEN_A) {
            nextDestination = DestinationState.PATHFINDING_TO_A;
            destination = DestinationState.PATHFINDING_TO_AB;
            currentConstraints = fastConstraints; // Use faster constraints for first part
        } else if (destination == DestinationState.PATHFINDING_TO_AB_THEN_B) {
            nextDestination = DestinationState.PATHFINDING_TO_B;
            destination = DestinationState.PATHFINDING_TO_AB;
            currentConstraints = fastConstraints; // Use faster constraints for first part
        } else if (destination == DestinationState.PATHFINDING_TO_CD_THEN_C) {
            nextDestination = DestinationState.PATHFINDING_TO_C;
            destination = DestinationState.PATHFINDING_TO_CD;
            currentConstraints = fastConstraints; // Use faster constraints for first part
        } else if (destination == DestinationState.PATHFINDING_TO_CD_THEN_D) {
            nextDestination = DestinationState.PATHFINDING_TO_D;
            destination = DestinationState.PATHFINDING_TO_CD;
            currentConstraints = fastConstraints; // Use faster constraints for first part
        } else if (destination == DestinationState.PATHFINDING_TO_EF_THEN_E) {
            nextDestination = DestinationState.PATHFINDING_TO_E;
            destination = DestinationState.PATHFINDING_TO_EF;
            currentConstraints = fastConstraints; // Use faster constraints for first part
        } else if (destination == DestinationState.PATHFINDING_TO_EF_THEN_F) {
            nextDestination = DestinationState.PATHFINDING_TO_F;
            destination = DestinationState.PATHFINDING_TO_EF;
            currentConstraints = fastConstraints; // Use faster constraints for first part
        } else if (destination == DestinationState.PATHFINDING_TO_GH_THEN_G) {
            nextDestination = DestinationState.PATHFINDING_TO_G;
            destination = DestinationState.PATHFINDING_TO_GH;
            currentConstraints = fastConstraints; // Use faster constraints for first part
        } else if (destination == DestinationState.PATHFINDING_TO_GH_THEN_H) {
            nextDestination = DestinationState.PATHFINDING_TO_H;
            destination = DestinationState.PATHFINDING_TO_GH;
            currentConstraints = fastConstraints; // Use faster constraints for first part
        } else if (destination == DestinationState.PATHFINDING_TO_IJ_THEN_I) {
            nextDestination = DestinationState.PATHFINDING_TO_I;
            destination = DestinationState.PATHFINDING_TO_IJ;
            currentConstraints = fastConstraints; // Use faster constraints for first part
        } else if (destination == DestinationState.PATHFINDING_TO_IJ_THEN_J) {
            nextDestination = DestinationState.PATHFINDING_TO_J;
            destination = DestinationState.PATHFINDING_TO_IJ;
            currentConstraints = fastConstraints; // Use faster constraints for first part
        } else if (destination == DestinationState.PATHFINDING_TO_KL_THEN_K) {
            nextDestination = DestinationState.PATHFINDING_TO_K;
            destination = DestinationState.PATHFINDING_TO_KL;
            currentConstraints = fastConstraints; // Use faster constraints for first part
        } else if (destination == DestinationState.PATHFINDING_TO_KL_THEN_L) {
            nextDestination = DestinationState.PATHFINDING_TO_L;
            destination = DestinationState.PATHFINDING_TO_KL;
            currentConstraints = fastConstraints; // Use faster constraints for first part
        } else if (destination == DestinationState.PATHFINDING_TO_CLOSEST_TAG) {
            // Navigate directly in front of the closest tag
            boolean success = navigateToClosestTag(1.5, 0.0, null);
            if (success) {
                SmartDashboard.putString("Navigation/Status", "Navigating to closest tag");
            } else {
                SmartDashboard.putString("Navigation/Status", "No AprilTags visible");
            }
            return;
        } else if (destination == DestinationState.PATHFINDING_TO_LEFT_OF_TAG) {
            // Navigate to the left side of the closest tag
            boolean success = navigateToClosestTag(0.55, -0.164, null);  // 1.0m to the left
            if (success) {
                SmartDashboard.putString("Navigation/Status", "Navigating to left of closest tag");
                // Check if we're at the correct distance
                if (isAtTargetPosition(0.55, -0.164, 0.01)) {
                    // Blink green when at correct position
                    LED.getInstance().blinkSection1(0, 255, 0, 1.5);
                }
            } else {
                SmartDashboard.putString("Navigation/Status", "No AprilTags visible");
            }
            return;
        } else if (destination == DestinationState.PATHFINDING_TO_RIGHT_OF_TAG) {
            // Navigate to the right side of the closest tag
            boolean success = navigateToClosestTag(0.55, 0.164, null);  // -1.0m to the left (= right)
            if (success) {
                SmartDashboard.putString("Navigation/Status", "Navigating to right of closest tag");
                // Check if we're at the correct distance
                if (isAtTargetPosition(0.55, 0.164, 0.01)) {
                    // Blink green when at correct position
                    LED.getInstance().blinkSection1(0, 255, 0, 1.5);
                }
            } else {
                SmartDashboard.putString("Navigation/Status", "No AprilTags visible");
            }
            return;
        }
        
        // Find the target pose based on the destination
        Pose2d targetPose = getPoseForDestination(destination);

        // Use faster constraints for intermediate points
        if (destination == DestinationState.PATHFINDING_TO_AB ||
            destination == DestinationState.PATHFINDING_TO_CD ||
            destination == DestinationState.PATHFINDING_TO_EF ||
            destination == DestinationState.PATHFINDING_TO_GH ||
            destination == DestinationState.PATHFINDING_TO_IJ ||
            destination == DestinationState.PATHFINDING_TO_KL) {
            currentConstraints = fastConstraints;
        }
        
        // Create the pathfinding command
        boolean useAccurateNavigation = 
            destination == DestinationState.PATHFINDING_TO_A ||
            destination == DestinationState.PATHFINDING_TO_B ||
            destination == DestinationState.PATHFINDING_TO_C ||
            destination == DestinationState.PATHFINDING_TO_D ||
            destination == DestinationState.PATHFINDING_TO_E ||
            destination == DestinationState.PATHFINDING_TO_F ||
            destination == DestinationState.PATHFINDING_TO_G ||
            destination == DestinationState.PATHFINDING_TO_H ||
            destination == DestinationState.PATHFINDING_TO_I ||
            destination == DestinationState.PATHFINDING_TO_J ||
            destination == DestinationState.PATHFINDING_TO_K ||
            destination == DestinationState.PATHFINDING_TO_L ||
            destination == DestinationState.PATHFINDING_TO_CSL ||
            destination == DestinationState.PATHFINDING_TO_CSR;
            
        // Use accurate navigation for important destinations
        if (useAccurateNavigation && driveSubsystem != null) {
            SmartDashboard.putString("Navigation/Status", "Using enhanced accurate navigation");
            activePathCommand = createAccuratePathCommand(driveSubsystem, targetPose)
                .until(() -> driverWantsControl())
                .finallyDo((interrupted) -> {
                    if (!interrupted) {
                        if (nextDestination == null) {
                            currentDestination = DestinationState.MANUAL_DRIVING;
                        }
                    } else {
                        nextDestination = null;
                        currentDestination = DestinationState.MANUAL_DRIVING;
                    }
                    activePathCommand = null;
                    SmartDashboard.putString("Navigation/Status", "Ready");
                });
        } else {
            // Use standard pathfinding for other destinations
            activePathCommand = AutoBuilder.pathfindToPose(targetPose, currentConstraints)
                .until(() -> driverWantsControl())
                .finallyDo((interrupted) -> {
                    if (!interrupted) {
                        if (nextDestination == null) {
                            currentDestination = DestinationState.MANUAL_DRIVING;
                        }
                    } else {
                        nextDestination = null;
                        currentDestination = DestinationState.MANUAL_DRIVING;
                    }
                    activePathCommand = null;
                });
        }
        
        // Schedule the command
        activePathCommand.schedule();
        
        // Update current destination
        currentDestination = destination;
    }
    
    /**
     * Returns the target pose for the given destination
     */
    private Pose2d getPoseForDestination(DestinationState destination) {
        switch (destination) {
            case PATHFINDING_TO_A:
                return Constants.FieldConstants.A;
            case PATHFINDING_TO_B:
                return Constants.FieldConstants.B;
            case PATHFINDING_TO_AB:
                return Constants.FieldConstants.AB;
            case PATHFINDING_TO_C:
                return Constants.FieldConstants.C;
            case PATHFINDING_TO_D:
                return Constants.FieldConstants.D;
            case PATHFINDING_TO_CD:
                return Constants.FieldConstants.CD;
            case PATHFINDING_TO_E:
                return Constants.FieldConstants.E;
            case PATHFINDING_TO_F:
                return Constants.FieldConstants.F;
            case PATHFINDING_TO_EF:
                return Constants.FieldConstants.EF;
            case PATHFINDING_TO_G:
                return Constants.FieldConstants.G;
            case PATHFINDING_TO_H:
                return Constants.FieldConstants.H;
            case PATHFINDING_TO_GH:
                return Constants.FieldConstants.GH;
            case PATHFINDING_TO_I:
                return Constants.FieldConstants.I;
            case PATHFINDING_TO_J:
                return Constants.FieldConstants.J;
            case PATHFINDING_TO_IJ:
                return Constants.FieldConstants.IJ;
            case PATHFINDING_TO_K:
                return Constants.FieldConstants.K;
            case PATHFINDING_TO_L:
                return Constants.FieldConstants.L;
            case PATHFINDING_TO_KL:
                return Constants.FieldConstants.KL;
            case PATHFINDING_TO_CSL:
                return Constants.FieldConstants.CSL;
            case PATHFINDING_TO_CSR:
                return Constants.FieldConstants.CSR;
            default:
                return null;
        }
    }
    
    /**
     * Cancels any active pathfinding operation
     */
    public void cancelPathfinding() {
        if (activePathCommand != null) {
            activePathCommand.cancel();
            activePathCommand = null;
        }
        currentDestination = DestinationState.MANUAL_DRIVING;
    }
    
    /**
     * Checks if the driver is attempting to take manual control
     */
    private boolean driverWantsControl() {
        return Math.abs(driver.getLeftX()) > 0.3 ||
               Math.abs(driver.getLeftY()) > 0.3 ||
               Math.abs(driver.getRightX()) > 0.3 ||
               Math.abs(driver.getRightY()) > 0.3;
    }

    /**
     * Creates a path command to a target pose with improved accuracy
     * Uses vision-based localization and terminal pose correction
     */
    public Command createAccuratePathCommand(Drive drive, Pose2d targetPose) {
        // Build the path command
        Command pathCommand = AutoBuilder.pathfindToPose(
            targetPose, 
            constraints,
            0.0 // Goal end velocity
        );
        
        // Combine with our terminal pose accuracy command
        return pathCommand
            .andThen(DriveCommands.improvePathEndAccuracy(drive, targetPose))
            .withName("AccuratePath");
    }
    
    /**
     * Creates a path command to a target pose relative to an AprilTag
     * @param tagId The ID of the AprilTag to navigate relative to
     * @param relativeOffset The offset from the tag (x forward, y left)
     * @param targetHeading The desired heading at the target position
     */
    public Command createTagRelativePathCommand(Drive drive, int tagId, Translation2d relativeOffset, Rotation2d targetHeading) {
        // Check if vision system is available
        if (visionSystem == null) {
            SmartDashboard.putString("Navigation/Error", "Vision system not connected");
            return Commands.none();
        }
        
        // Get the tag pose from the field layout
        var tagPose = Constants.Vision.kTagLayout.getTagPose(tagId);
        
        if (tagPose.isEmpty()) {
            SmartDashboard.putString("Navigation/Error", "Tag ID " + tagId + " not found in field layout");
            return Commands.none();
        }
        
        // Calculate the target pose by applying the relative offset to the tag pose
        Pose2d tagPose2d = tagPose.get().toPose2d();
        
        // Transform the offset based on tag rotation
        Translation2d rotatedOffset = new Translation2d(
            relativeOffset.getX() * tagPose2d.getRotation().getCos() - relativeOffset.getY() * tagPose2d.getRotation().getSin(),
            relativeOffset.getX() * tagPose2d.getRotation().getSin() + relativeOffset.getY() * tagPose2d.getRotation().getCos()
        );
        
        // Apply the offset to the tag position
        Pose2d targetPose = new Pose2d(
            tagPose2d.getX() + rotatedOffset.getX(),
            tagPose2d.getY() + rotatedOffset.getY(),
            targetHeading
        );
        
        SmartDashboard.putString("Navigation/TagRelativeTarget", "Tag: " + tagId + 
                                ", X: " + targetPose.getX() + 
                                ", Y: " + targetPose.getY());
        
        // Create a path to this target pose
        return createAccuratePathCommand(drive, targetPose);
    }
    
    /**
     * Navigate to a position relative to the specified AprilTag
     * @param tagId The ID of the AprilTag to navigate relative to
     * @param xOffset Forward offset from tag in meters (positive is in front of tag)
     * @param yOffset Left offset from tag in meters (positive is to the left of tag)
     * @param headingDegrees Desired robot heading in degrees
     */
    public void navigateToTagRelative(int tagId, double xOffset, double yOffset, double headingDegrees) {
        // Cancel existing navigation
        cancelPathfinding();
        
        Translation2d offset = new Translation2d(xOffset, yOffset);
        Rotation2d heading = Rotation2d.fromDegrees(headingDegrees);
        
        // Create and schedule the command
        activePathCommand = createTagRelativePathCommand(driveSubsystem, tagId, offset, heading)
            .until(() -> driverWantsControl())
            .finallyDo((interrupted) -> {
                activePathCommand = null;
                currentDestination = DestinationState.MANUAL_DRIVING;
                SmartDashboard.putString("Navigation/Status", "Ready");
            });
        
        activePathCommand.schedule();
        currentDestination = DestinationState.MANUAL_DRIVING; // We'll need to extend the enum for tags
    }
    
    /**
     * Find the closest visible AprilTag and return its ID
     * @return The ID of the closest visible tag, or -1 if no tags are visible
     */
    public int findClosestVisibleTag() {
        if (visionSystem == null) {
            return -1;
        }
        
        // Get all visible tag translations
        Map<String, Translation3d> tagTranslations = visionSystem.getAllTagTranslations();
        
        // Find the closest tag based on distance
        double closestDistance = Double.MAX_VALUE;
        int closestTagId = -1;
        
        for (Map.Entry<String, Translation3d> entry : tagTranslations.entrySet()) {
            String cameraName = entry.getKey();
            // Get the camera that detected this tag
            PhotonCamera camera = visionSystem.getCamera(cameraName);
            var result = camera.getLatestResult();
            
            if (result.hasTargets()) {
                // Find the tag with the smallest distance (closest)
                PhotonTrackedTarget target = result.getBestTarget();
                int tagId = target.getFiducialId();
                double distance = entry.getValue().getNorm();
                
                if (distance < closestDistance) {
                    closestDistance = distance;
                    closestTagId = tagId;
                }
            }
        }
        
        return closestTagId;
    }
    
    /**
     * Navigate to the closest visible AprilTag
     * @param xOffset Forward offset from tag in meters (positive is in front of tag)
     * @param yOffset Left offset from tag in meters (positive is to the left of tag)
     * @param headingDegrees Desired robot heading in degrees, or null to automatically face the tag
     * @return True if navigation started, false if no tags were visible
     */
    public boolean navigateToClosestTag(double xOffset, double yOffset, Double headingDegrees) {
        // Find the closest visible tag
        int tagId = findClosestVisibleTag();
        
        if (tagId < 0) {
            // No tags visible
            SmartDashboard.putString("Navigation/Error", "No AprilTags visible");
            return false;
        }
        
        // Get the tag pose to calculate heading
        if (headingDegrees == null) {
            // Auto-calculate heading to face the tag
            var tagPose = Constants.Vision.kTagLayout.getTagPose(tagId);
            if (tagPose.isPresent()) {
                // Get the tag's rotation, then invert it (to face it)
                Rotation2d tagRotation = tagPose.get().toPose2d().getRotation();
                double autoHeading = tagRotation.getDegrees() + 180.0; // Add 180° to face the tag
                
                // Navigate to the tag with auto-calculated heading
                navigateToTagRelative(tagId, xOffset, yOffset, autoHeading);
                SmartDashboard.putNumber("Navigation/AutoHeading", autoHeading);
                return true;
            }
        }
        
        // Use specified heading (or default to 180° if auto failed)
        double heading = (headingDegrees != null) ? headingDegrees : 180.0;
        
        // Navigate to the tag with specified or default heading
        navigateToTagRelative(tagId, xOffset, yOffset, heading);
        return true;
    }

    /**
     * Get the current destination state
     * @return The current destination state
     */
    public DestinationState getCurrentDestination() {
        return currentDestination;
    }

    /**
     * Checks if the robot is at the target position
     * @param x The x coordinate of the target position
     * @param y The y coordinate of the target position
     * @param tolerance The tolerance for the distance to the target position
     * @return True if the robot is at the target position, false otherwise
     */
    private boolean isAtTargetPosition(double x, double y, double tolerance) {
        // Get the closest visible tag
        int tagId = findClosestVisibleTag();
        if (tagId < 0) {
            return false; // No tags visible
        }

        // Get the current robot pose
        Pose2d robotPose = driveSubsystem.getPose();

        // Get the tag pose
        var tagPoseOptional = Constants.Vision.kTagLayout.getTagPose(tagId);
        if (tagPoseOptional.isEmpty()) {
            return false; // Tag not found in field layout
        }

        // Calculate the target position relative to the tag
        Pose2d tagPose = tagPoseOptional.get().toPose2d();
        
        // Create a transformation relative to the tag's coordinate system
        // x = forward from tag, y = left from tag
        Transform2d relativeTransform = new Transform2d(
            new Translation2d(x, y),
            new Rotation2d() // No rotation
        );
        
        // Apply the transformation to get the absolute target position
        Pose2d targetPose = tagPose.plus(relativeTransform);
        
        // Calculate distance from robot to target
        double distance = robotPose.getTranslation().getDistance(targetPose.getTranslation());
        
        // Check if we're within tolerance
        return distance <= tolerance;
    }
} 