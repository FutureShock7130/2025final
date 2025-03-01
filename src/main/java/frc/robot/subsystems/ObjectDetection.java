package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.subsystems.drive.Drive;
import java.util.List;
import java.util.Map;
import java.util.HashMap;
import java.util.Optional;
import java.util.stream.Collectors;
import java.util.Comparator;
import java.util.Iterator;
import java.util.EnumSet;

/**
 * Object Detection Subsystem
 * This subsystem uses PhotonVision to detect objects classified as "0" (algae) or "1" (coral)
 * Provides the following functions:
 * - Control tracking features through Shuffleboard
 * - Track and follow detected objects
 * - Auto-aim at objects
 * - Track closest objects
 * - Estimate object distance
 * - Calculate object positions on the field
 */
public class ObjectDetection extends SubsystemBase {
    private final PhotonCamera camera;
    private int targetClass = 0; // Default to track class 0 (algae)
    private final PIDController turnController;
    private final PIDController driveController;
    private Drive driveSubsystem;
    
    // Game object size definitions (in meters)
    private static final double ALGAE_DIAMETER = 0.413; // Algae diameter
    private static final double CORAL_LENGTH = 0.30;   // Coral length
    
    // Camera settings
    private static final double CAMERA_HORIZONTAL_FOV = 70.0; // Camera horizontal field of view
    private static final double CAMERA_RESOLUTION_WIDTH = 640.0; // Camera resolution width
    
    // Command states
    private boolean isAiming = false;
    private boolean isFollowing = false;
    private Command activeCommand = null;
    
    // Object tracking system
    private Map<Integer, TrackedObject> trackedObjects = new HashMap<>();
    private Field2d fieldWidget = new Field2d();
    
    // Shuffleboard entries
    private final ShuffleboardTab visionTab = Shuffleboard.getTab("Object Detection");
    private GenericEntry targetClassEntry;
    private GenericEntry targetAreaEntry;
    private GenericEntry aimToleranceEntry;
    private GenericEntry aimButtonEntry;
    private GenericEntry followButtonEntry;
    private GenericEntry stopButtonEntry;
    private GenericEntry distanceEstimateEntry;
    
    private static final double TARGET_AREA_SETPOINT = 37.0; 
    private static final double AIM_TOLERANCE_DEGREES = 2.0; 
    
    // Filtering related variables
    private double filteredYaw = 0.0;
    private double filteredArea = 0.0;
    private double lastTurnOutput = 0.0;
    private double lastDriveOutput = 0.0;
    
    // Adjustable variables
    private double SLEW_RATE_LIMIT = 0.05;  // Output change rate limit
    
    // Fields for storing historical data
    private final int FILTER_HISTORY_SIZE = 10; // Size of history buffer
    private double[] yawHistory = new double[FILTER_HISTORY_SIZE];
    private double[] areaHistory = new double[FILTER_HISTORY_SIZE];
    
    // Convolution kernel options
    private double[] uniformKernel; // Uniform convolution kernel
    private double[] gaussianKernel; // Gaussian convolution kernel
    private double[] currentYawKernel; // Current yaw convolution kernel
    private double[] currentAreaKernel; // Current area convolution kernel
    
    /**
     * Information about a tracked object
     */
    public class TrackedObject {
        public int id;
        public Pose2d pose;
        public double distance;
        public double lastUpdated;
        
        public TrackedObject(int id, Pose2d pose, double distance) {
            this.id = id;
            this.pose = pose;
            this.distance = distance;
            this.lastUpdated = Timer.getFPGATimestamp();
        }
        
        public boolean isStale() {
            // Consider stale if not updated for 3 seconds
            return Timer.getFPGATimestamp() - lastUpdated > 3.0;
        }
    }
    
    public ObjectDetection() {
        camera = new PhotonCamera("WEB_CAM");
        turnController = new PIDController(0.007, 0, 0.0001); // Turn PID
        driveController = new PIDController(0.05, 0, 0.0001); // Approach PID
        
        // Initialize convolution kernels
        uniformKernel = createUniformKernel(5); // Size 5 uniform kernel
        gaussianKernel = createGaussianKernel(5, 1.0); // Size 5, sigma 1.0 Gaussian kernel
        
        // Default to gaussian kernel
        currentYawKernel = gaussianKernel;
        currentAreaKernel = gaussianKernel;
        
        // Initialize history data
        for (int i = 0; i < FILTER_HISTORY_SIZE; i++) {
            yawHistory[i] = 0.0;
            areaHistory[i] = 0.0;
        }
        
        setupShuffleboardControls();
    }
    
    /**
     * Create 1D uniform convolution kernel
     */
    private double[] createUniformKernel(int size) {
        double[] kernel = new double[size];
        for (int i = 0; i < size; i++) {
            kernel[i] = 1.0 / size;
        }
        return kernel;
    }
    
    /**
     * Create 1D Gaussian convolution kernel
     */
    private double[] createGaussianKernel(int size, double sigma) {
        double[] kernel = new double[size];
        double sum = 0.0;
        int center = size / 2;
        
        for (int i = 0; i < size; i++) {
            int x = i - center;
            kernel[i] = Math.exp(-(x*x) / (2*sigma*sigma));
            sum += kernel[i];
        }
        
        // Normalize kernel
        for (int i = 0; i < size; i++) {
            kernel[i] /= sum;
        }
        
        return kernel;
    }
    
    /**
     * Apply 1D convolution filter
     */
    private double applyConvolution(double[] data, double[] kernel) {
        if (data.length < kernel.length) {
            // Not enough data, return most recent value
            return data[0];
        }
        
        double result = 0.0;
        for (int i = 0; i < kernel.length; i++) {
            result += data[i] * kernel[i];
        }
        
        return result;
    }
    
    /**
     * Update history array
     */
    private void updateHistory(double[] history, double newValue) {
        // Shift all values back one position
        for (int i = history.length - 1; i > 0; i--) {
            history[i] = history[i-1];
        }
        // Put new value at front
        history[0] = newValue;
    }
    
    /**
     * Setup Shuffleboard controls
     */
    private void setupShuffleboardControls() {
        targetClassEntry = visionTab.add("Target Class", targetClass)
            .withPosition(0, 0)
            .withSize(1, 1)
            .withProperties(Map.of("subtitle", "Target Class 0=algae, 1=coral"))
            .getEntry();
        
        targetAreaEntry = visionTab.add("Target Area Size", TARGET_AREA_SETPOINT)
            .withPosition(1, 0)
            .withSize(1, 1)
            .withProperties(Map.of("subtitle", "Target Area"))
            .getEntry();
        
        aimToleranceEntry = visionTab.add("Aim Tolerance", AIM_TOLERANCE_DEGREES)
            .withPosition(2, 0)
            .withSize(1, 1)
            .withProperties(Map.of("subtitle", "Aim Tolerance (deg)"))
            .getEntry();
        
        aimButtonEntry = visionTab.add("Aim at Target", false)
            .withPosition(0, 1)
            .withSize(1, 1)
            .withProperties(Map.of("colorWhenTrue", "green", "subtitle", "Aim at Target"))
            .getEntry();
        
        followButtonEntry = visionTab.add("Follow Target", false)
            .withPosition(1, 1)
            .withSize(1, 1)
            .withProperties(Map.of("colorWhenTrue", "blue", "subtitle", "Follow Target"))
            .getEntry();
        
        stopButtonEntry = visionTab.add("Stop", false)
            .withPosition(2, 1)
            .withSize(1, 1)
            .withProperties(Map.of("colorWhenTrue", "red", "subtitle", "Stop"))
            .getEntry();
        
        distanceEstimateEntry = visionTab.add("Estimated Distance", 0.0)
            .withPosition(3, 0)
            .withSize(1, 1)
            .withProperties(Map.of("subtitle", "Estimated Distance (m)"))
            .getEntry();
        
        visionTab.addBoolean("Target Visible", this::isTargetVisible)
            .withPosition(0, 2)
            .withSize(1, 1)
            .withProperties(Map.of("subtitle", "Target Visible"));
        
        visionTab.addBoolean("Aimed", this::isAimedAtTarget)
            .withPosition(1, 2)
            .withSize(1, 1)
            .withProperties(Map.of("subtitle", "Aimed"));
        
        visionTab.addString("Status", () -> {
            if (isFollowing) return "Following";
            if (isAiming) return "Aiming";
            return "Idle";
        })
            .withPosition(2, 2)
            .withSize(1, 1)
            .withProperties(Map.of("subtitle", "Status"));
            
        visionTab.addString("Target Class Name", () -> 
            targetClass == 0 ? "Algae" : "Coral")
            .withPosition(3, 1)
            .withSize(1, 1)
            .withProperties(Map.of("subtitle", "Target Class Name"));
            
        // Add field visualization
        visionTab.add("Field", fieldWidget)
            .withSize(5, 3)
            .withPosition(0, 3)
            .withProperties(Map.of("subtitle", "Field"));
            
        // Display tracked objects list
        visionTab.addString("Tracked Objects", this::getTrackedObjectsAsString)
            .withSize(2, 3)
            .withPosition(5, 3)
            .withProperties(Map.of("subtitle", "Tracked Objects"));
            
        // Add debug values
        visionTab.addNumber("Target Yaw", () -> {
            var result = camera.getLatestResult();
            if (result.hasTargets()) {
                var target = getBestTarget(result);
                return target != null ? target.getYaw() : 0.0;
            }
            return 0.0;
        })
            .withPosition(3, 2)
            .withSize(1, 1)
            .withProperties(Map.of("subtitle", "Target Yaw"));
            
        visionTab.addNumber("PID Turn Error", () -> {
            var result = camera.getLatestResult();
            if (result.hasTargets()) {
                var target = getBestTarget(result);
                return target != null ? turnController.getPositionError() : 0.0;
            }
            return 0.0;
        })
            .withPosition(4, 2)
            .withSize(1, 1)
            .withProperties(Map.of("subtitle", "PID Turn Error"));
        
        // Add filter settings tab
        ShuffleboardTab filterTab = Shuffleboard.getTab("Filter Settings");
        
        // Yaw filter settings
        filterTab.addString("Yaw Filter Type", () -> 
            (currentYawKernel == gaussianKernel) ? "Gaussian" : "Uniform")
            .withPosition(0, 0)
            .withSize(1, 1)
            .withProperties(Map.of("subtitle", "Yaw Filter Type"));
        
        // Button to toggle yaw filter type
        GenericEntry yawFilterToggleEntry = filterTab.add("Toggle Yaw Filter", false)
            .withPosition(1, 0)
            .withSize(1, 1)
            .withProperties(Map.of("colorWhenTrue", "green", "subtitle", "Toggle Yaw Filter"))
            .getEntry();
            
        NetworkTableInstance.getDefault().addListener(
            yawFilterToggleEntry.getTopic(),
            EnumSet.of(NetworkTableEvent.Kind.kValueAll),
            event -> {
                if (event.valueData.value.getBoolean()) {
                    currentYawKernel = (currentYawKernel == gaussianKernel) ? 
                        uniformKernel : gaussianKernel;
                    yawFilterToggleEntry.setBoolean(false);
                }
            });
        
        // Area filter settings
        filterTab.addString("Area Filter Type", () -> 
            (currentAreaKernel == gaussianKernel) ? "Gaussian" : "Uniform")
            .withPosition(0, 1)
            .withSize(1, 1)
            .withProperties(Map.of("subtitle", "Area Filter Type"));
        
        // Button to toggle area filter type
        GenericEntry areaFilterToggleEntry = filterTab.add("Toggle Area Filter", false)
            .withPosition(1, 1)
            .withSize(1, 1)
            .withProperties(Map.of("colorWhenTrue", "green", "subtitle", "Toggle Area Filter"))
            .getEntry();
            
        NetworkTableInstance.getDefault().addListener(
            areaFilterToggleEntry.getTopic(),
            EnumSet.of(NetworkTableEvent.Kind.kValueAll),
            event -> {
                if (event.valueData.value.getBoolean()) {
                    currentAreaKernel = (currentAreaKernel == gaussianKernel) ? 
                        uniformKernel : gaussianKernel;
                    areaFilterToggleEntry.setBoolean(false);
                }
            });
        
        // Display filtered values
        filterTab.addNumber("Raw Yaw", () -> {
            var result = camera.getLatestResult();
            if (result.hasTargets()) {
                var target = getBestTarget(result);
                return target != null ? target.getYaw() : 0.0;
            }
            return 0.0;
        })
            .withPosition(2, 0)
            .withSize(1, 1)
            .withProperties(Map.of("subtitle", "Raw Yaw"));
        
        filterTab.addNumber("Filtered Yaw", () -> filteredYaw)
            .withPosition(3, 0)
            .withSize(1, 1)
            .withProperties(Map.of("subtitle", "Filtered Yaw"));
        
        filterTab.addNumber("Raw Area", () -> {
            var result = camera.getLatestResult();
            if (result.hasTargets()) {
                var target = getBestTarget(result);
                return target != null ? target.getArea() : 0.0;
            }
            return 0.0;
        })
            .withPosition(2, 1)
            .withSize(1, 1)
            .withProperties(Map.of("subtitle", "Raw Area"));
        
        filterTab.addNumber("Filtered Area", () -> filteredArea)
            .withPosition(3, 1)
            .withSize(1, 1)
            .withProperties(Map.of("subtitle", "Filtered Area"));
        
        // Slew rate limit settings
        filterTab.addNumber("Slew Rate Limit", () -> SLEW_RATE_LIMIT)
            .withPosition(0, 2)
            .withSize(1, 1)
            .withProperties(Map.of("subtitle", "Slew Rate Limit"));
        
        // Slider to adjust slew rate limit
        GenericEntry sliderEntry = filterTab.add("Adjust Slew Rate", SLEW_RATE_LIMIT)
            .withPosition(1, 2)
            .withSize(2, 1)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0.01, "max", 0.2, "block increment", 0.01, "subtitle", "Adjust Slew Rate"))
            .getEntry();
            
        NetworkTableInstance.getDefault().addListener(
            sliderEntry.getTopic(),
            EnumSet.of(NetworkTableEvent.Kind.kValueAll),
            event -> {
                SLEW_RATE_LIMIT = event.valueData.value.getDouble();
            });
    }
    
    /**
     * Convert tracked objects to string for display
     */
    private String getTrackedObjectsAsString() {
        if (trackedObjects.isEmpty()) {
            return "No Objects";
        }
        
        return trackedObjects.values().stream()
            .filter(obj -> !obj.isStale())
            .map(obj -> String.format("ID:%d %s @ (%.2f, %.2f) Distance:%.2f",
                obj.id,
                obj.id == 0 ? "Algae" : "Coral",
                obj.pose.getX(),
                obj.pose.getY(),
                obj.distance))
            .collect(Collectors.joining("\n"));
    }
    
    /**
     * Set the drive subsystem
     * Must be set before using aim or follow functions
     */
    public void setDriveSubsystem(Drive drive) {
        this.driveSubsystem = drive;
    }
    
    @Override
    public void periodic() {
        // Get the latest PhotonVision result
        var result = camera.getLatestResult();
        
        // Update tracked objects
        if (result.hasTargets()) {
            updateTrackedObjects(result);
        }
        
        // Clear stale object tracking
        removeStaleObjects();
        
        // Update field visualization
        updateFieldWidget();
        
        // Check if user updated settings from Shuffleboard
        checkShuffleboardControls();
    }
    
    /**
     * Remove stale object tracking
     */
    private void removeStaleObjects() {
        // Use iterator to safely remove stale items
        Iterator<Map.Entry<Integer, TrackedObject>> it = trackedObjects.entrySet().iterator();
        while (it.hasNext()) {
            if (it.next().getValue().isStale()) {
                it.remove();
            }
        }
    }
    
    /**
     * Update field visualization
     */
    private void updateFieldWidget() {
        // Update robot position
        if (driveSubsystem != null) {
            fieldWidget.setRobotPose(driveSubsystem.getPose());
        }
        
        // Update object positions
        trackedObjects.forEach((id, obj) -> {
            if (!obj.isStale()) {
                String objectType = id == 0 ? "algae" : "coral";
                fieldWidget.getObject(objectType + "-" + id).setPose(obj.pose);
            }
        });
    }
    
    /**
     * Calculate object distance
     * @param target tracked target
     * @return distance (meters)
     */
    public double calculateDistance(PhotonTrackedTarget target) {

        double apparentWidth = 0;
    
        try {
            var corners = target.getMinAreaRectCorners();
            if (corners != null && corners.size() >= 4) {

                double diag1 = Math.hypot(
                    corners.get(0).x - corners.get(2).x,
                    corners.get(0).y - corners.get(2).y
                );
                double diag2 = Math.hypot(
                    corners.get(1).x - corners.get(3).x,
                    corners.get(1).y - corners.get(3).y
                );
                apparentWidth = Math.max(diag1, diag2) / Math.sqrt(2);
            }
        } catch (Exception e) {
            apparentWidth = 0;
        }
        
        if (apparentWidth <= 0) {
            // If width cannot be obtained, use area as fallback
            return estimateDistanceFromArea(target.getArea());
        }
        
        // Determine actual size based on object ID
        double actualWidth;
        if (target.getFiducialId() == 0) {
            actualWidth = ALGAE_DIAMETER; // Algae diameter
        } else {
            actualWidth = CORAL_LENGTH; // Coral length
        }
        
        // Use angular formula to calculate distance
        return (actualWidth * CAMERA_RESOLUTION_WIDTH) / 
               (2 * apparentWidth * Math.tan(Math.toRadians(CAMERA_HORIZONTAL_FOV/2)));
    }
    
    /**
     * Estimate distance from object area (primary method)
     * @param area object area (percentage 0-100)
     * @return estimated distance (meters)
     */
    private double estimateDistanceFromArea(double area) {
        // Choose appropriate calibration factor based on object class
        double scaleFactor;
        if (targetClass == 0) { // Algae
            scaleFactor = 0.35; // Calibrated through experiment
        } else { // Coral
            scaleFactor = 0.40; // Calibrated through experiment
        }
        
        // Area is percentage (0-100), larger area means object is closer
        if (area < 0.1) {
            // Avoid division by very small values
            return 10.0; // Maximum distance limit (10 meters)
        }
        
        // Experiments show distance is roughly proportional to inverse square root of area
        // Distance ≈ scaleFactor / √(Area)
        return scaleFactor / Math.sqrt(area / 100.0);
    }
    
    /**
     * Update tracked objects
     */
    private void updateTrackedObjects(PhotonPipelineResult result) {
        if (!result.hasTargets() || driveSubsystem == null) {
            return;
        }
        
        for (PhotonTrackedTarget target : result.getTargets()) {
            int objectId = target.getFiducialId();
            if (objectId != 0 && objectId != 1) continue; // Only process objects with ID 0 or 1
            
            // Calculate object distance
            double distance = calculateDistance(target);
            
            // Calculate position relative to robot
            double yaw = Math.toRadians(target.getYaw());
            
            // Relative position vector (forward is positive x-axis)
            Translation2d objectRelativePos = new Translation2d(
                distance * Math.cos(yaw),
                distance * Math.sin(yaw)
            );
            
            // Transform to field coordinate system
            Pose2d robotPose = driveSubsystem.getPose();
            double robotHeading = robotPose.getRotation().getRadians();
            
            Pose2d objectPose = new Pose2d(
                robotPose.getX() + objectRelativePos.getX() * Math.cos(robotHeading) - 
                            objectRelativePos.getY() * Math.sin(robotHeading),
                robotPose.getY() + objectRelativePos.getX() * Math.sin(robotHeading) + 
                            objectRelativePos.getY() * Math.cos(robotHeading),
                new Rotation2d(0)
            );
            
            // Update object position
            trackedObjects.put(objectId, new TrackedObject(objectId, objectPose, distance));
            
            // If this is the current target class, update to Shuffleboard
            if (objectId == targetClass) {
                distanceEstimateEntry.setDouble(distance);
            }
        }
    }
    
    /**
     * Check if user updated settings from Shuffleboard
     */
    private void checkShuffleboardControls() {
        // Read target class setting
        int newTargetClass = (int) targetClassEntry.getDouble(targetClass);
        if (newTargetClass != targetClass && (newTargetClass == 0 || newTargetClass == 1)) {
            targetClass = newTargetClass;
        }
        
        // Check button states
        boolean aimRequested = aimButtonEntry.getBoolean(false);
        boolean followRequested = followButtonEntry.getBoolean(false);
        boolean stopRequested = stopButtonEntry.getBoolean(false);
        
        // Handle stop request
        if (stopRequested) {
            stopAllCommands();
            stopButtonEntry.setBoolean(false);
        }
        // Handle aim request
        else if (aimRequested && !isAiming && !isFollowing) {
            startAiming();
            aimButtonEntry.setBoolean(false);
        }
        // Handle follow request
        else if (followRequested && !isFollowing) {
            startFollowing();
            followButtonEntry.setBoolean(false);
        }
    }
    
    /**
     * Start aiming at target
     */
    private void startAiming() {
        if (driveSubsystem == null) {
            return;
        }
        
        stopAllCommands();
        isAiming = true;
        activeCommand = new AimAtTargetCommand(driveSubsystem);
        CommandScheduler.getInstance().schedule(activeCommand);
    }
    
    /**
     * Start following target
     */
    private void startFollowing() {
        if (driveSubsystem == null) {
            return;
        }
        
        stopAllCommands();
        isFollowing = true;
        activeCommand = new FollowTargetCommand(driveSubsystem);
        CommandScheduler.getInstance().schedule(activeCommand);
    }
    
    /**
     * Stop all commands
     */
    private void stopAllCommands() {
        if (activeCommand != null) {
            activeCommand.cancel();
            activeCommand = null;
        }
        isAiming = false;
        isFollowing = false;
        
        if (driveSubsystem != null) {
            driveSubsystem.stop();
        }
    }
    
    /**
     * Get the best target from results based on current target class (0 or 1)
     */
    public PhotonTrackedTarget getBestTarget(PhotonPipelineResult result) {
        if (!result.hasTargets()) {
            return null;
        }
        
        List<PhotonTrackedTarget> targets = result.getTargets();
        
        // First, try to find a target matching the class
        for (PhotonTrackedTarget target : targets) {
            // Assume fiducialId field contains class (0 or 1)
            if (target.getFiducialId() == targetClass) {
                return target;
            }
        }
        
        return getClosestTarget(targets);
    }
    
    /**
     * Get the closest target from a list, based on area (larger = closer)
     */
    public PhotonTrackedTarget getClosestTarget(List<PhotonTrackedTarget> targets) {
        if (targets.isEmpty()) {
            return null;
        }
        
        PhotonTrackedTarget closestTarget = targets.get(0);
        double largestArea = closestTarget.getArea();
        
        for (PhotonTrackedTarget target : targets) {
            double area = target.getArea();
            if (area > largestArea) {
                closestTarget = target;
                largestArea = area;
            }
        }
        
        return closestTarget;
    }
    
    /**
     * Get filtered yaw value using convolution
     * @param target target
     * @return filtered yaw value (degrees)
     */
    private double getFilteredYaw(PhotonTrackedTarget target) {
        if (target == null) return 0.0;
        
        // Update history data
        updateHistory(yawHistory, target.getYaw());
        
        // Apply convolution filter
        double filteredValue = applyConvolution(yawHistory, currentYawKernel);
        
        // Update member variable for Shuffleboard display
        filteredYaw = filteredValue;
        
        return filteredValue;
    }
    
    /**
     * Get filtered area value using convolution
     * @param target target
     * @return filtered area value
     */
    private double getFilteredArea(PhotonTrackedTarget target) {
        if (target == null) return 0.0;
        
        // Update history data
        updateHistory(areaHistory, target.getArea());
        
        // Apply convolution filter
        double filteredValue = applyConvolution(areaHistory, currentAreaKernel);
        
        // Update member variable for Shuffleboard display
        filteredArea = filteredValue;
        
        return filteredValue;
    }
    
    /**
     * Apply rate limiting to prevent sudden output changes
     * @param newValue new value
     * @param lastValue previous value
     * @return limited value
     */
    private double applyRateLimit(double newValue, double lastValue) {
        double change = newValue - lastValue;
        
        // Limit rate of change
        if (change > SLEW_RATE_LIMIT) {
            change = SLEW_RATE_LIMIT;
        } else if (change < -SLEW_RATE_LIMIT) {
            change = -SLEW_RATE_LIMIT;
        }
        
        return lastValue + change;
    }
    
    /**
     * Set the class to track (0=algae, 1=coral)
     */
    public void setTargetClass(int classID) {
        if (classID == 0 || classID == 1) {
            this.targetClass = classID;
            targetClassEntry.setDouble(classID);
        }
    }
    
    /**
     * Calculate turning speed needed to aim at target
     * @return turning speed (positive=right, negative=left)
     */
    public double calculateAimOutput() {
        var result = camera.getLatestResult();
        if (!result.hasTargets()) {
            // If no target, gradually reduce turning output
            lastTurnOutput = applyRateLimit(0.0, lastTurnOutput);
            return lastTurnOutput;
        }
        
        var target = getBestTarget(result);
        if (target == null) {
            // If no target, gradually reduce turning output
            lastTurnOutput = applyRateLimit(0.0, lastTurnOutput);
            return lastTurnOutput;
        }
        
        // Use filtered yaw value instead of raw value
        double filteredYawValue = getFilteredYaw(target);
        
        // Calculate turning output based on filtered value
        double turnOutput = turnController.calculate(filteredYawValue, 0);
        
        // Apply rate limiting
        lastTurnOutput = applyRateLimit(turnOutput, lastTurnOutput);
        
        // Output debug info
        SmartDashboard.putNumber("Raw Yaw", target.getYaw());
        SmartDashboard.putNumber("Filtered Yaw", filteredYawValue);
        
        return lastTurnOutput;
    }
    
    /**
     * Calculate drive speed needed to approach target
     * @return drive speed (positive=forward, negative=backward)
     */
    public double calculateDriveOutput() {
        var result = camera.getLatestResult();
        if (!result.hasTargets()) {
            // If no target, gradually reduce drive output
            lastDriveOutput = applyRateLimit(0.0, lastDriveOutput);
            return lastDriveOutput;
        }
        
        var target = getBestTarget(result);
        if (target == null) {
            // If no target, gradually reduce drive output
            lastDriveOutput = applyRateLimit(0.0, lastDriveOutput);
            return lastDriveOutput;
        }
        
        // Use filtered area instead of raw area
        double filteredAreaValue = getFilteredArea(target);
        double targetArea = targetAreaEntry.getDouble(TARGET_AREA_SETPOINT);
        
        // Calculate drive output based on filtered value
        double driveOutput = -driveController.calculate(filteredAreaValue, targetArea);
        
        // Apply rate limiting
        lastDriveOutput = applyRateLimit(driveOutput, lastDriveOutput);
        
        // Output debug info
        SmartDashboard.putNumber("Raw Area", target.getArea());
        SmartDashboard.putNumber("Filtered Area", filteredAreaValue);
        
        return lastDriveOutput;
    }
    
    /**
     * Check if a target of the specified class is visible
     */
    public boolean isTargetVisible() {
        var result = camera.getLatestResult();
        return result.hasTargets() && getBestTarget(result) != null;
    }
    
    /**
     * Check if we're aimed at the target
     */
    public boolean isAimedAtTarget() {
        var result = camera.getLatestResult();
        if (!result.hasTargets()) {
            return false;
        }
        
        var target = getBestTarget(result);
        if (target == null) {
            return false;
        }
        
        double tolerance = aimToleranceEntry.getDouble(AIM_TOLERANCE_DEGREES);
        // Use filtered yaw to determine if aimed
        return Math.abs(filteredYaw) < tolerance;
    }
    
    /**
     * Get the camera used for object detection
     */
    public PhotonCamera getCamera() {
        return camera;
    }
    
    /**
     * Get the current target class (0=algae, 1=coral)
     */
    public int getTargetClass() {
        return targetClass;
    }
    
    /**
     * Get estimated distance to target object (meters)
     * @return distance, or -1 if no visible target
     */
    public double getTargetDistance() {
        var result = camera.getLatestResult();
        if (!result.hasTargets()) {
            return -1.0;
        }
        
        var target = getBestTarget(result);
        if (target == null) {
            return -1.0;
        }
        
        return calculateDistance(target);
    }
    
    /**
     * Get position of a specific class of object (if visible)
     * @param classId object class ID (0=algae, 1=coral)
     * @return optional object position
     */
    public Optional<Pose2d> getObjectPosition(int classId) {
        TrackedObject obj = trackedObjects.get(classId);
        if (obj != null && !obj.isStale()) {
            return Optional.of(obj.pose);
        }
        return Optional.empty();
    }
    
    /**
     * Get the nearest object
     * @return optional object ID and position
     */
    public Optional<TrackedObject> getNearestObject() {
        if (trackedObjects.isEmpty() || driveSubsystem == null) {
            return Optional.empty();
        }
        
        Pose2d robotPose = driveSubsystem.getPose();
        return trackedObjects.values().stream()
            .filter(obj -> !obj.isStale())
            .min(Comparator.comparingDouble(obj -> 
                robotPose.getTranslation().getDistance(obj.pose.getTranslation())));
    }
    
    // ===================== Command Classes =====================
    
    /**
     * Command to aim at target
     * This command will rotate the robot until it directly faces the target
     */
    private class AimAtTargetCommand extends Command {
        private final Drive driveSubsystem;
        
        /**
         * Create a new AimAtTargetCommand
         * 
         * @param driveSubsystem drive subsystem to control robot movement
         */
        public AimAtTargetCommand(Drive driveSubsystem) {
            this.driveSubsystem = driveSubsystem;
            
            // This command requires these two subsystems
            addRequirements(ObjectDetection.this, driveSubsystem);
        }
        
        @Override
        public void execute() {
            // Get turn value needed to aim at visual target
            double turnOutput = calculateAimOutput();
            
            // Apply rotation to drive system (no forward/backward motion)
            driveSubsystem.runVelocity(new ChassisSpeeds(0, 0, turnOutput));
            
            // Debug output
            SmartDashboard.putNumber("Aim Turn Output", turnOutput);
        }
        
        @Override
        public boolean isFinished() {
            // Command completes when we're aimed at target
            return isAimedAtTarget();
        }
        
        @Override
        public void end(boolean interrupted) {
            // Stop drive system
            driveSubsystem.stop();
            isAiming = false;
        }
    }
    
    /**
     * Command to follow target
     * This command both aims at the target and maintains a specific distance from it
     */
    private class FollowTargetCommand extends Command {
        private final Drive driveSubsystem;
        
        /**
         * Create a new FollowTargetCommand
         * 
         * @param driveSubsystem drive subsystem for robot movement
         */
        public FollowTargetCommand(Drive driveSubsystem) {
            this.driveSubsystem = driveSubsystem;
            
            // This command requires these subsystems
            addRequirements(ObjectDetection.this, driveSubsystem);
        }
        
        @Override
        public void execute() {
            if (!isTargetVisible()) {
                driveSubsystem.stop();
                return;
            }
            
            // Calculate aiming output
            double turnOutput = calculateAimOutput();
            
            // Calculate drive output
            double driveOutput = calculateDriveOutput();
            
            // Apply to drive system
            driveSubsystem.runVelocity(new ChassisSpeeds(driveOutput, 0, turnOutput));
            
            // Debug output
            SmartDashboard.putNumber("Follow Drive Output", driveOutput);
            SmartDashboard.putNumber("Follow Turn Output", turnOutput);
        }
        
        @Override
        public boolean isFinished() {
            return false;
        }
        
        @Override
        public void end(boolean interrupted) {
            driveSubsystem.stop();
            isFollowing = false;
        }
    }
}