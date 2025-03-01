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
import edu.wpi.first.networktables.GenericEntry;
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

/**
 * Object Detection Subsystem
 * This subsystem uses PhotonVision to detect objects classified as "0" (algae) or "1" (coral)
 * Provides the following functionality:
 * - Control tracking features through Shuffleboard
 * - Track and follow detected objects
 * - Automatically aim at objects
 * - Track the closest object
 * - Estimate object distance
 * - Calculate object positions on the field
 */
public class ObjectDetection extends SubsystemBase {
    private final PhotonCamera camera;
    private int targetClass = 0; // Default tracking class 0 (algae)
    private final PIDController turnController;
    private final PIDController driveController;
    private Drive driveSubsystem;
    
    // Game object size definitions (units: meters)
    private static final double ALGAE_DIAMETER = 0.413; // algae diameter
    private static final double CORAL_LENGTH = 0.30;   // coral length
    
    // Camera settings
    private static final double CAMERA_HORIZONTAL_FOV = 70.0; // Camera horizontal field of view
    private static final double CAMERA_RESOLUTION_WIDTH = 640.0; // Camera resolution width
    
    // Command state
    private boolean isAiming = false;
    private boolean isFollowing = false;
    private Command activeCommand = null;
    
    // Object tracking system
    private Map<Integer, TrackedObject> trackedObjects = new HashMap<>();
    private Field2d fieldWidget = new Field2d();
    
    // Shuffleboard items
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
    
    // 濾波相關的變數
    private double filteredYaw = 0.0;
    private double filteredArea = 0.0;
    private double lastTurnOutput = 0.0;
    private double lastDriveOutput = 0.0;
    
    // 濾波常數 (可根據需要調整)
    private static final double YAW_FILTER_CLOSE = 0.85; // 近距離時強度較高的濾波 (越高 = 濾波越強)
    private static final double YAW_FILTER_FAR = 0.5;    // 遠距離時強度較低的濾波
    private static final double AREA_FILTER = 0.7;       // 面積濾波強度
    private static final double SLEW_RATE_LIMIT = 0.05;  // 每次循環輸出變化的限制
    private static final double CLOSE_DISTANCE = 1.0;    // 多少米被視為"靠近"
    
    /**
     * Information for tracked objects
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
            // If not updated for more than 3 seconds, consider stale
            return Timer.getFPGATimestamp() - lastUpdated > 3.0;
        }
    }
    
    public ObjectDetection() {
        camera = new PhotonCamera("WEB_CAM");
        turnController = new PIDController(0.007, 0, 0.0001); // Turning PID
        driveController = new PIDController(0.05, 0, 0.0001); // Approach PID
        setupShuffleboardControls();
    }
    
    /**
     * Set up Shuffleboard controls
     */
    private void setupShuffleboardControls() {
        targetClassEntry = visionTab.add("Target (0=algae, 1=coral)", targetClass)
            .withPosition(0, 0)
            .withSize(1, 1)
            .getEntry();
        
        targetAreaEntry = visionTab.add("Target Area Size", TARGET_AREA_SETPOINT)
            .withPosition(1, 0)
            .withSize(1, 1)
            .getEntry();
        
        aimToleranceEntry = visionTab.add("Aim Tolerance (deg)", AIM_TOLERANCE_DEGREES)
            .withPosition(2, 0)
            .withSize(1, 1)
            .getEntry();
        
        aimButtonEntry = visionTab.add("Aim at Target", false)
            .withPosition(0, 1)
            .withSize(1, 1)
            .withProperties(Map.of("colorWhenTrue", "green"))
            .getEntry();
        
        followButtonEntry = visionTab.add("Follow Target", false)
            .withPosition(1, 1)
            .withSize(1, 1)
            .withProperties(Map.of("colorWhenTrue", "blue"))
            .getEntry();
        
        stopButtonEntry = visionTab.add("Stop", false)
            .withPosition(2, 1)
            .withSize(1, 1)
            .withProperties(Map.of("colorWhenTrue", "red"))
            .getEntry();
        
        distanceEstimateEntry = visionTab.add("Estimated Distance (m)", 0.0)
            .withPosition(3, 0)
            .withSize(1, 1)
            .getEntry();
        
        visionTab.addBoolean("Target Visible", this::isTargetVisible)
            .withPosition(0, 2)
            .withSize(1, 1);
        
        visionTab.addBoolean("Aimed", this::isAimedAtTarget)
            .withPosition(1, 2)
            .withSize(1, 1);
        
        visionTab.addString("Status", () -> {
            if (isFollowing) return "Following";
            if (isAiming) return "Aiming";
            return "Idle";
        })
            .withPosition(2, 2)
            .withSize(1, 1);
            
        visionTab.addString("Target Class", () -> 
            targetClass == 0 ? "algae" : "coral")
            .withPosition(3, 1)
            .withSize(1, 1);
            
        // Add field visualization
        visionTab.add("Field", fieldWidget)
            .withSize(5, 3)
            .withPosition(0, 3);
            
        // Display tracked objects list
        visionTab.addString("Tracked Objects", this::getTrackedObjectsAsString)
            .withSize(2, 3)
            .withPosition(5, 3);
            
        // Add debug values for troubleshooting
        visionTab.addNumber("Target Yaw", () -> {
            var result = camera.getLatestResult();
            if (result.hasTargets()) {
                var target = getBestTarget(result);
                return target != null ? target.getYaw() : 0.0;
            }
            return 0.0;
        })
            .withPosition(3, 2)
            .withSize(1, 1);
            
        visionTab.addNumber("PID Turn Error", () -> {
            var result = camera.getLatestResult();
            if (result.hasTargets()) {
                var target = getBestTarget(result);
                return target != null ? turnController.getPositionError() : 0.0;
            }
            return 0.0;
        })
            .withPosition(4, 2)
            .withSize(1, 1);
            
        // 增加顯示濾波後的數值
        visionTab.addNumber("Filtered Yaw", () -> filteredYaw)
            .withPosition(4, 0)
            .withSize(1, 1);
            
        visionTab.addNumber("Filtered Area", () -> filteredArea)
            .withPosition(4, 1)
            .withSize(1, 1);
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
                obj.id == 0 ? "algae" : "coral",
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
     * Clear stale object tracking
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
     * @param target Tracking target
     * @return Distance (meters)
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
            // If width cannot be obtained, use area as a backup method
            return estimateDistanceFromArea(target.getArea());
        }
        
        // Determine actual size based on object ID
        double actualWidth;
        if (target.getFiducialId() == 0) {
            actualWidth = ALGAE_DIAMETER; // algae diameter
        } else {
            actualWidth = CORAL_LENGTH; // coral length
        }
        
        // Use angle formula to calculate distance
        return (actualWidth * CAMERA_RESOLUTION_WIDTH) / 
               (2 * apparentWidth * Math.tan(Math.toRadians(CAMERA_HORIZONTAL_FOV/2)));
    }
    
    /**
     * Estimate distance from object area (main method)
     * @param area Object area (percentage 0-100)
     * @return Estimated distance (meters)
     */
    private double estimateDistanceFromArea(double area) {
        // Choose appropriate calibration factor based on object class
        double scaleFactor;
        if (targetClass == 0) { // algae
            scaleFactor = 0.35; // To be calibrated through experiments
        } else { // coral
            scaleFactor = 0.40; // To be calibrated through experiments
        }
        
        // Area is percentage (0-100), larger area means object is closer
        if (area < 0.1) {
            // Avoid dividing by extremely small values
            return 10.0; // Maximum distance limit (10 meters)
        }
        
        // Experiments show that distance is roughly proportional to the reciprocal of the square root of area
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
            
            // If it's the current target class, update to Shuffleboard
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
        
        // First, try to find targets matching the class
        for (PhotonTrackedTarget target : targets) {
            // Assume fiducialId field contains class (0 or 1)
            if (target.getFiducialId() == targetClass) {
                return target;
            }
        }
        
        return getClosestTarget(targets);
    }
    
    /**
     * Get the closest target from the target list based on area (larger = closer)
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
     * 獲取經過濾波的偏航值
     * @param target 目標
     * @return 經過濾波的偏航值 (角度)
     */
    private double getFilteredYaw(PhotonTrackedTarget target) {
        if (target == null) return 0.0;
        
        // 根據距離計算濾波因子
        double distance = calculateDistance(target);
        double filterFactor = YAW_FILTER_FAR;
        
        // 當靠近目標時應用更強的濾波
        if (distance < CLOSE_DISTANCE) {
            // 線性增加濾波強度（距離越近，濾波越強）
            double t = Math.max(0, distance / CLOSE_DISTANCE);
            filterFactor = YAW_FILTER_CLOSE * (1 - t) + YAW_FILTER_FAR * t;
        }
        
        // 應用低通濾波
        filteredYaw = filterFactor * filteredYaw + (1 - filterFactor) * target.getYaw();
        return filteredYaw;
    }
    
    /**
     * 獲取經過濾波的面積值
     * @param target 目標
     * @return 經過濾波的面積值
     */
    private double getFilteredArea(PhotonTrackedTarget target) {
        if (target == null) return 0.0;
        
        // 應用低通濾波
        filteredArea = AREA_FILTER * filteredArea + (1 - AREA_FILTER) * target.getArea();
        return filteredArea;
    }
    
    /**
     * 應用變化率限制，防止輸出突然變化
     * @param newValue 新值
     * @param lastValue 上一個值
     * @return 限制後的值
     */
    private double applyRateLimit(double newValue, double lastValue) {
        double change = newValue - lastValue;
        
        // 限制變化率
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
     * Calculate the turning speed needed to aim at the target
     * @return Turning speed (positive = turn right, negative = turn left)
     */
    public double calculateAimOutput() {
        var result = camera.getLatestResult();
        if (!result.hasTargets()) {
            // 如果沒有目標，逐漸減少轉向輸出
            lastTurnOutput = applyRateLimit(0.0, lastTurnOutput);
            return lastTurnOutput;
        }
        
        var target = getBestTarget(result);
        if (target == null) {
            // 如果沒有目標，逐漸減少轉向輸出
            lastTurnOutput = applyRateLimit(0.0, lastTurnOutput);
            return lastTurnOutput;
        }
        
        // 使用經過濾波的偏航值而不是原始值
        double filteredYawValue = getFilteredYaw(target);
        
        // 根據濾波後的值計算轉向輸出
        double turnOutput = turnController.calculate(filteredYawValue, 0);
        
        // 應用變化率限制
        lastTurnOutput = applyRateLimit(turnOutput, lastTurnOutput);
        
        // 輸出調試信息
        SmartDashboard.putNumber("Raw Yaw", target.getYaw());
        SmartDashboard.putNumber("Filtered Yaw", filteredYawValue);
        
        return lastTurnOutput;
    }
    
    /**
     * Calculate driving speed needed to approach target
     * @return Drive speed (positive = forward, negative = backward)
     */
    public double calculateDriveOutput() {
        var result = camera.getLatestResult();
        if (!result.hasTargets()) {
            // 如果沒有目標，逐漸減少驅動輸出
            lastDriveOutput = applyRateLimit(0.0, lastDriveOutput);
            return lastDriveOutput;
        }
        
        var target = getBestTarget(result);
        if (target == null) {
            // 如果沒有目標，逐漸減少驅動輸出
            lastDriveOutput = applyRateLimit(0.0, lastDriveOutput);
            return lastDriveOutput;
        }
        
        // 使用經過濾波的面積而不是原始面積
        double filteredAreaValue = getFilteredArea(target);
        double targetArea = targetAreaEntry.getDouble(TARGET_AREA_SETPOINT);
        
        // 根據濾波後的值計算驅動輸出
        double driveOutput = -driveController.calculate(filteredAreaValue, targetArea);
        
        // 應用變化率限制
        lastDriveOutput = applyRateLimit(driveOutput, lastDriveOutput);
        
        // 輸出調試信息
        SmartDashboard.putNumber("Raw Area", target.getArea());
        SmartDashboard.putNumber("Filtered Area", filteredAreaValue);
        
        return lastDriveOutput;
    }
    
    /**
     * Check if a target with the specified class is visible
     */
    public boolean isTargetVisible() {
        var result = camera.getLatestResult();
        return result.hasTargets() && getBestTarget(result) != null;
    }
    
    /**
     * Check if we are aimed at the target
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
        // 使用濾波後的偏航值來決定是否已對準
        return Math.abs(filteredYaw) < tolerance;
    }
    
    /**
     * Get the camera used for object detection
     */
    public PhotonCamera getCamera() {
        return camera;
    }
    
    /**
     * Get current target class (0=algae, 1=coral)
     */
    public int getTargetClass() {
        return targetClass;
    }
    
    /**
     * Get the estimated distance to the target object (meters)
     * @return Distance, or -1 if no target visible
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
     * Get object position for a specific class (if visible)
     * @param classId Object class ID (0=algae, 1=coral)
     * @return Optional with object position
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
     * @return Optional with object ID and position
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
         * @param driveSubsystem Drive subsystem that controls robot movement
         */
        public AimAtTargetCommand(Drive driveSubsystem) {
            this.driveSubsystem = driveSubsystem;
            
            // This command requires these two subsystems
            addRequirements(ObjectDetection.this, driveSubsystem);
        }
        
        @Override
        public void execute() {
            // Get turning value needed to aim at vision target
            double turnOutput = calculateAimOutput();
            
            // Apply rotation to drive system (no forward/backward motion)
            driveSubsystem.runVelocity(new ChassisSpeeds(0, 0, turnOutput));
            
            // Debug output
            SmartDashboard.putNumber("Aim Turn Output", turnOutput);
        }
        
        @Override
        public boolean isFinished() {
            // Command completes when we are aimed at the target
            return isAimedAtTarget();
        }
        
        @Override
        public void end(boolean interrupted) {
            // Stop the drive system
            driveSubsystem.stop();
            isAiming = false;
        }
    }
    
    /**
     * Command to follow target
     * This command both aims at the target and maintains a specified distance from it
     */
    private class FollowTargetCommand extends Command {
        private final Drive driveSubsystem;
        
        /**
         * Create a new FollowTargetCommand
         * 
         * @param driveSubsystem Drive subsystem for robot movement
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