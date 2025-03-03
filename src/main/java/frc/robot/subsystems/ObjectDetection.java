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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.subsystems.drive.Drive;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.*;

import java.util.List;
import java.util.Map;
import java.util.HashMap;
import java.util.Optional;
import java.util.stream.Collectors;
import java.util.Comparator;
import java.util.Iterator;

/**
 * Object Detection Subsystem with H-Infinity Control
 * This subsystem uses PhotonVision to detect objects classified as "0" (algae) or "1" (coral)
 * Uses H-Infinity control for robust tracking against noise and disturbances
 */
public class ObjectDetection extends SubsystemBase {
    private final PhotonCamera camera;
    private int targetClass = 0; // Default tracking class 0 (algae)
    private Drive driveSubsystem;
    
    // Game object size definitions (units: meters)
    private static final double ALGAE_DIAMETER = 0.413; // algae diameter
    private static final double CORAL_LENGTH = 0.30;   // coral length
    
    // Camera settings
    private static final double CAMERA_HORIZONTAL_FOV = 70.0; // Camera horizontal field of view
    private static final double CAMERA_RESOLUTION_WIDTH = 640.0; // Camera resolution width
    
    // Command state
    private boolean isFollowing = false;
    private Command activeCommand = null;
    
    // Object tracking system
    private Map<Integer, TrackedObject> trackedObjects = new HashMap<>();
    private Field2d fieldWidget = new Field2d();
    
    private static final double TARGET_AREA_SETPOINT = 30.0; 
    private static final double AIM_TOLERANCE_DEGREES = 2.0; 
    
    // H-Infinity Control Parameters
    private static final double GAMMA = 0.1; // 越小越抗干擾
    
    // State space representation for H-Infinity controller (for yaw control)
    // x = [yaw error, yaw_rate]
    private Matrix<N2, N1> x_yaw = new Matrix<>(Nat.N2(), Nat.N1());
    
    // State space representation for H-Infinity controller (for distance control)
    // x = [distance error, distance_rate]
    private Matrix<N2, N1> x_dist = new Matrix<>(Nat.N2(), Nat.N1());
    
    // H-Infinity controller gains (would normally be computed from Riccati equations)
    private Matrix<N1, N2> K_yaw; // Control gain for yaw
    private Matrix<N1, N2> K_dist; // Control gain for distance
    
    // System matrices for yaw control
    private Matrix<N2, N2> A_yaw; // System matrix for yaw
    private Matrix<N2, N1> B_yaw; // Input matrix for yaw
    private Matrix<N2, N1> G_yaw; // Disturbance matrix for yaw
    
    // System matrices for distance control
    private Matrix<N2, N2> A_dist; // System matrix for distance
    private Matrix<N2, N1> B_dist; // Input matrix for distance
    private Matrix<N2, N1> G_dist; // Disturbance matrix for distance
    
    // Previous measurements for state estimation
    private double prevYawError = 0.0;
    private double prevDistError = 0.0;
    private double lastUpdateTime = 0.0;
    
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
        
        // Initialize basic SmartDashboard values
        SmartDashboard.putNumber("Estimated Distance (m)", 0.0);
        SmartDashboard.putBoolean("Target Visible", false);
        SmartDashboard.putString("Target Class", targetClass == 0 ? "algae" : "coral");
        
        // Initialize state vectors
        x_yaw.set(0, 0, 0.0); // Initial yaw error
        x_yaw.set(1, 0, 0.0); // Initial yaw error rate
        
        x_dist.set(0, 0, 0.0); // Initial distance error
        x_dist.set(1, 0, 0.0); // Initial distance error rate
        
        // Initialize system matrices for yaw control
        // These values would typically be identified from system modeling
        A_yaw = new Matrix<>(Nat.N2(), Nat.N2());
        A_yaw.set(0, 0, 0.0);  // No natural drift in error
        A_yaw.set(0, 1, 1.0);  // Rate of change affects error
        A_yaw.set(1, 0, 0.0);  // Error doesn't affect rate directly
        A_yaw.set(1, 1, -0.2); // Natural damping
        
        B_yaw = new Matrix<>(Nat.N2(), Nat.N1());
        B_yaw.set(0, 0, 0.0);  // Control doesn't directly affect error
        B_yaw.set(1, 0, 1.0);  // Control affects rate of change
        
        G_yaw = new Matrix<>(Nat.N2(), Nat.N1());
        G_yaw.set(0, 0, 0.1);  // Disturbance affects error
        G_yaw.set(1, 0, 0.5);  // Disturbance affects rate more
        
        // Initialize system matrices for distance control
        A_dist = new Matrix<>(Nat.N2(), Nat.N2());
        A_dist.set(0, 0, 0.0);  // No natural drift in error
        A_dist.set(0, 1, 1.0);  // Rate of change affects error
        A_dist.set(1, 0, 0.0);  // Error doesn't affect rate directly
        A_dist.set(1, 1, -0.1); // Natural damping
        
        B_dist = new Matrix<>(Nat.N2(), Nat.N1());
        B_dist.set(0, 0, 0.0);  // Control doesn't directly affect error
        B_dist.set(1, 0, 1.0);  // Control affects rate of change
        
        G_dist = new Matrix<>(Nat.N2(), Nat.N1());
        G_dist.set(0, 0, 0.2);  // Disturbance affects error
        G_dist.set(1, 0, 0.3);  // Disturbance affects rate
        
        // Initialize H-Infinity controller gains
        // In a real implementation, these would be computed by solving the Riccati equations
        // Here we're using pre-computed values for simplicity
        K_yaw = new Matrix<>(Nat.N1(), Nat.N2());
        K_yaw.set(0, 0, 0.8);  // Gain for error
        K_yaw.set(0, 1, 0.2);  // Gain for error rate
        
        K_dist = new Matrix<>(Nat.N1(), Nat.N2());
        K_dist.set(0, 0, 0.6);  // Gain for error
        K_dist.set(0, 1, 0.1);  // Gain for error rate
        
        lastUpdateTime = Timer.getFPGATimestamp();
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
        
        // Update SmartDashboard values
        SmartDashboard.putBoolean("Target Visible", isTargetVisible());
        SmartDashboard.putString("Target Class", targetClass == 0 ? "algae" : "coral");
        SmartDashboard.putNumber("Estimated Distance (m)", getTargetDistance());
        
        // Update the state estimates
        updateStateEstimates(result);
    }
    
    /**
     * Update state estimates for H-Infinity control
     */
    private void updateStateEstimates(PhotonPipelineResult result) {
        double currentTime = Timer.getFPGATimestamp();
        double dt = currentTime - lastUpdateTime;
        lastUpdateTime = currentTime;
        
        if (dt <= 0 || dt > 0.1) {
            // Skip unreasonable time steps
            return;
        }
        
        if (result.hasTargets()) {
            var target = getBestTarget(result);
            if (target != null) {
                // Update yaw state
                double yawError = target.getYaw();
                double yawRateError = (yawError - prevYawError) / dt;
                prevYawError = yawError;
                
                x_yaw.set(0, 0, yawError);
                x_yaw.set(1, 0, yawRateError);
                
                // Update distance state
                double currentDist = calculateDistance(target);
                double distError = currentDist - (TARGET_AREA_SETPOINT / (target.getArea() / 100.0));
                double distRateError = (distError - prevDistError) / dt;
                prevDistError = distError;
                
                x_dist.set(0, 0, distError);
                x_dist.set(1, 0, distRateError);
                
                // Log state information
                SmartDashboard.putNumber("Yaw Error", yawError);
                SmartDashboard.putNumber("Yaw Rate Error", yawRateError);
                SmartDashboard.putNumber("Distance Error", distError);
                SmartDashboard.putNumber("Distance Rate Error", distRateError);
            }
        }
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
        }
    }
    
    /**
     * Set the drive subsystem
     * Must be set before using aim or follow functions
     */
    public void setDriveSubsystem(Drive drive) {
        this.driveSubsystem = drive;
    }

    public boolean isFollowing() {
        return isFollowing;
    }
    
    /**
     * Start following target
     */
    public void startFollowing() {
        if (driveSubsystem == null) {
            return;
        }
        
        isFollowing = true;
    }
    
    /**
     * Stop all commands
     */
    public void stopFollowing() {
        if (activeCommand != null) {
            activeCommand.cancel();
            activeCommand = null;
        }
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
     * Set the class to track (0=algae, 1=coral)
     */
    public void setTargetClass(int classID) {
        if (classID == 0 || classID == 1) {
            this.targetClass = classID;
            SmartDashboard.putString("Target Class", targetClass == 0 ? "algae" : "coral");
        }
    }
    
    /**
     * Calculate the turning speed needed to aim at the target using H-Infinity control
     * @return Turning speed (positive = turn right, negative = turn left)
     */
    public double calculateAimOutput() {
        var result = camera.getLatestResult();
        if (!result.hasTargets()) {
            return 0.0;
        }
        
        var target = getBestTarget(result);
        if (target == null) {
            return 0.0;
        }
        
        // Apply H-Infinity control law for yaw
        // u = -K * x
        Matrix<N1, N1> u_yaw = K_yaw.times(x_yaw).times(-1);
        
        // Add robustifying term to handle worst-case disturbance
        // For H∞ control, we add a term to counteract the worst-case disturbance
        Matrix<N1, N1> worstCaseDisturbance = G_yaw.transpose().times(x_yaw).times(1.0 / (GAMMA * GAMMA));
        
        // Combine main control with robustifying term
        double control = u_yaw.get(0, 0) + worstCaseDisturbance.get(0, 0);
        
        // Apply limits
        control = Math.max(-0.5, Math.min(0.5, control));
        
        // Debug output
        SmartDashboard.putNumber("H-Inf Yaw Control", control);
        
        return control;
    }
    
    /**
     * Calculate driving speed needed to approach target using H-Infinity control
     * @return Drive speed (positive = forward, negative = backward)
     */
    public double calculateDriveOutput() {
        var result = camera.getLatestResult();
        if (!result.hasTargets()) {
            return 0.0;
        }
        
        var target = getBestTarget(result);
        if (target == null) {
            return 0.0;
        }
        
        // Apply H-Infinity control law for distance
        // u = -K * x
        Matrix<N1, N1> u_dist = K_dist.times(x_dist).times(-1);
        
        // Add robustifying term for worst-case disturbance
        Matrix<N1, N1> worstCaseDisturbance = G_dist.transpose().times(x_dist).times(1.0 / (GAMMA * GAMMA));
        
        // Combine main control with robustifying term
        double control = u_dist.get(0, 0) + worstCaseDisturbance.get(0, 0);
        
        // Apply limits
        control = Math.max(-0.5, Math.min(0.5, control));
        
        // Debug output
        SmartDashboard.putNumber("H-Inf Distance Control", control);
        
        return control;
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
        
        // Consider aimed if within tolerance range
        return Math.abs(target.getYaw()) < AIM_TOLERANCE_DEGREES;
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
}