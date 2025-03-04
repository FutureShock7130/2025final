package frc.robot;

import static frc.robot.Constants.Vision.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.EnumSet;
import java.util.HashMap;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import java.util.Queue;
import java.util.LinkedList;

public class Vision extends SubsystemBase {
    private final Map<String, PhotonCamera> cameras;
    private final Map<String, PhotonPoseEstimator> photonEstimators;
    private String currentCamera;
    private Matrix<N3, N1> curStdDevs;
    
    // Kawman filter variables UwU
    private static final double MINIMUM_AMBIGUITY = 0.5;  // Lower is better for confidence
    private KalmanFilter<N3, N3, N3> poseFilter;
    private Pose2d lastFilteredPose = new Pose2d();
    private double lastTimestamp = 0;
    private boolean isFilterInitialized = false;

    // Simulation
    private PhotonCameraSim cameraSim;
    private VisionSystemSim visionSim;

    private SendableChooser<String> cameraChooser;

    private boolean processingEnabled = true;
    private boolean hasTarget = false;

    // H-infinity inspired wobust filtering parameters
    private final double GAMMA = 0.5;  // Robustness parameter (smaller = more robust, larger = more optimal)
    private final double MAX_INNOVATION_THRESHOLD = 0.15;  // Lower value = less trust in big jumps
    private final double MIN_TRUST_FACTOR = 0.3;  // Minimum trust in measurements (0-1)
    private Matrix<N3, N3> adaptiveR; // Adaptive measurement noise covariance
    private Queue<Matrix<N3, N1>> innovationHistory = new LinkedList<>();
    private final int INNOVATION_HISTORY_SIZE = 10;

    public Vision() {
        cameras = new HashMap<>();
        photonEstimators = new HashMap<>();
        
        // Initialize Kawman filter with state space model (x, y, theta)
        // State transition matrix (A) - how state evolves without control or measurement
        var A = MatBuilder.fill(Nat.N3(), Nat.N3(), 
                1, 0, 0, 
                0, 1, 0, 
                0, 0, 1);
        
        // Control input matrix (B) - we're not using control input
        var B = MatBuilder.fill(Nat.N3(), Nat.N3(), 
                0, 0, 0, 
                0, 0, 0, 
                0, 0, 0);
        
        // Measurement matrix (C) - identity because we measure state directly
        var C = MatBuilder.fill(Nat.N3(), Nat.N3(), 
                1, 0, 0, 
                0, 1, 0, 
                0, 0, 1);
                
        // Process noise (Q) - uncertainty in the model
        var Q = MatBuilder.fill(Nat.N3(), Nat.N3(), 
                0.01, 0, 0, 
                0, 0.01, 0, 
                0, 0, 0.01);
                
        // Measurement noise (R) - uncertainty in measurements
        var R = MatBuilder.fill(Nat.N3(), Nat.N3(), 
                0.1, 0, 0, 
                0, 0.1, 0, 
                0, 0, 0.1);
                
        // Initial state estimate
        var xHat = VecBuilder.fill(0, 0, 0);
                
        // Initial error covariance
        var P0 = MatBuilder.fill(Nat.N3(), Nat.N3(), 
                1, 0, 0, 
                0, 1, 0, 
                0, 0, 1);
                
        // Create the LinearSystem first
        var plant = new LinearSystem<N3, N3, N3>(A, B, C, MatBuilder.fill(Nat.N3(), Nat.N3(), 0, 0, 0, 0, 0, 0, 0, 0, 0));
        
        // Create the Kalman filter with the correct constructor signature
        poseFilter = new KalmanFilter<N3, N3, N3>(
            Nat.N3(),
            Nat.N3(),
            plant,
            VecBuilder.fill(0.01, 0.01, 0.01), // Process noise stds
            VecBuilder.fill(0.1, 0.1, 0.1),    // Measurement noise stds
            0.02                               // Nominal dt
        );
        poseFilter.setXhat(xHat);

        // Define transforms for each camera
        Map<String, Transform3d> robotToCamTransforms = new HashMap<>();
        robotToCamTransforms.put("CAM_1", new Transform3d(
                new Translation3d(0.331964, -0.209447, 0.223367), // right camera
                new Rotation3d(0.0, Math.toRadians(0), 0.0)));

        robotToCamTransforms.put("CAM_2", new Transform3d(
                new Translation3d(0.331964, 0.209447, 0.223367), // left camera
                new Rotation3d(0.0, Math.toRadians(0), Math.toRadians(0))));

        robotToCamTransforms.put("CAM_3", new Transform3d(
                new Translation3d(-0.331964, -0.209447, 0.223367), // top camera
                new Rotation3d(0.0, Math.toRadians(0), Math.toRadians(180))));

        // Create cameras and estimators with their specific transforms
        for (Map.Entry<String, Transform3d> entry : robotToCamTransforms.entrySet()) {
            String name = entry.getKey();
            Transform3d transform = entry.getValue();

            cameras.put(name, new PhotonCamera(name));
            photonEstimators.put(name,
                    new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                            transform));
        }

        currentCamera = "CAM_1"; // Start with first camera

        // Add camera selector to Shuffleboard
        ShuffleboardTab visionTab = Shuffleboard.getTab("Vision");
        cameraChooser = new SendableChooser<>();
        for (String name : cameras.keySet()) {
            cameraChooser.addOption(name, name);
        }
        visionTab.add("Camera Selection", cameraChooser)
                .withSize(2, 1)
                .withPosition(0, 0);

        // // Listen for camera selection changes
        NetworkTableInstance.getDefault()
                .getTable("SmartDashboard")
                .getSubTable("Camera Selection")
                .addListener(
                        EnumSet.of(NetworkTableEvent.Kind.kValueAll),
                        (table, key, event) -> currentCamera = cameraChooser.getSelected());

        // ----- Simulation
        if (Robot.isSimulation()) {
            // Create the vision system simulation which handles cameras and targets on the
            // field.
            visionSim = new VisionSystemSim("main");
            // Add all the AprilTags inside the tag layout as visible targets to this
            // simulated field.
            visionSim.addAprilTags(kTagLayout);
            // Create simulated camera properties. These can be set to mimic your actual
            // camera.
            var cameraProp = new SimCameraProperties();
            cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
            cameraProp.setCalibError(0.35, 0.10);
            cameraProp.setFPS(15);
            cameraProp.setAvgLatencyMs(50);
            cameraProp.setLatencyStdDevMs(15);
            // Create a PhotonCameraSim which will update the linked PhotonCamera's values
            // with visible
            // targets.
            cameraSim = new PhotonCameraSim(cameras.get(currentCamera), cameraProp);
            // Add the simulated camera to view the targets on this simulated field.
            visionSim.addCamera(cameraSim, robotToCamTransforms.get(currentCamera));

            cameraSim.enableDrawWireframe(true);
        }

        // Initialize adaptive measurement noise
        adaptiveR = MatBuilder.fill(Nat.N3(), Nat.N3(), 
                0.1, 0, 0, 
                0, 0.1, 0, 
                0, 0, 0.1);
    }

    /**
     * Get estimated poses from ALL cameras and return the best one
     * 
     * @return The best estimated robot pose from any camera
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        Optional<EstimatedRobotPose> bestEstimate = Optional.empty();
        double bestAmbiguity = Double.MAX_VALUE;  // Lower is better

        for (Map.Entry<String, PhotonCamera> cameraEntry : cameras.entrySet()) {
            String cameraName = cameraEntry.getKey();
            PhotonCamera camera = cameraEntry.getValue();
            PhotonPoseEstimator estimator = photonEstimators.get(cameraName);

            for (var result : camera.getAllUnreadResults()) {
                // Skip results with no targets or high ambiguity
                if (!result.hasTargets() || result.getBestTarget().getPoseAmbiguity() > MINIMUM_AMBIGUITY) {
                    continue;
                }

                Optional<EstimatedRobotPose> visionEst = estimator.update(result);
                updateEstimationStdDevs(visionEst, result.getTargets());

                if (visionEst.isPresent()) {
                    double currentAmbiguity = result.getBestTarget().getPoseAmbiguity();
                    if (bestEstimate.isEmpty() || currentAmbiguity < bestAmbiguity) {
                        bestEstimate = visionEst;
                        bestAmbiguity = currentAmbiguity;
                    }
                }
            }
        }

        // Apply Kawman filter if we have a valid pose
        if (bestEstimate.isPresent()) {
            Pose2d filteredPose = applyKalmanFilter(
                bestEstimate.get().estimatedPose.toPose2d(),
                bestEstimate.get().timestampSeconds
            );
            
            return Optional.of(new EstimatedRobotPose(
                new Pose3d(filteredPose), 
                bestEstimate.get().timestampSeconds, 
                bestEstimate.get().targetsUsed,
                bestEstimate.get().strategy
            ));
        }

        // If no new vision data, predict forward
        if (isFilterInitialized) {
            double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
            double dt = now - lastTimestamp;
            if (dt > 0 && dt < 1.0) {  // Only predict if time difference is reasonable
                // Prediction update with no control input
                poseFilter.predict(VecBuilder.fill(0, 0, 0), dt);
                
                // Extract the predicted state
                Matrix<N3, N1> state = poseFilter.getXhat();
                lastFilteredPose = new Pose2d(
                    state.get(0, 0), 
                    state.get(1, 0), 
                    new Rotation2d(state.get(2, 0))
                );
                lastTimestamp = now;
            }
        }

        return bestEstimate;
    }

    private Pose2d applyKalmanFilter(Pose2d newPose, double timestamp) {
        // Convert pose to state vector
        Matrix<N3, N1> measurement = VecBuilder.fill(
            newPose.getX(),
            newPose.getY(),
            newPose.getRotation().getRadians()
        );
        
        // If this is the first measurement, initialize the filter
        if (!isFilterInitialized) {
            poseFilter.setXhat(measurement);
            lastFilteredPose = newPose;
            lastTimestamp = timestamp;
            isFilterInitialized = true;
            return newPose;
        }
        
        // Time update - predict
        double dt = timestamp - lastTimestamp;
        if (dt > 0 && dt < 1.0) {  // Only update if time difference is reasonable
            // Predict with no control input
            poseFilter.predict(VecBuilder.fill(0, 0, 0), dt);
            
            // H-infinity inspired robust filtering:
            // 1. Calculate innovation (difference between measurement and prediction)
            Matrix<N3, N1> predicted = poseFilter.getXhat();
            Matrix<N3, N1> innovation = measurement.minus(predicted);
            
            // 2. Check if innovation is within acceptable bounds
            double innovationMagnitude = Math.sqrt(
                Math.pow(innovation.get(0, 0), 2) + 
                Math.pow(innovation.get(1, 0), 2)
            );
            
            // 3. Store innovation for adaptive filtering
            innovationHistory.add(innovation);
            while (innovationHistory.size() > INNOVATION_HISTORY_SIZE) {
                innovationHistory.remove();
            }
            
            // 4. Update adaptive measurement noise based on recent innovations
            updateAdaptiveNoise();
            
            // 5. Apply H-infinity inspired robustness
            if (innovationMagnitude <= MAX_INNOVATION_THRESHOLD) {
                // Calculate adaptive gain similar to H-infinity approach
                double trustFactor = Math.max(
                    MIN_TRUST_FACTOR, 
                    1.0 - (innovationMagnitude / MAX_INNOVATION_THRESHOLD) * (1.0 - MIN_TRUST_FACTOR)
                );
                
                // Instead of trying to set R directly, adjust the filter gain behavior
                // by updating process noise 
                Matrix<N3, N3> adjustedP = poseFilter.getP().times(1.0 / trustFactor);
                // The setP method doesn't exist, so we will use a different approach
                // for now, just update our measurement without modifying the filter's parameters
                poseFilter.correct(VecBuilder.fill(0, 0, 0), measurement);
            } else {
                // Innovation too large, likely an outlier
                // Use minimum trust and maximum process noise for robustness
                Matrix<N3, N3> outlierP = poseFilter.getP().times(1.0 / MIN_TRUST_FACTOR);
                // Instead of setP, we will modify our approach
                // Just use a stronger bounded innovation to achieve similar effect
                Matrix<N3, N1> boundedInnovation = innovation.times(MAX_INNOVATION_THRESHOLD / innovationMagnitude);
                Matrix<N3, N1> boundedMeasurement = predicted.plus(boundedInnovation);
                poseFilter.correct(VecBuilder.fill(0, 0, 0), boundedMeasurement);
            }
            
            // Extract the filtered state
            Matrix<N3, N1> state = poseFilter.getXhat();
            lastFilteredPose = new Pose2d(
                state.get(0, 0), 
                state.get(1, 0), 
                new Rotation2d(state.get(2, 0))
            );
            lastTimestamp = timestamp;
        }
        
        return lastFilteredPose;
    }

    /**
     * Updates the adaptive measurement noise based on recent innovations
     * This implements a simplified concept from H-infinity filtering
     * for improved robustness against outliers and non-Gaussian noise
     */
    private void updateAdaptiveNoise() {
        if (innovationHistory.size() < 3) return;
        
        // Calculate variance of recent innovations
        double sumX = 0, sumY = 0, sumTheta = 0;
        double sumXSquared = 0, sumYSquared = 0, sumThetaSquared = 0;
        
        for (Matrix<N3, N1> innovation : innovationHistory) {
            double x = innovation.get(0, 0);
            double y = innovation.get(1, 0);
            double theta = innovation.get(2, 0);
            
            sumX += x;
            sumY += y;
            sumTheta += theta;
            
            sumXSquared += x * x;
            sumYSquared += y * y;
            sumThetaSquared += theta * theta;
        }
        
        int n = innovationHistory.size();
        double varX = (sumXSquared - (sumX * sumX) / n) / (n - 1);
        double varY = (sumYSquared - (sumY * sumY) / n) / (n - 1);
        double varTheta = (sumThetaSquared - (sumTheta * sumTheta) / n) / (n - 1);
        
        // Apply H-infinity inspired tuning to the noise (using GAMMA)
        double robustVarX = varX * (1 + GAMMA * Math.sqrt(varX));
        double robustVarY = varY * (1 + GAMMA * Math.sqrt(varY));
        double robustVarTheta = varTheta * (1 + GAMMA * Math.sqrt(varTheta));
        
        // Ensure minimum values
        robustVarX = Math.max(0.01, robustVarX);
        robustVarY = Math.max(0.01, robustVarY);
        robustVarTheta = Math.max(0.005, robustVarTheta);
        
        // Update adaptive measurement noise matrix
        adaptiveR = MatBuilder.fill(Nat.N3(), Nat.N3(), 
                robustVarX, 0, 0, 
                0, robustVarY, 0, 
                0, 0, robustVarTheta);
    }

    /**
     * Calculates new standard deviations This algorithm is a heuristic that creates
     * dynamic standard
     * deviations based on number of tags, estimation strategy, and distance from
     * the tags.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     * @param targets       All targets in this camera frame
     */
    private void updateEstimationStdDevs(
            Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            curStdDevs = kSingleTagStdDevs;

        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = kSingleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an
            // average-distance metric
            for (var tgt : targets) {
                var tagPose = photonEstimators.get(currentCamera).getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty())
                    continue;
                numTags++;
                avgDist += tagPose
                        .get()
                        .toPose2d()
                        .getTranslation()
                        .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) {
                // No tags visible. Default to single-tag std devs
                curStdDevs = kSingleTagStdDevs;
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                if (numTags > 1)
                    estStdDevs = kMultiTagStdDevs;
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 4)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else
                    estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                curStdDevs = estStdDevs;
            }
        }
    }

    /**
     * Returns the latest standard deviations of the estimated pose from {@link
     * #getEstimatedGlobalPose()}, for use with {@link
     * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
     * SwerveDrivePoseEstimator}. This should
     * only be used when there are targets visible.
     */
    public Matrix<N3, N1> getEstimationStdDevs() {
        return curStdDevs;
    }

    // ----- Simulation

    public void simulationPeriodic(Pose2d robotSimPose) {
        visionSim.update(robotSimPose);
    }

    /** Reset pose history of the robot in the vision system simulation. */
    public void resetSimPose(Pose2d pose) {
        if (Robot.isSimulation())
            visionSim.resetRobotPose(pose);
    }

    /** A Field2d for visualizing our robot and objects on the field. */
    public Field2d getSimDebugField() {
        if (!Robot.isSimulation())
            return null;
        return visionSim.getDebugField();
    }

    /**
     * Gets translations to AprilTags from ALL cameras
     * 
     * @return Map of camera name to tag translation (if visible)
     */
    public Map<String, Translation3d> getAllTagTranslations() {
        Map<String, Translation3d> translations = new HashMap<>();

        // Check ALL cameras for visible tags
        for (Map.Entry<String, PhotonCamera> cameraEntry : cameras.entrySet()) {
            String cameraName = cameraEntry.getKey();
            PhotonCamera camera = cameraEntry.getValue();

            var result = camera.getLatestResult();
            if (result.hasTargets()) {
                PhotonTrackedTarget target = result.getBestTarget();
                Transform3d targetTransform = target.getBestCameraToTarget();
                translations.put(cameraName, targetTransform.getTranslation());
            }
        }

        return translations;
    }

    /**
     * Gets the latest estimated pose from vision! OwO
     * 
     * @return The estimated Pose2d, or null if no valid vision data
     */
    public Pose2d getLatestPose() {
        Optional<EstimatedRobotPose> visionEst = getEstimatedGlobalPose();

        if (visionEst.isPresent()) {
            return visionEst.get().estimatedPose.toPose2d();
        }

        return null;
    }

    public void disableProcessing() {
        processingEnabled = false;
    }

    public boolean hasTarget() {
        // Check ALL cameras for visible tags
        for (Map.Entry<String, PhotonCamera> cameraEntry : cameras.entrySet()) {
            PhotonCamera camera = cameraEntry.getValue();
            
            var result = camera.getLatestResult();
        
            hasTarget = result.hasTargets();
        }
        return hasTarget;
    }

    @Override
    public void periodic() {
        if (!processingEnabled) {
            return; // Skip all vision processing
        }
        // Regular vision processing...
        // We can still keep the camera selector for debug viewing,
        // but now we're processing all cameras regardless of selection
        String selected = cameraChooser.getSelected();
        if (selected != null && selected != currentCamera) {
            currentCamera = selected;
        }

        // Put ALL tag translations on dashboard
        Map<String, Translation3d> allTranslations = getAllTagTranslations();

        // Clear previous values
        for (String camera : cameras.keySet()) {
            SmartDashboard.putNumber(camera + "/Tag/X", 0.0);
            SmartDashboard.putNumber(camera + "/Tag/Y", 0.0);
            SmartDashboard.putNumber(camera + "/Tag/Z", 0.0);
        }

        // Update with new values
        for (Map.Entry<String, Translation3d> entry : allTranslations.entrySet()) {
            String camera = entry.getKey();
            Translation3d translation = entry.getValue();

            SmartDashboard.putNumber(camera + "/Tag/X", translation.getX());
            SmartDashboard.putNumber(camera + "/Tag/Y", translation.getY());
            SmartDashboard.putNumber(camera + "/Tag/Z", translation.getZ());
        }
    }
}
