package frc.robot;

import static frc.robot.Constants.Vision.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
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

public class Vision extends SubsystemBase {
    private final Map<String, PhotonCamera> cameras;
    private final Map<String, PhotonPoseEstimator> photonEstimators;
    private String currentCamera;
    private Matrix<N3, N1> curStdDevs;

    // Simulation
    private PhotonCameraSim cameraSim;
    private VisionSystemSim visionSim;

    private SendableChooser<String> cameraChooser;

    private boolean processingEnabled = true;
    private boolean hasTarget = false;

    public Vision() {
        cameras = new HashMap<>();
        photonEstimators = new HashMap<>();

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
    }

    /**
     * Get estimated poses from ALL cameras and return the best one
     * 
     * @return The best estimated robot pose from any camera
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        Optional<EstimatedRobotPose> bestEstimate = Optional.empty();
        double bestDistance = Double.MAX_VALUE;

        // Check ALL cameras instead of just the current one
        for (Map.Entry<String, PhotonCamera> cameraEntry : cameras.entrySet()) {
            String cameraName = cameraEntry.getKey();
            PhotonCamera camera = cameraEntry.getValue();
            PhotonPoseEstimator estimator = photonEstimators.get(cameraName);

            // Process each camera's results
            for (var result : camera.getAllUnreadResults()) {
                Optional<EstimatedRobotPose> visionEst = estimator.update(result);
                updateEstimationStdDevs(visionEst, result.getTargets());

                // If we got a valid estimate, check if it's better than our current best
                if (visionEst.isPresent()) {
                    // Determine quality metric - here we're using number of tags
                    // You could use different metrics like average distance to tags
                    int numTags = result.getTargets().size();

                    // Higher tag count = better estimate
                    if (bestEstimate.isEmpty() || numTags > bestDistance) {
                        bestEstimate = visionEst;
                        bestDistance = numTags;
                    }
                }
            }
        }

        return bestEstimate;
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
