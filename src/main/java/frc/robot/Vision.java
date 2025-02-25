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
    
    public Vision() {
        cameras = new HashMap<>();
        photonEstimators = new HashMap<>();
        
        // Define transforms for each camera
        Map<String, Transform3d> robotToCamTransforms = new HashMap<>();
        robotToCamTransforms.put("CAM_1", new Transform3d(
            new Translation3d(0.3, 0.25, 0.20),  // right camera
            new Rotation3d(0.0, Math.toRadians(0), 0.0))); 
            
        robotToCamTransforms.put("CAM_2", new Transform3d(
            new Translation3d(0, 0, 0),  // left camera
            new Rotation3d(0.0, Math.toRadians(0), Math.toRadians(0)))); 
            
        robotToCamTransforms.put("CAM_3", new Transform3d(
            new Translation3d(0, 0, 0),  // top camera
            new Rotation3d(0.0, Math.toRadians(0), 0.0))); 
        
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
            
        // Listen for camera selection changes
        NetworkTableInstance.getDefault()
            .getTable("SmartDashboard")
            .getSubTable("Camera Selection")
            .addListener(
                EnumSet.of(NetworkTableEvent.Kind.kValueAll), 
                (table, key, event) -> 
                    currentCamera = cameraChooser.getSelected());

        // ----- Simulation
        if (Robot.isSimulation()) {
            // Create the vision system simulation which handles cameras and targets on the field.
            visionSim = new VisionSystemSim("main");
            // Add all the AprilTags inside the tag layout as visible targets to this simulated field.
            visionSim.addAprilTags(kTagLayout);
            // Create simulated camera properties. These can be set to mimic your actual camera.
            var cameraProp = new SimCameraProperties();
            cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
            cameraProp.setCalibError(0.35, 0.10);
            cameraProp.setFPS(15);
            cameraProp.setAvgLatencyMs(50);
            cameraProp.setLatencyStdDevMs(15);
            // Create a PhotonCameraSim which will update the linked PhotonCamera's values with visible
            // targets.
            cameraSim = new PhotonCameraSim(cameras.get(currentCamera), cameraProp);
            // Add the simulated camera to view the targets on this simulated field.
            visionSim.addCamera(cameraSim, robotToCamTransforms.get(currentCamera));

            cameraSim.enableDrawWireframe(true);
        }
    }

    /**
     * The latest estimated robot pose on the field from vision data. This may be empty. This should
     * only be called once per loop.
     *
     * <p>Also includes updates for the standard deviations, which can (optionally) be retrieved with
     * {@link getEstimationStdDevs}
     *
     * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
     *     used for estimation.
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        // Use current selected camera UwU
        PhotonCamera camera = cameras.get(currentCamera);
        PhotonPoseEstimator estimator = photonEstimators.get(currentCamera);
        
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        for (var change : camera.getAllUnreadResults()) {
            visionEst = estimator.update(change);
            updateEstimationStdDevs(visionEst, change.getTargets());
        }
        return visionEst;
    }

    /**
     * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard
     * deviations based on number of tags, estimation strategy, and distance from the tags.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     * @param targets All targets in this camera frame
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

            // Precalculation - see how many tags we found, and calculate an average-distance metric
            for (var tgt : targets) {
                var tagPose = photonEstimators.get(currentCamera).getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty()) continue;
                numTags++;
                avgDist +=
                        tagPose
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
                if (numTags > 1) estStdDevs = kMultiTagStdDevs;
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 4)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                curStdDevs = estStdDevs;
            }
        }
    }

    /**
     * Returns the latest standard deviations of the estimated pose from {@link
     * #getEstimatedGlobalPose()}, for use with {@link
     * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}. This should
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
        if (Robot.isSimulation()) visionSim.resetRobotPose(pose);
    }

    /** A Field2d for visualizing our robot and objects on the field. */
    public Field2d getSimDebugField() {
        if (!Robot.isSimulation()) return null;
        return visionSim.getDebugField();
    }

    /**
     * Gets the latest estimated pose from vision! OwO
     * @return The estimated Pose2d, or null if no valid vision data
     */
    public Pose2d getLatestPose() {
        Optional<EstimatedRobotPose> visionEst = getEstimatedGlobalPose();
        
        // Check if we have a valid vision estimate >w<
        if (visionEst.isPresent()) {
            return visionEst.get().estimatedPose.toPose2d();
        }
        
        return null; // No valid pose found ʕ•ᴥ•ʔ
    }

    /**
     * Gets the 3D translation to the best AprilTag currently visible UwU
     * @return Translation3d to the tag, or null if no tag is visible
     */
    public Translation3d getTagTranslation() {
        PhotonCamera camera = cameras.get(currentCamera);
        var result = camera.getLatestResult();
        
        // Check if we have any targets ʕ•ᴥ•ʔ
        if (result.hasTargets()) {
            // Get best target (closest to camera center)
            PhotonTrackedTarget target = result.getBestTarget();
            
            // Get the transform to the target
            Transform3d targetTransform = target.getBestCameraToTarget();
            return targetTransform.getTranslation();
        }
        
        return null;  // No targets found (╥﹏╥)
    }

    @Override
    public void periodic() {
        // Update camera selection if needed
        String selected = cameraChooser.getSelected();
        if (selected != null && selected != currentCamera) {
            currentCamera = selected;
        }

        // Put tag translation on dashboard ✨
        Translation3d tagTranslation = getTagTranslation();
        if (tagTranslation != null) {
            SmartDashboard.putNumber("Tag/X Distance (m)", tagTranslation.getX());
            SmartDashboard.putNumber("Tag/Y Distance (m)", tagTranslation.getY());
            SmartDashboard.putNumber("Tag/Z Distance (m)", tagTranslation.getZ());
        } else {
            // Clear values when no tag is visible UwU
            SmartDashboard.putNumber("Tag/X Distance (m)", 0.0);
            SmartDashboard.putNumber("Tag/Y Distance (m)", 0.0);
            SmartDashboard.putNumber("Tag/Z Distance (m)", 0.0);
        }
    }
}
