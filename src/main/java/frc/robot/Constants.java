
package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final Mode currentMode = Mode.REAL;

    public static enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }

    public static class Vision {
        // The layout of the AprilTags on the field
        public static final AprilTagFieldLayout kTagLayout = AprilTagFields.k2025Reefscape
                .loadAprilTagLayoutField();

        // The standard deviations of our vision estimated poses, which affect
        // correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(48, 48, 8);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(24, 24, 1);
    }

    public static class FieldConstants {
        // starting pose
        public static final Pose2d Left = new Pose2d(new Translation2d(7.231, 7.300), Rotation2d.fromDegrees(180));
        public static final Pose2d Mid = new Pose2d(new Translation2d(8.068, 6.178), Rotation2d.fromDegrees(180));
        public static final Pose2d Right = new Pose2d(new Translation2d(8.068, 5.067), Rotation2d.fromDegrees(180));
        // REEF scoring pose
        public static final Pose2d A = new Pose2d(new Translation2d(3.160, 4.190), Rotation2d.fromDegrees(0));
        public static final Pose2d B = new Pose2d(new Translation2d(3.16, 3.853), Rotation2d.fromDegrees(0));
        public static final Pose2d C = new Pose2d(new Translation2d(3.743, 2.990), Rotation2d.fromDegrees(60));
        public static final Pose2d D = new Pose2d(new Translation2d(4.061, 2.826), Rotation2d.fromDegrees(60));
        public static final Pose2d E = new Pose2d(new Translation2d(5.026, 2.785), Rotation2d.fromDegrees(120));
        public static final Pose2d F = new Pose2d(new Translation2d(5.341, 3), Rotation2d.fromDegrees(120));
        public static final Pose2d G = new Pose2d(new Translation2d(5.8460, 3.859), Rotation2d.fromDegrees(180));
        public static final Pose2d H = new Pose2d(new Translation2d(5.818, 4.195), Rotation2d.fromDegrees(180));
        public static final Pose2d I = new Pose2d(new Translation2d(5.321, 5.078), Rotation2d.fromDegrees(-120));
        public static final Pose2d J = new Pose2d(new Translation2d(4.981, 5.238), Rotation2d.fromDegrees(-120));
        public static final Pose2d K = new Pose2d(new Translation2d(3.990, 5.211), Rotation2d.fromDegrees(-60));
        public static final Pose2d L = new Pose2d(new Translation2d(3.641, 5.120), Rotation2d.fromDegrees(-60));
        public static final Pose2d AB = new Pose2d(new Translation2d(2.393, 4.110), Rotation2d.fromDegrees(0));
        public static final Pose2d CD = new Pose2d(new Translation2d(3.330, 2.035), Rotation2d.fromDegrees(60));
        public static final Pose2d EF = new Pose2d(new Translation2d(5.635, 2.215), Rotation2d.fromDegrees(120));
        public static final Pose2d GH = new Pose2d(new Translation2d(6.682, 3.940), Rotation2d.fromDegrees(180));
        public static final Pose2d IJ = new Pose2d(new Translation2d(5.565, 6.076), Rotation2d.fromDegrees(-120));
        public static final Pose2d KL = new Pose2d(new Translation2d(3.270, 5.986), Rotation2d.fromDegrees(-60));
        
        // Algae & Coral
        public static final Pose2d LA = new Pose2d(new Translation2d(1.843, 5.848), Rotation2d.fromDegrees(180));
        public static final Pose2d LC = new Pose2d(new Translation2d(1.843, 5.848), Rotation2d.fromDegrees(180));
        public static final Pose2d MA = new Pose2d(new Translation2d(1.843, 4.020), Rotation2d.fromDegrees(180));
        public static final Pose2d MC = new Pose2d(new Translation2d(1.843, 4.020), Rotation2d.fromDegrees(180));
        public static final Pose2d RA = new Pose2d(new Translation2d(1.843, 2.166), Rotation2d.fromDegrees(180));
        public static final Pose2d RC = new Pose2d(new Translation2d(1.843, 2.166), Rotation2d.fromDegrees(180));
        // PROCESSOR
        public static final Pose2d PRO = new Pose2d(new Translation2d(6.347, 0.573), Rotation2d.fromDegrees(-90));
        // Coral Station
        public static final Pose2d CSR = new Pose2d(new Translation2d(1.175, 0.936), Rotation2d.fromDegrees(-126));
        public static final Pose2d CSL = new Pose2d(new Translation2d(1.073, 7.300), Rotation2d.fromDegrees(126));
    }

}