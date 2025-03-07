
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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.drive.Drive;

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
        static {

        }

        // REEF scoring pose
        public static final Pose2d A;
        public static final Pose2d B;
        public static final Pose2d C;
        public static final Pose2d D;
        public static final Pose2d E;
        public static final Pose2d F;
        public static final Pose2d G;
        public static final Pose2d H;
        public static final Pose2d I;
        public static final Pose2d J;
        public static final Pose2d K;
        public static final Pose2d L;
        public static final Pose2d AB;
        public static final Pose2d CD;
        public static final Pose2d EF;
        public static final Pose2d GH;
        public static final Pose2d IJ;
        public static final Pose2d KL;
        static {
            if (DriverStation.getAlliance().get() == Alliance.Blue) {
                A = new Pose2d(new Translation2d(3.177, 4.191), Rotation2d.fromDegrees(0));
                B = new Pose2d(new Translation2d(3.235, 3.876), Rotation2d.fromDegrees(0));
                C = new Pose2d(new Translation2d(3.722, 2.987), Rotation2d.fromDegrees(60));
                D = new Pose2d(new Translation2d(3.969, 2.886), Rotation2d.fromDegrees(60));
                E = new Pose2d(new Translation2d(4.984, 2.865), Rotation2d.fromDegrees(120));
                F = new Pose2d(new Translation2d(5.246, 3.055), Rotation2d.fromDegrees(120));
                G = new Pose2d(new Translation2d(5.725, 3.916), Rotation2d.fromDegrees(180));
                H = new Pose2d(new Translation2d(5.701, 4.191), Rotation2d.fromDegrees(180));
                I = new Pose2d(new Translation2d(5.255, 5.017), Rotation2d.fromDegrees(-120));
                J = new Pose2d(new Translation2d(4.981, 5.157), Rotation2d.fromDegrees(-120));
                K = new Pose2d(new Translation2d(4.008, 5.231), Rotation2d.fromDegrees(-60));
                L = new Pose2d(new Translation2d(3.767, 5.039), Rotation2d.fromDegrees(-60));
                AB = new Pose2d(new Translation2d(2.264, 3.976), Rotation2d.fromDegrees(0));
                CD = new Pose2d(new Translation2d(3.435, 2.126), Rotation2d.fromDegrees(60));
                EF = new Pose2d(new Translation2d(5.701, 2.019), Rotation2d.fromDegrees(120));
                GH = new Pose2d(new Translation2d(6.841, 4.070), Rotation2d.fromDegrees(180));
                IJ = new Pose2d(new Translation2d(5.577, 6.027), Rotation2d.fromDegrees(-120));
                KL = new Pose2d(new Translation2d(3.392, 5.957), Rotation2d.fromDegrees(-60));
            } else {
                A = new Pose2d(new Translation2d(14.272, 4.189), Rotation2d.fromDegrees(-180));
                B = new Pose2d(new Translation2d(14.294, 3.847), Rotation2d.fromDegrees(-180));
                C = new Pose2d(new Translation2d(13.827, 5.028), Rotation2d.fromDegrees(-120));
                D = new Pose2d(new Translation2d(13.893, 4.523), Rotation2d.fromDegrees(-120));
                E = new Pose2d(new Translation2d(12.576, 5.183), Rotation2d.fromDegrees(-60));
                F = new Pose2d(new Translation2d(12.273, 5.022), Rotation2d.fromDegrees(-60));
                G = new Pose2d(new Translation2d(11.799, 4.140), Rotation2d.fromDegrees(0));
                H = new Pose2d(new Translation2d(11.812, 3.833), Rotation2d.fromDegrees(0));
                I = new Pose2d(new Translation2d(12.336, 3.057), Rotation2d.fromDegrees(60));
                J = new Pose2d(new Translation2d(12.546, 2.846), Rotation2d.fromDegrees(60));
                K = new Pose2d(new Translation2d(13.609, 2.861), Rotation2d.fromDegrees(120));
                L = new Pose2d(new Translation2d(13.812, 3.043), Rotation2d.fromDegrees(120));
                AB = new Pose2d(new Translation2d(15.189, 4.189), Rotation2d.fromDegrees(-180));
                CD = new Pose2d(new Translation2d(14.298, 6.195), Rotation2d.fromDegrees(-120));
                EF = new Pose2d(new Translation2d(12.053, 6.076), Rotation2d.fromDegrees(-60));
                GH = new Pose2d(new Translation2d(10.887, 4.045), Rotation2d.fromDegrees(0));
                IJ = new Pose2d(new Translation2d(11.996, 2.005), Rotation2d.fromDegrees(60));
                KL = new Pose2d(new Translation2d(14.359, 1.981), Rotation2d.fromDegrees(120));
            }
        }

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