package frc.robot.subsystems;

public class SuperStructPosition {
    
    public static class ElevatorPos {
        public static double L1 = -0.001 * 0.6;
        public static double L2 = 18.69420 * 0.6;
        public static double L3 = 69.420 * 0.6;
        public static double L4 = 165 * 0.6;
        public static double CS = -0.001 * 0.6;
        public static double Default = -0.02 * 0.6;
    }

    public static class GrabberPos {
        public static double L1_L2_L3 = 0.463135;
        public static double L4 = 0.52074;
        public static double Default = 0.618896;
        public static double CS = 0.289307;
        public static double HIT_ALGAE = 0.412295;
    }

    public static class IntakePos {
        public static double Default = -0.390137;
        public static double Down = -0.229248;
        public static double Intake = -0.226074;
    }
}
