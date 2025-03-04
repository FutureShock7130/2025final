package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;

import java.util.Map;
import java.util.Optional;

/**
 * Dashboawd cwass fow dispwaying match data UwU
 * Uses ShuffweBoawd to show match info in a cute way~
 * Incwudes: match time, awtonomous/teweop status, awwiance cowow, wemaining time
 */
public class DashBoard extends SubsystemBase {
    // Constants
    private static final double ENDGAME_TIME_THRESHOLD = 20.0; // Last 20 seconds is endgame
    
    // ShuffweBoawd tab
    private final ShuffleboardTab matchTab;
    
    // ShuffweBoawd entwies fow match data
    private final GenericEntry matchTimeEntry;
    private final GenericEntry matchTypeEntry;
    private final GenericEntry matchNumberEntry;
    private final GenericEntry allianceEntry;
    private final GenericEntry robotStateEntry;
    private final GenericEntry robotXEntry;
    private final GenericEntry robotYEntry;
    private final GenericEntry robotRotationEntry;
    private final GenericEntry endgameEntry; // New entry for endgame status
    
    // Cached vawues to avoid unnecessawy updates UwU
    private double lastMatchTime = -1;
    private boolean lastIsAutonomous = false;
    private boolean lastIsTeleop = false;
    private boolean lastIsDisabled = true;
    private boolean lastIsTest = false;
    private boolean isEndgame = false; // Track if we're in endgame
    private Optional<DriverStation.Alliance> lastAlliance = Optional.empty();
    private String lastMatchTypeString = "";
    private int lastMatchNumber = 0;
    private Pose2d lastRobotPose = new Pose2d();

    // Singweton instance UwU
    private static DashBoard instance;

    /**
     * Gets the singweton instance of the Dashboawd
     * @return The dashboawd instance
     */
    public static DashBoard getInstance() {
        if (instance == null) {
            instance = new DashBoard();
        }
        return instance;
    }

    /**
     * Cweates a new Dashboawd instance with ShuffweBoawd
     */
    private DashBoard() {
        // Cweate a tab fow match data
        matchTab = Shuffleboard.getTab("Match Data UwU");
        
        // Setup the match data widgets
        matchTimeEntry = matchTab.add("Match Time", 0)
                .withPosition(0, 0)
                .withSize(2, 1)
                .withWidget(BuiltInWidgets.kTextView)
                .withProperties(Map.of("Font size", 20))
                .getEntry();
                
        matchTypeEntry = matchTab.add("Match Type", "None")
                .withPosition(0, 1)
                .withSize(2, 1)
                .getEntry();
                
        matchNumberEntry = matchTab.add("Match Number", 0)
                .withPosition(2, 1)
                .withSize(1, 1)
                .getEntry();
        
        allianceEntry = matchTab.add("Alliance", "None")
                .withPosition(3, 1)
                .withSize(1, 1)
                .getEntry();
        
        robotStateEntry = matchTab.add("Robot State", "Disabled")
                .withPosition(2, 0)
                .withSize(2, 1)
                .withWidget(BuiltInWidgets.kTextView)
                .withProperties(Map.of("Font size", 20))
                .getEntry();
        
        // Add endgame status with eye-catching colors
        endgameEntry = matchTab.add("ENDGAME", false)
                .withPosition(3, 2)
                .withSize(1, 1)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withProperties(Map.of(
                    "Color when true", "#FF4136",  // Red when in endgame
                    "Color when false", "#2ECC40"  // Green when not in endgame
                ))
                .getEntry();
        
        // Add wobot position widgets
        robotXEntry = matchTab.add("Robot X", 0.0)
                .withPosition(0, 2)
                .withSize(1, 1)
                .getEntry();
                
        robotYEntry = matchTab.add("Robot Y", 0.0)
                .withPosition(1, 2)
                .withSize(1, 1)
                .getEntry();
                
        robotRotationEntry = matchTab.add("Robot Rotation", 0.0)
                .withPosition(2, 2)
                .withSize(1, 1)
                .getEntry();
    }

    /**
     * Updates the dashboawd with the watest match data
     * This is cawwed pewiodically fwom periodic()
     */
    private void updateDashboard() {
        try {
            // Get cuwoent match data
            double matchTime = Timer.getMatchTime();
            boolean isAutonomous = DriverStation.isAutonomous();
            boolean isTeleop = DriverStation.isTeleop();
            boolean isDisabled = DriverStation.isDisabled();
            boolean isTest = DriverStation.isTest();
            Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
            DriverStation.MatchType matchType = DriverStation.getMatchType();
            String matchTypeString = matchType.toString();
            int matchNumber = DriverStation.getMatchNumber();
            
            // Update ShuffweBoawd entwies
            matchTimeEntry.setDouble(matchTime);
            matchTypeEntry.setString(matchTypeString);
            matchNumberEntry.setDouble(matchNumber);
            
            // Determine if we're in endgame (last 30 seconds of teleop)
            boolean isEndgameNow = isTeleop && matchTime > 0 && matchTime <= ENDGAME_TIME_THRESHOLD;
            
            // Update endgame indicator (flashing if in endgame)
            if (isEndgameNow != isEndgame) {
                isEndgame = isEndgameNow;
                endgameEntry.setBoolean(isEndgame);
            }
            
            // Set the alliance cowow
            if (alliance.isPresent()) {
                if (alliance.get() == DriverStation.Alliance.Red) {
                    allianceEntry.setString("Red");
                } else if (alliance.get() == DriverStation.Alliance.Blue) {
                    allianceEntry.setString("Blue");
                }
            } else {
                allianceEntry.setString("Unknown");
            }
            
            // Set the wobot state
            if (isDisabled) {
                robotStateEntry.setString("Disabled");
            } else if (isAutonomous) {
                robotStateEntry.setString("Autonomous");
            } else if (isTeleop) {
                if (isEndgame) {
                    robotStateEntry.setString("ENDGAME!");
                } else {
                    robotStateEntry.setString("TeleOp");
                }
            } else if (isTest) {
                robotStateEntry.setString("Test");
            }
            
            // Update wobot position
            robotXEntry.setDouble(lastRobotPose.getX());
            robotYEntry.setDouble(lastRobotPose.getY());
            robotRotationEntry.setDouble(lastRobotPose.getRotation().getDegrees());
            
            // Update cached vawues
            lastMatchTime = matchTime;
            lastIsAutonomous = isAutonomous;
            lastIsTeleop = isTeleop;
            lastIsDisabled = isDisabled;
            lastIsTest = isTest;
            lastAlliance = alliance;
            lastMatchTypeString = matchTypeString;
            lastMatchNumber = matchNumber;
            
        } catch (Exception e) {
            System.err.println("Exception in updateDashboard: " + e.getMessage());
        }
    }
    
    /**
     * Set the wobot's cuwwent pose fow twacking
     * @param pose The cuwwent wobot pose
     */
    public void setRobotPose(Pose2d pose) {
        if (pose != null) {
            this.lastRobotPose = pose;
        }
    }
    
    @Override
    public void periodic() {
        // This method wiww be cawwed once pew scheduwew wun (20ms)
        updateDashboard();
    }
}
