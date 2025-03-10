package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class NavigationController extends SubsystemBase {
    
    private enum DestinationState {
        MANUAL_DRIVING,
        PATHFINDING_TO_A,
        PATHFINDING_TO_B,
        PATHFINDING_TO_AB_THEN_A,
        PATHFINDING_TO_AB_THEN_B,
        PATHFINDING_TO_AB,
        PATHFINDING_TO_C,
        PATHFINDING_TO_D,
        PATHFINDING_TO_CD_THEN_C,
        PATHFINDING_TO_CD_THEN_D,
        PATHFINDING_TO_CD,
        PATHFINDING_TO_E,
        PATHFINDING_TO_F,
        PATHFINDING_TO_EF_THEN_E,
        PATHFINDING_TO_EF_THEN_F,
        PATHFINDING_TO_EF,
        PATHFINDING_TO_G,
        PATHFINDING_TO_H,
        PATHFINDING_TO_GH_THEN_G,
        PATHFINDING_TO_GH_THEN_H,
        PATHFINDING_TO_GH,
        PATHFINDING_TO_I,
        PATHFINDING_TO_J,
        PATHFINDING_TO_IJ_THEN_I,
        PATHFINDING_TO_IJ_THEN_J,
        PATHFINDING_TO_IJ,
        PATHFINDING_TO_K,
        PATHFINDING_TO_L,
        PATHFINDING_TO_KL_THEN_K,
        PATHFINDING_TO_KL_THEN_L,
        PATHFINDING_TO_KL,
        PATHFINDING_TO_CSL,
        PATHFINDING_TO_CSR
    }
    
    // Controllers
    private final XboxController driver;
    private final Joystick buttonBox1;
    private final Joystick buttonBox2;
    
    // Navigation state
    private DestinationState currentDestination = DestinationState.MANUAL_DRIVING;
    private DestinationState nextDestination = null;
    private Command activePathCommand = null;
    private final PathConstraints constraints;

    private static NavigationController mInstance = null;

    public static synchronized NavigationController getInstance() {
      if (mInstance == null) {
        mInstance = new NavigationController();
      }
      return mInstance;
    }
    
    /**
     * Creates a new navigation controller to manage path planning
     */
    public NavigationController() {
        driver = new XboxController(0);
        buttonBox1 = new Joystick(1);
        buttonBox2 = new Joystick(2);
        
        // Initialize path constraints 
        this.constraints = new PathConstraints(3, 3, 2 * Math.PI, 4 * Math.PI);
    }
    
    @Override
    public void periodic() {
        // Check if driver wants manual control
        if (driverWantsControl()) {
            cancelPathfinding();
        }

        // Check if we completed a step in a multi-step navigation
        if (activePathCommand == null && nextDestination != null) {
            // Start the next step
            startPathfinding(nextDestination);
            nextDestination = null;
        }

        // Check button presses to set new destinations
        DestinationState newDestination = checkButtonPresses();
        
        if (newDestination != currentDestination) {
            startPathfinding(newDestination);
        }
    }
    
    /**
     * Checks all controller inputs and returns the requested destination
     */
    private DestinationState checkButtonPresses() {
        
        if (buttonBox1.getRawAxis(1) >= 0.5) {
            return DestinationState.PATHFINDING_TO_AB_THEN_A;
        } else if (buttonBox1.getRawButtonPressed(5)) {
            return DestinationState.PATHFINDING_TO_AB_THEN_B;
        } else if (buttonBox1.getRawButtonPressed(6)) {
            return DestinationState.PATHFINDING_TO_CD_THEN_C;
        } else if (buttonBox1.getRawButtonPressed(1)) {
            return DestinationState.PATHFINDING_TO_CD_THEN_D;
        } else if (buttonBox1.getRawButtonPressed(10)) {
            return DestinationState.PATHFINDING_TO_EF_THEN_E;
        } else if (buttonBox1.getRawButtonPressed(9)) {
            return DestinationState.PATHFINDING_TO_EF_THEN_F;
        } else if (buttonBox1.getRawButtonPressed(8)) {
            return DestinationState.PATHFINDING_TO_GH_THEN_G;
        } else if (buttonBox1.getRawButtonPressed(7)) {
            return DestinationState.PATHFINDING_TO_GH_THEN_H;
        } else if (buttonBox1.getRawButtonPressed(4)) {
            return DestinationState.PATHFINDING_TO_IJ_THEN_I;
        } else if (buttonBox1.getRawButtonPressed(3)) {
            return DestinationState.PATHFINDING_TO_IJ_THEN_J;
        } else if (buttonBox2.getRawButtonPressed(3)) {
            return DestinationState.PATHFINDING_TO_KL_THEN_K;
        } else if (buttonBox1.getRawButtonPressed(2)) {
            return DestinationState.PATHFINDING_TO_KL_THEN_L;
        // } else if (driver.getLeftBumperPressed()) {
        //     return DestinationState.PATHFINDING_TO_CSL;
        // } else if (driver.getRightBumperPressed()) {
        //     return DestinationState.PATHFINDING_TO_CSR;
        }
        
        return currentDestination;
    }
    
    /**
     * Starts pathfinding to the specified destination
     */
    private void startPathfinding(DestinationState destination) {
        // Cancel any existing pathfinding
        cancelPathfinding();
        
        if (destination == DestinationState.PATHFINDING_TO_AB_THEN_A) {
            nextDestination = DestinationState.PATHFINDING_TO_A;
            destination = DestinationState.PATHFINDING_TO_AB;
        } else if (destination == DestinationState.PATHFINDING_TO_AB_THEN_B) {
            nextDestination = DestinationState.PATHFINDING_TO_B;
            destination = DestinationState.PATHFINDING_TO_AB;
        } else if (destination == DestinationState.PATHFINDING_TO_CD_THEN_C) {
            nextDestination = DestinationState.PATHFINDING_TO_C;
            destination = DestinationState.PATHFINDING_TO_CD;
        } else if (destination == DestinationState.PATHFINDING_TO_CD_THEN_D) {
            nextDestination = DestinationState.PATHFINDING_TO_D;
            destination = DestinationState.PATHFINDING_TO_CD;
        } else if (destination == DestinationState.PATHFINDING_TO_EF_THEN_E) {
            nextDestination = DestinationState.PATHFINDING_TO_E;
            destination = DestinationState.PATHFINDING_TO_EF;
        } else if (destination == DestinationState.PATHFINDING_TO_EF_THEN_F) {
            nextDestination = DestinationState.PATHFINDING_TO_F;
            destination = DestinationState.PATHFINDING_TO_EF;
        } else if (destination == DestinationState.PATHFINDING_TO_GH_THEN_G) {
            nextDestination = DestinationState.PATHFINDING_TO_G;
            destination = DestinationState.PATHFINDING_TO_GH;
        } else if (destination == DestinationState.PATHFINDING_TO_GH_THEN_H) {
            nextDestination = DestinationState.PATHFINDING_TO_H;
            destination = DestinationState.PATHFINDING_TO_GH;
        } else if (destination == DestinationState.PATHFINDING_TO_IJ_THEN_I) {
            nextDestination = DestinationState.PATHFINDING_TO_I;
            destination = DestinationState.PATHFINDING_TO_IJ;
        } else if (destination == DestinationState.PATHFINDING_TO_IJ_THEN_J) {
            nextDestination = DestinationState.PATHFINDING_TO_J;
            destination = DestinationState.PATHFINDING_TO_IJ;
        } else if (destination == DestinationState.PATHFINDING_TO_KL_THEN_K) {
            nextDestination = DestinationState.PATHFINDING_TO_K;
            destination = DestinationState.PATHFINDING_TO_KL;
        } else if (destination == DestinationState.PATHFINDING_TO_KL_THEN_L) {
            nextDestination = DestinationState.PATHFINDING_TO_L;
            destination = DestinationState.PATHFINDING_TO_KL;
        }
        
        // Find the target pose based on the destination
        Pose2d targetPose = getPoseForDestination(destination);

        
        // Create the pathfinding command
        activePathCommand = AutoBuilder.pathfindToPose(targetPose, constraints)
            .until(() -> driverWantsControl())
            .finallyDo((interrupted) -> {
                if (!interrupted) {
                    if (nextDestination == null) {
                        currentDestination = DestinationState.MANUAL_DRIVING;
                    }
                } else {
                    nextDestination = null;
                    currentDestination = DestinationState.MANUAL_DRIVING;
                }
                activePathCommand = null;
            });
        
        // Schedule the command
        activePathCommand.schedule();
        
        // Update current destination
        currentDestination = destination;
    }
    
    /**
     * Returns the target pose for the given destination
     */
    private Pose2d getPoseForDestination(DestinationState destination) {
        switch (destination) {
            case PATHFINDING_TO_A:
                return Constants.FieldConstants.A;
            case PATHFINDING_TO_B:
                return Constants.FieldConstants.B;
            case PATHFINDING_TO_AB:
                return Constants.FieldConstants.AB;
            case PATHFINDING_TO_C:
                return Constants.FieldConstants.C;
            case PATHFINDING_TO_D:
                return Constants.FieldConstants.D;
            case PATHFINDING_TO_CD:
                return Constants.FieldConstants.CD;
            case PATHFINDING_TO_E:
                return Constants.FieldConstants.E;
            case PATHFINDING_TO_F:
                return Constants.FieldConstants.F;
            case PATHFINDING_TO_EF:
                return Constants.FieldConstants.EF;
            case PATHFINDING_TO_G:
                return Constants.FieldConstants.G;
            case PATHFINDING_TO_H:
                return Constants.FieldConstants.H;
            case PATHFINDING_TO_GH:
                return Constants.FieldConstants.GH;
            case PATHFINDING_TO_I:
                return Constants.FieldConstants.I;
            case PATHFINDING_TO_J:
                return Constants.FieldConstants.J;
            case PATHFINDING_TO_IJ:
                return Constants.FieldConstants.IJ;
            case PATHFINDING_TO_K:
                return Constants.FieldConstants.K;
            case PATHFINDING_TO_L:
                return Constants.FieldConstants.L;
            case PATHFINDING_TO_KL:
                return Constants.FieldConstants.KL;
            case PATHFINDING_TO_CSL:
                return Constants.FieldConstants.CSL;
            case PATHFINDING_TO_CSR:
                return Constants.FieldConstants.CSR;
            default:
                return null;
        }
    }
    
    /**
     * Cancels any active pathfinding operation
     */
    public void cancelPathfinding() {
        if (activePathCommand != null) {
            activePathCommand.cancel();
            activePathCommand = null;
        }
        currentDestination = DestinationState.MANUAL_DRIVING;
    }
    
    /**
     * Checks if the driver is attempting to take manual control
     */
    private boolean driverWantsControl() {
        return Math.abs(driver.getLeftX()) > 0.3 ||
               Math.abs(driver.getLeftY()) > 0.3 ||
               Math.abs(driver.getRightX()) > 0.3 ||
               Math.abs(driver.getRightY()) > 0.3;
    }
} 