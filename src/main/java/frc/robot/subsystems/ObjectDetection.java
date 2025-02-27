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
import frc.robot.subsystems.drive.Drive;
import java.util.List;
import java.util.Map;
import java.util.Optional;

/**
 * 物件偵測子系統
 * 這個子系統使用PhotonVision來偵測分類為"0"或"1"的物件
 * 提供以下功能:
 * - 透過Shuffleboard控制追蹤功能
 * - 追蹤並跟隨偵測到的物件
 * - 自動對準物件
 * - 追蹤最近的物件
 */
public class ObjectDetection extends SubsystemBase {
    private final PhotonCamera camera;
    private int targetClass = 0; // 預設追蹤class 0 (有效值: 0或1)
    private final PIDController turnController;
    private final PIDController driveController;
    private Drive driveSubsystem;
    
    // 命令狀態
    private boolean isAiming = false;
    private boolean isFollowing = false;
    private Command activeCommand = null;
    
    // Shuffleboard 項目
    private final ShuffleboardTab visionTab = Shuffleboard.getTab("物件偵測");
    private GenericEntry targetClassEntry;
    private GenericEntry targetAreaEntry;
    private GenericEntry aimToleranceEntry;
    private GenericEntry aimButtonEntry;
    private GenericEntry followButtonEntry;
    private GenericEntry stopButtonEntry;
    

    private static final double TARGET_AREA_SETPOINT = 15.0; 
    private static final double AIM_TOLERANCE_DEGREES = 2.0; 
    
    public ObjectDetection() {
        camera = new PhotonCamera("WEB-CAM");
        

        turnController = new PIDController(0.05, 0, 0.005);
        driveController = new PIDController(0.1, 0, 0);
        

        setupShuffleboardControls();
    }
    
    /**
     * 設置Shuffleboard控制項
     */
    private void setupShuffleboardControls() {

        targetClassEntry = visionTab.add("目標 (0 or 1)", targetClass)
            .withPosition(0, 0)
            .withSize(1, 1)
            .getEntry();
        

        targetAreaEntry = visionTab.add("target_area_set", TARGET_AREA_SETPOINT)
            .withPosition(1, 0)
            .withSize(1, 1)
            .getEntry();
        

        aimToleranceEntry = visionTab.add("aim_degree", AIM_TOLERANCE_DEGREES)
            .withPosition(2, 0)
            .withSize(1, 1)
            .getEntry();
        
    
        aimButtonEntry = visionTab.add("對準目標", false)
            .withPosition(0, 1)
            .withSize(1, 1)
            .withProperties(Map.of("colorWhenTrue", "green"))
            .getEntry();
        
        // 跟隨按鈕
        followButtonEntry = visionTab.add("跟隨目標", false)
            .withPosition(1, 1)
            .withSize(1, 1)
            .withProperties(Map.of("colorWhenTrue", "blue"))
            .getEntry();
        
        // 停止按鈕
        stopButtonEntry = visionTab.add("停止", false)
            .withPosition(2, 1)
            .withSize(1, 1)
            .withProperties(Map.of("colorWhenTrue", "red"))
            .getEntry();
        
        // 顯示狀態
        visionTab.addBoolean("有目標", this::isTargetVisible)
            .withPosition(0, 2)
            .withSize(1, 1);
        
        visionTab.addBoolean("已對準", this::isAimedAtTarget)
            .withPosition(1, 2)
            .withSize(1, 1);
        
        visionTab.addString("狀態", () -> {
            if (isFollowing) return "跟隨中";
            if (isAiming) return "對準中";
            return "閒置";
        })
            .withPosition(2, 2)
            .withSize(1, 1);
    }
    
    /**
     * 設置驅動子系統
     * 必須在使用對準或跟隨功能前設置
     */
    public void setDriveSubsystem(Drive drive) {
        this.driveSubsystem = drive;
    }
    
    @Override
    public void periodic() {
        // 獲取最新的PhotonVision結果
        var result = camera.getLatestResult();
        
        // 顯示目標資訊
        if (result.hasTargets()) {
            var target = getBestTarget(result);
            if (target != null) {
                SmartDashboard.putNumber("偵測到的類別", target.getFiducialId());
                SmartDashboard.putNumber("目標偏航角", target.getYaw());
                SmartDashboard.putNumber("目標俯仰角", target.getPitch());
                SmartDashboard.putNumber("目標區域", target.getArea());
            }
        }
        
        // 檢查用戶是否從Shuffleboard更新了設定
        checkShuffleboardControls();
    }
    
    /**
     * 檢查Shuffleboard控制項的變更
     */
    private void checkShuffleboardControls() {
        // 讀取目標類別設定
        int newTargetClass = (int) targetClassEntry.getDouble(targetClass);
        if (newTargetClass != targetClass && (newTargetClass == 0 || newTargetClass == 1)) {
            targetClass = newTargetClass;
        }
        
        // 檢查按鈕狀態
        boolean aimRequested = aimButtonEntry.getBoolean(false);
        boolean followRequested = followButtonEntry.getBoolean(false);
        boolean stopRequested = stopButtonEntry.getBoolean(false);
        
        // 處理停止請求
        if (stopRequested) {
            stopAllCommands();
            stopButtonEntry.setBoolean(false);
        }
        // 處理對準請求
        else if (aimRequested && !isAiming && !isFollowing) {
            startAiming();
            aimButtonEntry.setBoolean(false);
        }
        // 處理跟隨請求
        else if (followRequested && !isFollowing) {
            startFollowing();
            followButtonEntry.setBoolean(false);
        }
    }
    
    /**
     * 開始對準目標
     */
    private void startAiming() {
        if (driveSubsystem == null) {
            System.err.println("錯誤: 未設置驅動子系統！");
            return;
        }
        
        stopAllCommands();
        isAiming = true;
        activeCommand = new AimAtTargetCommand(driveSubsystem);
        CommandScheduler.getInstance().schedule(activeCommand);
    }
    
    /**
     * 開始跟隨目標
     */
    private void startFollowing() {
        if (driveSubsystem == null) {
            System.err.println("錯誤: 未設置驅動子系統！");
            return;
        }
        
        stopAllCommands();
        isFollowing = true;
        activeCommand = new FollowTargetCommand(driveSubsystem);
        CommandScheduler.getInstance().schedule(activeCommand);
    }
    
    /**
     * 停止所有命令
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
     * 根據當前目標類別(0或1)從結果中獲取最佳目標
     */
    public PhotonTrackedTarget getBestTarget(PhotonPipelineResult result) {
        if (!result.hasTargets()) {
            return null;
        }
        
        List<PhotonTrackedTarget> targets = result.getTargets();
        
        // 首先，嘗試尋找與類別匹配的目標
        for (PhotonTrackedTarget target : targets) {
            // 假設fiducialId字段包含類別(0或1)
            if (target.getFiducialId() == targetClass) {
                return target;
            }
        }
        
        // 如果沒有匹配的類別，返回最近的目標
        return getClosestTarget(targets);
    }
    
    /**
     * 基於面積(越大=越近)從目標列表中獲取最近的目標
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
     * 設置要追蹤的類別(0或1)
     */
    public void setTargetClass(int classID) {
        if (classID == 0 || classID == 1) {
            this.targetClass = classID;
            targetClassEntry.setDouble(classID);
        }
    }
    
    /**
     * 計算對準目標所需的轉向速度
     * @return 轉向速度(正值=向右轉，負值=向左轉)
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
        
        // 使用PID計算居中目標所需的轉向量
        // 否定是因為正yaw意味著目標在右側，
        // 所以我們需要向左轉(負值)
        return -turnController.calculate(target.getYaw(), 0);
    }
    
    /**
     * 計算接近目標所需的驅動速度
     * @return 驅動速度(正值=前進，負值=後退)
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
        
        // 用面積作為距離的代理(面積越大=距離越近)
        double area = target.getArea();
        double targetArea = targetAreaEntry.getDouble(TARGET_AREA_SETPOINT);
        
        // 使用PID計算達到理想區域所需的驅動速度
        // 注意：如果當前區域 < 目標區域，我們需要前進(正值)
        // 如果當前區域 > 目標區域，我們需要後退(負值)
        return driveController.calculate(area, targetArea);
    }
    
    /**
     * 檢查是否可以看到具有指定類別的目標
     */
    public boolean isTargetVisible() {
        var result = camera.getLatestResult();
        if (!result.hasTargets()) {
            return false;
        }
        
        var target = getBestTarget(result);
        return target != null;
    }
    
    /**
     * 檢查我們是否已對準目標
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
        // 如果在容忍範圍內則視為已對準
        return Math.abs(target.getYaw()) < tolerance;
    }
    
    /**
     * 獲取用於物件偵測的攝像機
     */
    public PhotonCamera getCamera() {
        return camera;
    }
    
    /**
     * 獲取當前目標類別(0或1)
     */
    public int getTargetClass() {
        return targetClass;
    }
    
    // ===================== 命令類 =====================
    
    /**
     * 對準目標的命令
     * 此命令將旋轉機器人直到它直接面對目標
     */
    private class AimAtTargetCommand extends Command {
        private final Drive driveSubsystem;
        
        /**
         * 創建一個新的AimAtTargetCommand
         * 
         * @param driveSubsystem 控制機器人移動的驅動子系統
         */
        public AimAtTargetCommand(Drive driveSubsystem) {
            this.driveSubsystem = driveSubsystem;
            
            // 此命令需要這兩個子系統
            addRequirements(ObjectDetection.this, driveSubsystem);
        }
        
        @Override
        public void initialize() {
            // 記錄命令開始時的日誌
            System.out.println("開始對準目標類別: " + getTargetClass());
        }
        
        @Override
        public void execute() {
            // 獲取視覺對準目標所需的轉向值
            double turnOutput = calculateAimOutput();
            
            // 將旋轉應用到驅動系統(無前進/後退運動)
            driveSubsystem.runVelocity(new ChassisSpeeds(0, 0, turnOutput));
        }
        
        @Override
        public boolean isFinished() {
            // 當我們對準目標時，命令完成
            return isAimedAtTarget();
        }
        
        @Override
        public void end(boolean interrupted) {
            // 停止驅動系統
            driveSubsystem.stop();
            isAiming = false;
            
            // 記錄結束原因
            if (interrupted) {
                System.out.println("對準被中斷");
            } else {
                System.out.println("對準成功完成");
            }
        }
    }
    
    /**
     * 跟隨目標的命令
     * 此命令既會對準目標，又會保持與其的指定距離
     */
    private class FollowTargetCommand extends Command {
        private final Drive driveSubsystem;
        
        /**
         * 創建一個新的FollowTargetCommand
         * 
         * @param driveSubsystem 控制機器人移動的驅動子系統
         */
        public FollowTargetCommand(Drive driveSubsystem) {
            this.driveSubsystem = driveSubsystem;
            
            // 此命令需要這兩個子系統
            addRequirements(ObjectDetection.this, driveSubsystem);
        }
        
        @Override
        public void initialize() {
            // 記錄命令開始時的日誌
            System.out.println("開始跟隨目標類別: " + getTargetClass());
        }
        
        @Override
        public void execute() {
            // 檢查目標是否可見
            if (!isTargetVisible()) {
                // 如果沒有可見目標，停止機器人
                driveSubsystem.stop();
                return;
            }
            
            // 獲取對準目標的轉向值
            double turnOutput = calculateAimOutput();
            
            // 獲取接近/維持與目標距離的驅動值
            double driveOutput = calculateDriveOutput();
            
            // 將旋轉和前進/後退運動應用到驅動系統
            driveSubsystem.runVelocity(new ChassisSpeeds(driveOutput, 0, turnOutput));
            
            // 更新儀表板上的狀態
            SmartDashboard.putNumber("驅動輸出", driveOutput);
            SmartDashboard.putNumber("轉向輸出", turnOutput);
        }
        
        @Override
        public boolean isFinished() {
            // 此命令永遠不會自行結束
            return false;
        }
        
        @Override
        public void end(boolean interrupted) {
            // 停止驅動系統
            driveSubsystem.stop();
            isFollowing = false;
            
            // 記錄結束原因
            System.out.println("目標跟隨結束");
        }
    }
}