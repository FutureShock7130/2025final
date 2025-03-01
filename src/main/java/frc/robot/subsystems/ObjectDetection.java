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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.subsystems.drive.Drive;
import java.util.List;
import java.util.Map;
import java.util.HashMap;
import java.util.Optional;
import java.util.stream.Collectors;
import java.util.Comparator;
import java.util.Iterator;

/**
 * 物件偵測子系統
 * 這個子系統使用PhotonVision來偵測分類為"0"(algae)或"1"(coral)的物件
 * 提供以下功能:
 * - 透過Shuffleboard控制追蹤功能
 * - 追蹤並跟隨偵測到的物件
 * - 自動對準物件
 * - 追蹤最近的物件
 * - 估算物件距離
 * - 計算物件在場地上的位置
 */
public class ObjectDetection extends SubsystemBase {
    private final PhotonCamera camera;
    private int targetClass = 0; // 預設追蹤class 0 (algae)
    private final PIDController turnController;
    private final PIDController driveController;
    private Drive driveSubsystem;
    
    // 遊戲物體尺寸定義 (單位: 公尺)
    private static final double ALGAE_DIAMETER = 0.413; // algae的直徑
    private static final double CORAL_LENGTH = 0.30;   // coral的長度
    
    // 相機設定
    private static final double CAMERA_HORIZONTAL_FOV = 70.0; // 相機水平視角
    private static final double CAMERA_RESOLUTION_WIDTH = 640.0; // 相機解析度寬度
    
    // 命令狀態
    private boolean isAiming = false;
    private boolean isFollowing = false;
    private Command activeCommand = null;
    
    // 物體追蹤系統
    private Map<Integer, TrackedObject> trackedObjects = new HashMap<>();
    private Field2d fieldWidget = new Field2d();
    
    // Shuffleboard 項目
    private final ShuffleboardTab visionTab = Shuffleboard.getTab("物件偵測");
    private GenericEntry targetClassEntry;
    private GenericEntry targetAreaEntry;
    private GenericEntry aimToleranceEntry;
    private GenericEntry aimButtonEntry;
    private GenericEntry followButtonEntry;
    private GenericEntry stopButtonEntry;
    private GenericEntry distanceEstimateEntry;
    
    private static final double TARGET_AREA_SETPOINT = 15.0; 
    private static final double AIM_TOLERANCE_DEGREES = 2.0; 
    
    /**
     * 被追蹤物體的資訊
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
            // 如果超過3秒沒更新，視為過期
            return Timer.getFPGATimestamp() - lastUpdated > 3.0;
        }
    }
    
    public ObjectDetection() {
        camera = new PhotonCamera("WEB_CAM");
        turnController = new PIDController(0.05, 0, 0.005); //轉向PID
        driveController = new PIDController(0.1, 0, 0); //靠近PID
        setupShuffleboardControls();
    }
    
    /**
     * 設置Shuffleboard控制項
     */
    private void setupShuffleboardControls() {
        targetClassEntry = visionTab.add("target (0=algae, 1=coral)", targetClass)
            .withPosition(0, 0)
            .withSize(1, 1)
            .getEntry();
        
        targetAreaEntry = visionTab.add("target area", TARGET_AREA_SETPOINT)
            .withPosition(1, 0)
            .withSize(1, 1)
            .getEntry();
        
        aimToleranceEntry = visionTab.add("tolerance degree", AIM_TOLERANCE_DEGREES)
            .withPosition(2, 0)
            .withSize(1, 1)
            .getEntry();
        
        aimButtonEntry = visionTab.add("aim target", false)
            .withPosition(0, 1)
            .withSize(1, 1)
            .withProperties(Map.of("colorWhenTrue", "green"))
            .getEntry();
        
        followButtonEntry = visionTab.add("follow target", false)
            .withPosition(1, 1)
            .withSize(1, 1)
            .withProperties(Map.of("colorWhenTrue", "blue"))
            .getEntry();
        
        stopButtonEntry = visionTab.add("stop", false)
            .withPosition(2, 1)
            .withSize(1, 1)
            .withProperties(Map.of("colorWhenTrue", "red"))
            .getEntry();
        
        distanceEstimateEntry = visionTab.add("estimate distance", 0.0)
            .withPosition(3, 0)
            .withSize(1, 1)
            .getEntry();
        
        visionTab.addBoolean("has target", this::isTargetVisible)
            .withPosition(0, 2)
            .withSize(1, 1);
        
        visionTab.addBoolean("is aimed", this::isAimedAtTarget)
            .withPosition(1, 2)
            .withSize(1, 1);
        
        visionTab.addString("state", () -> {
            if (isFollowing) return "following";
            if (isAiming) return "aiming";
            return "super idle";
        })
            .withPosition(2, 2)
            .withSize(1, 1);
            
        visionTab.addString("target type", () -> 
            targetClass == 0 ? "algae" : "coral")
            .withPosition(3, 1)
            .withSize(1, 1);
            
        // 添加視覺化場地
        visionTab.add("feild", fieldWidget)
            .withSize(5, 3)
            .withPosition(0, 3);
            
        // 顯示追蹤物體清單
        visionTab.addString("traced object", this::getTrackedObjectsAsString)
            .withSize(2, 3)
            .withPosition(5, 3);
    }
    
    /**
     * 將追蹤的物體轉換為字串顯示
     */
    private String getTrackedObjectsAsString() {
        if (trackedObjects.isEmpty()) {
            return "no object";
        }
        
        return trackedObjects.values().stream()
            .filter(obj -> !obj.isStale())
            .map(obj -> String.format("ID:%d %s @ (%.2f, %.2f) 距離:%.2f",
                obj.id,
                obj.id == 0 ? "algae" : "coral",
                obj.pose.getX(),
                obj.pose.getY(),
                obj.distance))
            .collect(Collectors.joining("\n"));
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
        
        // 更新被追蹤的物體
        if (result.hasTargets()) {
            updateTrackedObjects(result);
        }
        
        // 清除過期的物體追蹤
        removeStaleObjects();
        
        // 更新場地視覺化
        updateFieldWidget();
        
        // 檢查用戶是否從Shuffleboard更新了設定
        checkShuffleboardControls();
    }
    
    /**
     * 清除過期的物體追蹤
     */
    private void removeStaleObjects() {
        // 使用迭代器安全地移除過期項目
        Iterator<Map.Entry<Integer, TrackedObject>> it = trackedObjects.entrySet().iterator();
        while (it.hasNext()) {
            if (it.next().getValue().isStale()) {
                it.remove();
            }
        }
    }
    
    /**
     * 更新場地視覺化
     */
    private void updateFieldWidget() {
        // 更新機器人位置
        if (driveSubsystem != null) {
            fieldWidget.setRobotPose(driveSubsystem.getPose());
        }
        
        // 更新物體位置
        trackedObjects.forEach((id, obj) -> {
            if (!obj.isStale()) {
                String objectType = id == 0 ? "algae" : "coral";
                fieldWidget.getObject(objectType + "-" + id).setPose(obj.pose);
            }
        });
    }
    
    /**
     * 計算物體的距離
     * @param target 追蹤目標
     * @return 距離（公尺）
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
            // 如果寬度無法獲取，使用面積作為備用方法
            return estimateDistanceFromArea(target.getArea());
        }
        
        // 根據物體ID判定實際尺寸
        double actualWidth;
        if (target.getFiducialId() == 0) {
            actualWidth = ALGAE_DIAMETER; // algae的直徑
        } else {
            actualWidth = CORAL_LENGTH; // coral的長度
        }
        
        // 使用視角公式計算距離
        return (actualWidth * CAMERA_RESOLUTION_WIDTH) / 
               (2 * apparentWidth * Math.tan(Math.toRadians(CAMERA_HORIZONTAL_FOV/2)));
    }
    
    /**
     * 從物體面積估計距離（主要方法）
     * @param area 物體面積（0-100的百分比）
     * @return 估計距離（公尺）
     */
    private double estimateDistanceFromArea(double area) {
        // 根據物體類別選擇適當的校準因子
        double scaleFactor;
        if (targetClass == 0) { // algae
            scaleFactor = 0.35; // 要通過實驗校準
        } else { // coral
            scaleFactor = 0.40; // 要通過實驗校準
        }
        
        // 面積是百分比（0-100），面積越大，物體越近
        if (area < 0.1) {
            // 避免除以極小值
            return 10.0; // 最大距離限制（10米）
        }
        
        // 實驗表明，距離與面積平方根的倒數大致成正比
        // Distance ≈ scaleFactor / √(Area)
        return scaleFactor / Math.sqrt(area / 100.0);
    }
    
    /**
     * 更新追蹤的物體
     */
    private void updateTrackedObjects(PhotonPipelineResult result) {
        if (!result.hasTargets() || driveSubsystem == null) {
            return;
        }
        
        for (PhotonTrackedTarget target : result.getTargets()) {
            int objectId = target.getFiducialId();
            if (objectId != 0 && objectId != 1) continue; // 只處理ID為0或1的物體
            
            // 計算物體距離
            double distance = calculateDistance(target);
            
            // 計算相對於機器人的位置
            double yaw = Math.toRadians(target.getYaw());
            
            // 相對位置向量 (前方為x軸正向)
            Translation2d objectRelativePos = new Translation2d(
                distance * Math.cos(yaw),
                distance * Math.sin(yaw)
            );
            
            // 轉換到場地坐標系
            Pose2d robotPose = driveSubsystem.getPose();
            double robotHeading = robotPose.getRotation().getRadians();
            
            Pose2d objectPose = new Pose2d(
                robotPose.getX() + objectRelativePos.getX() * Math.cos(robotHeading) - 
                            objectRelativePos.getY() * Math.sin(robotHeading),
                robotPose.getY() + objectRelativePos.getX() * Math.sin(robotHeading) + 
                            objectRelativePos.getY() * Math.cos(robotHeading),
                new Rotation2d(0)
            );
            
            // 更新物體位置
            trackedObjects.put(objectId, new TrackedObject(objectId, objectPose, distance));
            
            // 如果是當前目標類別，更新到Shuffleboard
            if (objectId == targetClass) {
                distanceEstimateEntry.setDouble(distance);
            }
        }
    }
    
    /**
     * 檢查用戶是否從Shuffleboard更新了設定
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
     * 設置要追蹤的類別(0=algae, 1=coral)
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
        return driveController.calculate(area, targetArea);
    }
    
    /**
     * 檢查是否可以看到具有指定類別的目標
     */
    public boolean isTargetVisible() {
        var result = camera.getLatestResult();
        return result.hasTargets() && getBestTarget(result) != null;
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
     * 獲取當前目標類別(0=algae, 1=coral)
     */
    public int getTargetClass() {
        return targetClass;
    }
    
    /**
     * 獲取目標物體的估計距離（米）
     * @return 距離，如果沒有可見目標則返回-1
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
     * 獲取特定類別的物體位置（如果可見）
     * @param classId 物體類別ID (0=algae, 1=coral)
     * @return 物體位置的Optional
     */
    public Optional<Pose2d> getObjectPosition(int classId) {
        TrackedObject obj = trackedObjects.get(classId);
        if (obj != null && !obj.isStale()) {
            return Optional.of(obj.pose);
        }
        return Optional.empty();
    }
    
    /**
     * 獲取最近的物體
     * @return 包含物體ID和位置的Optional
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
         * @param driveSubsystem 
         */
        public FollowTargetCommand(Drive driveSubsystem) {
            this.driveSubsystem = driveSubsystem;
            

            addRequirements(ObjectDetection.this, driveSubsystem);
        }
        
        @Override
        public void execute() {
            if (!isTargetVisible()) {
                driveSubsystem.stop();
                return;
            }
            

            double turnOutput = calculateAimOutput();
            

            double driveOutput = calculateDriveOutput();
            

            driveSubsystem.runVelocity(new ChassisSpeeds(driveOutput, 0, turnOutput));
            

            SmartDashboard.putNumber("驅動輸出", driveOutput);
            SmartDashboard.putNumber("轉向輸出", turnOutput);
        }
        
        @Override
        public boolean isFinished() {
            return false;
        }
        
        @Override
        public void end(boolean interrupted) {
            driveSubsystem.stop();
            isFollowing = false;
        }
    }
}