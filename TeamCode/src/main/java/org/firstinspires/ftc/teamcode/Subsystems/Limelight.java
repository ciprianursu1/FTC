package org.firstinspires.ftc.teamcode.Subsystems;

import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class Limelight {
    Limelight3A limelight;
    IMU imu;
    int tagID;
    Pose pose;
    double headingOffset;
    double lastTimestamp = 0;
    double turretYawOffset = 0;
    double turretOffsetX = RobotConfig.TURRET_OFFSET_X_METERS;
    double turretOffsetY = RobotConfig.TURRET_OFFSET_Y_METERS;
    double limelightOffsetX = RobotConfig.LIMELIGHT_OFFSET_X_METERS;
    double limelightOffsetY = RobotConfig.LIMELIGHT_OFFSET_Y_METERS;
    boolean motifMode = false;
    boolean freshData = false;
    public Limelight(Limelight3A limelight, IMU imu){
        this.limelight = limelight;
        this.imu = imu;
    }
    public void changePipeline(int pipeline,boolean isMotifOnly){
        limelight.pipelineSwitch(pipeline);
        motifMode = isMotifOnly;
    }
    public int getTagID(){
        return tagID;
    }
    public Pose getPose(){
        return pose;
    }
    public boolean isFreshData(){
        return freshData;
    }
    public void setTurretYawOffset(double turretYawOffset, AngleUnit angleUnit){
        if(angleUnit == AngleUnit.RADIANS) this.turretYawOffset = Math.toDegrees(turretYawOffset);
        else this.turretYawOffset = turretYawOffset;
    }
    public double getTurretYawOffset(){
        return turretYawOffset;
    }
    public void setPoseOffsets(double turretOffsetX, double turretOffsetY, double limelightOffsetX, double limelightOffsetY){
        this.turretOffsetX = turretOffsetX;
        this.turretOffsetY = turretOffsetY;
        this.limelightOffsetX = limelightOffsetX;
        this.limelightOffsetY = limelightOffsetY;
    }
    public void init(double startHeading,AngleUnit angleUnit){
        limelight.start();
        if(angleUnit == AngleUnit.RADIANS) headingOffset = Math.toDegrees(startHeading + Math.PI/2); // convert to FTC heading
        else headingOffset = startHeading + 90.0;
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.UP)));
        imu.resetYaw();
    }
    public void update(){
        freshData = false;
        limelight.updateRobotOrientation(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)+headingOffset+turretYawOffset);
        LLResult result = limelight.getLatestResult();
        if(result != null && result.isValid()) {
            double timestamp = result.getTimestamp();
            freshData = timestamp > lastTimestamp;
            lastTimestamp = timestamp;
            if (motifMode) {
                if (!result.getFiducialResults().isEmpty()) {
                    tagID = result.getFiducialResults().get(0).getFiducialId();
                }
            } else {
                pose = limelightToPedro(
                        result.getBotpose_MT2(),
                        turretYawOffset,
                        turretOffsetX,
                        turretOffsetY,
                        limelightOffsetX,
                        limelightOffsetY
                );
            }
        }
    }
    public static Pose limelightToPedro(Pose3D limelightPose3D) {
        return limelightToPedro(limelightPose3D, 0.0);
    }
    public static Pose limelightToPedro(Pose3D limelightPose3D, double turretYawOffset) {
        return limelightToPedro(
                limelightPose3D,
                turretYawOffset,
                0.0,
                0.0,
                0.0,
                0.0
        );
    }
    public static Pose limelightToPedro(
            Pose3D limelightPose3D,
            double turretYawOffset,
            double turretOffsetX,
            double turretOffsetY,
            double limelightOffsetX,
            double limelightOffsetY) {
        if (limelightPose3D == null) {
            return new Pose(0, 0, 0);
        }
        double xInches = limelightPose3D.getPosition().toUnit(DistanceUnit.INCH).x;
        double yInches = limelightPose3D.getPosition().toUnit(DistanceUnit.INCH).y;
        double headingRadians = limelightPose3D.getOrientation().getYaw(AngleUnit.RADIANS) - Math.toRadians(turretYawOffset);
        double turretYawRadians = Math.toRadians(turretYawOffset);
        double turretOffsetXInches = turretOffsetX * RobotConfig.INCHES_PER_METER;
        double turretOffsetYInches = turretOffsetY * RobotConfig.INCHES_PER_METER;
        double limelightOffsetXInches = limelightOffsetX * RobotConfig.INCHES_PER_METER;
        double limelightOffsetYInches = limelightOffsetY * RobotConfig.INCHES_PER_METER;
        xInches -= turretOffsetXInches * Math.cos(headingRadians) - turretOffsetYInches * Math.sin(headingRadians);
        yInches -= turretOffsetXInches * Math.sin(headingRadians) + turretOffsetYInches * Math.cos(headingRadians);
        xInches -= limelightOffsetXInches * Math.cos(headingRadians + turretYawRadians) - limelightOffsetYInches * Math.sin(headingRadians + turretYawRadians);
        yInches -= limelightOffsetXInches * Math.sin(headingRadians + turretYawRadians) + limelightOffsetYInches * Math.cos(headingRadians + turretYawRadians);
        Pose ftcCenterPose = new Pose(xInches, yInches, headingRadians, FTCCoordinates.INSTANCE);
        return ftcCenterPose.getAsCoordinateSystem(PedroCoordinates.INSTANCE);
    }

}
