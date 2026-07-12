package org.firstinspires.ftc.teamcode.Subsystems;


import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Robot {
    MecanumDrive drive;
    public ArtifactHandler artifactHandler;
    Telemetry telemetry;
    Gamepad gamepad1;
    Gamepad gamepad2;
    PinpointLocalizer pinpoint;
    Pose resetPose;
    Pose pose;
    boolean isAuto;
    boolean intakeToggle = false;
    boolean manualSort = false;
    boolean alliance;
    double[][][] autoEnableAimZones;
    double[][][] autoEnableAimZoneTargets;
    double[] primaryTargetPos;
    double[] secondaryTargetPos;
    int zone = 0;
    public Robot(MecanumDrive drive, ArtifactHandler artifactHandler, Gamepad gamepad1, Gamepad gamepad2, PinpointLocalizer pinpoint, Pose resetPose,double[][][] autoEnableAimZones, double[][][] autoEnableAimZoneTargets,double[] primaryTargetPos,double[] secondaryTargetPos,boolean alliance, boolean isAuto){
        this(drive, artifactHandler, gamepad1, gamepad2, pinpoint, resetPose, autoEnableAimZones, autoEnableAimZoneTargets, primaryTargetPos, secondaryTargetPos, alliance, isAuto, null);
    }
    public Robot(MecanumDrive drive, ArtifactHandler artifactHandler, Gamepad gamepad1, Gamepad gamepad2, PinpointLocalizer pinpoint, Pose resetPose,double[][][] autoEnableAimZones, double[][][] autoEnableAimZoneTargets,double[] primaryTargetPos,double[] secondaryTargetPos,boolean alliance, boolean isAuto, Telemetry telemetry){
        this.drive = drive;
        this.artifactHandler = artifactHandler;
        this.telemetry = telemetry;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.pinpoint = pinpoint;
        this.isAuto = isAuto;
        this.resetPose = resetPose;
        this.autoEnableAimZones = autoEnableAimZones;
        this.autoEnableAimZoneTargets = autoEnableAimZoneTargets;
        this.alliance = alliance;
        this.primaryTargetPos = primaryTargetPos;
        this.secondaryTargetPos = secondaryTargetPos;
    }
    public void update(){
        pose = pinpoint.getPose();
        if(!isAuto) {
            checkGamepadInput();
            if(!artifactHandler.shoot) autoEnableAim();
            else artifactHandler.enableAiming(true);
            drive.update(pose.getHeading());
        }
        artifactHandler.update(manualSort);
        updateTelemetry();
    }
    public void setTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;
    }
    private double sign(double x1,double y1,double x2,double y2,double x3,double y3){
        return (x1-x3)*(y2-y3)-(x2-x3)*(y1-y3);
    }
    private void autoEnableAim() {
        double robotX = pose.getX();
        double robotY = pose.getY();
        double d1,d2,d3;
        boolean has_neg, has_pos;
        for (int i = 0; i < autoEnableAimZones.length; i++) {
            d1 = sign(robotX, robotY, autoEnableAimZones[i][0][0], autoEnableAimZones[i][0][1], autoEnableAimZones[i][1][0], autoEnableAimZones[i][1][1]);
            d2 = sign(robotX, robotY, autoEnableAimZones[i][1][0], autoEnableAimZones[i][1][1], autoEnableAimZones[i][2][0], autoEnableAimZones[i][2][1]);
            d3 = sign(robotX, robotY, autoEnableAimZones[i][2][0], autoEnableAimZones[i][2][1], autoEnableAimZones[i][0][0], autoEnableAimZones[i][0][1]);
            has_neg = d1 < 0 || d2 < 0 || d3 < 0;
            has_pos = d1 > 0 || d2 > 0 || d3 > 0;
            if (has_neg && has_pos){
                artifactHandler.enableAiming(false);
            } else {
                zone = i + 1;
                artifactHandler.enableAiming(true);
                artifactHandler.setTargetX(autoEnableAimZoneTargets[i][alliance? 1 : 0][0]);
                artifactHandler.setTargetY(autoEnableAimZoneTargets[i][alliance? 1 : 0][1]);
                return;
            }
        }
        zone = 0;
    }
    public int getZone(){
        return zone;
    }
    private void checkGamepadInput(){
        if(gamepad1.right_trigger > 0.5){
            artifactHandler.setTargetX(primaryTargetPos[0]);
            artifactHandler.setTargetY(primaryTargetPos[1]);
            artifactHandler.requestShoot(true);
        } else if (gamepad1.left_trigger > 0.5){
            artifactHandler.setTargetX(secondaryTargetPos[0]);
            artifactHandler.setTargetY(secondaryTargetPos[1]);
            artifactHandler.requestShoot(true);
        } else {
            artifactHandler.requestShoot(false);
        }

        if(gamepad1.circleWasPressed()) {
            intakeToggle = !intakeToggle;
            artifactHandler.turnOnIntake(intakeToggle,false);
        }
        if(gamepad1.psWasPressed()) manualSort = !manualSort;
        if(gamepad1.triangleWasPressed()) artifactHandler.setNextColor(1);
        if(gamepad1.squareWasPressed()) artifactHandler.setNextColor(2);
        if(gamepad1.cross) artifactHandler.turnOnIntake(true,true);
        else artifactHandler.turnOnIntake(intakeToggle, false);
        if(gamepad1.optionsWasPressed() && gamepad1.shareWasPressed()) pinpoint.setPose(resetPose);
    }
    public void init(){
        if(!isAuto) drive.init();
        artifactHandler.init(isAuto);
    }
    private void updateTelemetry() {
        if(telemetry == null) return;
        telemetry.addLine("=== Robot ===");
        telemetry.addData("Robot Auto", isAuto);
        telemetry.addData("Robot Alliance", alliance ? "BLUE" : "RED");
        telemetry.addData("Robot Zone", zone);
        telemetry.addData("Robot Intake Toggle", intakeToggle);
        telemetry.addData("Robot Manual Sort", manualSort);
        telemetry.addData("Robot Pose", pose == null ? "null" : String.format("(%.2f, %.2f, %.2f deg)", pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading())));
        telemetry.addData("Robot Reset Pose", "(%.2f, %.2f, %.2f deg)", resetPose.getX(), resetPose.getY(), Math.toDegrees(resetPose.getHeading()));
        telemetry.addData("Robot Primary Target", "%.2f / %.2f", primaryTargetPos[0], primaryTargetPos[1]);
        telemetry.addData("Robot Secondary Target", "%.2f / %.2f", secondaryTargetPos[0], secondaryTargetPos[1]);
        drive.appendTelemetry(telemetry);
        artifactHandler.appendTelemetry(telemetry);
        telemetry.update();
    }
}
// WORK IN PROGRESS
