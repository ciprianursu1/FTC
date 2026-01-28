package org.firstinspires.ftc.teamcode.TeleOp.Main;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Turret {
    MotorEx launcher;
    MotorEx turretAngler;
    Servo trajectoryAngleModifier;
    final double[][] launchZoneBig = {{-18,144},{162,144},{72,55}};
    final double[][] launchZoneSmall = {{40,0},{102,0},{72,32}};
    final double maxLauncherPower = 0.7;
    final double absoluteTurretHeight = 0.35; //meters
    final double absoluteTargetHeight = 1.1; //meters
    final double relativeHeight = absoluteTargetHeight - absoluteTurretHeight;
    final double prefferedMaxHeightThrow = relativeHeight + 0.3; //meters (relative to turret height)
    final double kP = 0.015;
    final double launcherEfficiency = 0.5; // needs experimenting
    final double flywheelRadius = 0.048;
    final double g = 9.81;
    final double maxTurretAnglerPower = 0.2;
    final double trajectoryAngleModifierGearRatio = 112.0/24.0;
    final double turretAnglerGearRatio = 76/24.0;
    final double turretAnglerDegPerTick = 360/(384.5*turretAnglerGearRatio);
    final double trajectoryAnglerMaxTravel = 300.0;
    final double trajectoryAnglePosPerDegree = trajectoryAngleModifierGearRatio/trajectoryAnglerMaxTravel;
    final double minAngle = 22.5;
    final double maxAngle = 45.0;
    final double startTurretAngle = -180.0;
    final double turretLimitLeft = -110;
    final double turretLimitRight = 110;
    final double turretOffsetX = 0.0;
    final double turretOffsetY = 0.0;
    final double launcherStartRPM = 3000.0;
    final int maxFlywheelRPM = 6000;
    Pose robotLocation = new Pose();
    boolean aimingEnabled = false;
    boolean launcherEnabled = false;
    double targetX = 0.0;
    double targetY = 0.0;
    double turretX = 0.0;
    double turretY = 0.0;
    double trajectoryAngle = 22.5;
    int flywheelTargetRPM = 0;
    PinpointLocalizer Pinpoint;
    public Turret(HardwareMap hwMap, String launcherName, String turretAnglerName, String trajectoryAngleModifierName, PinpointLocalizer pinpoint){
        launcher = new MotorEx(hwMap,launcherName, Motor.GoBILDA.BARE);
        turretAngler = new MotorEx(hwMap,turretAnglerName,Motor.GoBILDA.RPM_435);
        trajectoryAngleModifier = hwMap.get(Servo.class,trajectoryAngleModifierName);
        Pinpoint = pinpoint;
    }
    public void init(double targetX, double targetY){
        launcher.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        turretAngler.setZeroPowerBehavior((Motor.ZeroPowerBehavior.BRAKE));
        launcher.setRunMode(Motor.RunMode.VelocityControl);
        launcher.setVeloCoefficients(12.0,0.0,0.0);
        turretAngler.setRunMode(Motor.RunMode.RawPower);
        this.targetX = targetX;
        this.targetY = targetY;
    }
    private void setTrajectoryAngle(double angle){
        angle = Range.clip(angle,minAngle,maxAngle);
        double position = (angle - minAngle)*trajectoryAnglePosPerDegree;
        trajectoryAngleModifier.setPosition(position);
    }
    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }
    private void updateTurretAim() {

        double robotX = robotLocation.getX();
        double robotY = robotLocation.getY();
        double robotHeading = robotLocation.getHeading();
        double robotHeadingDeg = Math.toDegrees(robotHeading);
        turretX = robotX + turretOffsetX*Math.cos(robotHeading) - turretOffsetY*Math.sin(robotHeading);
        turretY = robotY + turretOffsetX*Math.sin(robotHeading) + turretOffsetY*Math.cos(robotHeading);
        double dx = targetX - turretX;
        double dy = targetY - turretY;

        double fieldAngle = Math.toDegrees(Math.atan2(dy, dx));

        double currentTurretDeg = turretAngler.getCurrentPosition() * turretAnglerDegPerTick + startTurretAngle;
        double targetTurretDeg = normalizeAngle(fieldAngle - robotHeadingDeg);
        currentTurretDeg=normalizeAngle(currentTurretDeg);
        if(targetTurretDeg < 0 && targetTurretDeg > turretLimitLeft){
            targetTurretDeg = turretLimitLeft;
        } else if (targetTurretDeg >= 0 && targetTurretDeg < turretLimitRight) {
            targetTurretDeg = turretLimitRight;
        }

        double error = normalizeAngle(targetTurretDeg - currentTurretDeg);

        double power = error * kP;
        power = Range.clip(power, -maxTurretAnglerPower, maxTurretAnglerPower);
        turretAngler.set(power);
    }
    private void computeParameters() {

        double d = Math.hypot(targetX - turretX, targetY - turretY) * 0.0254;

        if (d <= 0 || prefferedMaxHeightThrow <= 0) {
            return;
        }

        if (relativeHeight >= prefferedMaxHeightThrow) {
            return;
        }

        double k = (4 * prefferedMaxHeightThrow / d)
                * (1 - Math.sqrt(1 - relativeHeight / prefferedMaxHeightThrow));

        boolean constrained = (k > 2.414 || k < 1.0);

        if (constrained) {

            trajectoryAngle = (k > 2.414) ? 45.0 : 22.5;

            double tanTerm = Math.tan(Math.toRadians(90.0 - trajectoryAngle));

            double denom = (d / 2.0) * tanTerm - relativeHeight;

            if (denom <= 0) {
                return;
            }

            double maxHeightThrow =
                    (d * d * tanTerm * tanTerm)
                            / (16.0 * denom);

            if (maxHeightThrow <= 0) {
                return;
            }

            double sinTerm = Math.sin(Math.toRadians(90.0 - trajectoryAngle));

            if (Math.abs(sinTerm) < 1e-6) {
                return;
            }

            double exitVelocity =
                    Math.sqrt(2 * g * maxHeightThrow) / sinTerm;

            flywheelTargetRPM =
                    (int)(60 * exitVelocity
                            / (2 * Math.PI * flywheelRadius * launcherEfficiency));

        } else {

            trajectoryAngle = 90.0 - Math.toDegrees(Math.atan(k));

            double sinTerm = Math.sin(Math.toRadians(90.0 - trajectoryAngle));

            if (Math.abs(sinTerm) < 1e-6) {
                return;
            }

            double exitVelocity =
                    Math.sqrt(2 * g * prefferedMaxHeightThrow) / sinTerm;

            flywheelTargetRPM =
                    Math.min((int)(60 * exitVelocity
                            / (2 * Math.PI * flywheelRadius * launcherEfficiency)),maxFlywheelRPM);
        }
    }

    private void updateLauncher(){
        launcher.setVelocity(flywheelTargetRPM*360.0, AngleUnit.DEGREES);
    }
    private void updateTrajectoryAngle(){
        setTrajectoryAngle(trajectoryAngle);
    }
    private void updatePose(){
        robotLocation = Pinpoint.getPose();
    }
    public void update(){
        updatePose();
        disableIfNotInLaunchZone();
        if(!aimingEnabled) return;
        updateTurretAim();
        computeParameters();
        updateLauncher();
        updateTrajectoryAngle();

    }
    public void disableIfNotInLaunchZone(){
        double robotX = robotLocation.getX();
        double robotY = robotLocation.getY();
        double d1,d2,d3;
        boolean has_neg, has_pos;
        if(robotY > 48) {
            d1 = sign(robotX, robotY, launchZoneBig[0][0], launchZoneBig[0][1], launchZoneBig[1][0], launchZoneBig[1][1]);
            d2 = sign(robotX, robotY, launchZoneBig[1][0], launchZoneBig[1][1], launchZoneBig[2][0], launchZoneBig[2][1]);
            d3 = sign(robotX, robotY, launchZoneBig[2][0], launchZoneBig[2][1], launchZoneBig[0][0], launchZoneBig[0][1]);
        }else{
            d1 = sign(robotX, robotY, launchZoneSmall[0][0], launchZoneSmall[0][1], launchZoneSmall[1][0], launchZoneSmall[1][1]);
            d2 = sign(robotX, robotY, launchZoneSmall[1][0], launchZoneSmall[1][1], launchZoneSmall[2][0], launchZoneSmall[2][1]);
            d3 = sign(robotX, robotY, launchZoneSmall[2][0], launchZoneSmall[2][1], launchZoneSmall[0][0], launchZoneSmall[0][1]);
        }
        has_neg = d1 < 0 || d2 < 0 || d3 < 0;
        has_pos = d1 > 0 || d2 > 0 || d3 > 0;
        if (has_neg && has_pos){
            disableAiming();
        } else {
            if(!aimingEnabled) enableAiming();
        }
    }
    private double sign(double x1,double y1,double x2,double y2,double x3,double y3){
        return (x1-x3)*(y2-y3)-(x2-x3)*(y1-y3);
    }

    public void enableLauncher(){
        if(launcherEnabled) return;
        launcherEnabled = true;
        launcher.set(maxLauncherPower);
        launcher.setVelocity(launcherStartRPM*360.0, AngleUnit.DEGREES);
        flywheelTargetRPM = 3000;
    }
    public void disableLauncher(){
        if(!launcherEnabled) return;
        launcherEnabled = false;
        launcher.set(0);
        launcher.setVelocity(0,AngleUnit.DEGREES);
        flywheelTargetRPM = 0;
    }
    public void enableAiming(){
        aimingEnabled = true;
    }
    public void disableAiming(){
        aimingEnabled = false;
    }
}