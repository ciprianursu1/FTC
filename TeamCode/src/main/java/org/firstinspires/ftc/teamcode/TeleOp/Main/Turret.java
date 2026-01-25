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
    final double absoluteTurretHeight = 0.4; //meters
    final double absoluteTargetHeight = 0.9; //meters
    final double relativeHeight = absoluteTargetHeight - absoluteTurretHeight;
    final double prefferedMaxHeightThrow = 1.6; //meters (relative to turret height)
    final double kP = 0.015;
    final double launcherEfficiency = 0.5; // needs experimenting
    final double flywheelRadius = 0.048;
    final double g = 9.81;
    final double maxTurretAnglerPower = 0.2;
    final double trajectoryAngleModifierGearRatio = 4.0;
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
    Pose robotLocation = new Pose();
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
        if((targetTurretDeg < 0 && targetTurretDeg > turretLimitLeft) || (targetTurretDeg >= 0 && targetTurretDeg < turretLimitRight)){
            targetTurretDeg = startTurretAngle;
        }
        currentTurretDeg=normalizeAngle(currentTurretDeg);
        double error = normalizeAngle(targetTurretDeg - currentTurretDeg);

        double power = error * kP;
        power = Range.clip(power, -maxTurretAnglerPower, maxTurretAnglerPower);
        turretAngler.set(power);
    }
    private void computeParameters() {

        double d = Math.hypot(targetX - turretX, targetY - turretY);

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
                    (int)(60 * exitVelocity
                            / (2 * Math.PI * flywheelRadius * launcherEfficiency));
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
        updateTurretAim();
        updatePose();
        computeParameters();
        updateLauncher();
    }
}