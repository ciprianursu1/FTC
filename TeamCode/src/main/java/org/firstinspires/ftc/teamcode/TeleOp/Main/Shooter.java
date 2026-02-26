package org.firstinspires.ftc.teamcode.TeleOp.Main;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Shooter {
    DcMotorEx flywheel;
    DcMotorEx turret;
    Servo trajectoryAngleModifier;
    Pose pose;
    Pose velocity;
    Telemetry telemetry;
    final double maxTrajectoryAngle = 70;
    final double minTrajectoryAngle = 50;
    static final double FLYWHEEL_TICKS_PER_REV = 28*1.4;
    double targetX = 0;
    double targetY = 144;
    double minFlywheelRPM = 1000/1.4;
    double maxFlywheelRPM = 6000/1.4;
    final double trajectoryAngleModifierGearRatio = 127/15.0;
    final double trajectoryAnglerMaxTravel = 300.0;
    final double trajectoryAnglePosPerDegree = trajectoryAngleModifierGearRatio/trajectoryAnglerMaxTravel;
    static final double g = 9.81;
    static final double flywheelRadius = 0.048;
    static final double launcherEfficiency = 0.43;
    public static final double turretOffsetX = 0.0;
    public static final double turretOffsetY = 52/1000.0;
    final double[][] launchZoneBig = {{-18,144},{162,144},{72,55}};
    final double[][] launchZoneSmall = {{40,0},{102,0},{72,40}};
    private boolean aimingEnabled = false;
    private boolean launcherEnabled = false;
    final double absoluteTurretHeight = 0.25; //meters
    final double absoluteTargetHeight = 0.9; //meters
    final double relativeHeight = absoluteTargetHeight - absoluteTurretHeight;
    final double kP = 0.015;
    final double kF = 0.025;
    int zone = 0;
    final double MAX_POWER_TURRET = 0.4;
    final double startTurretAngle = -180.0;
    final double LEFT_LIMIT = -110;
    final double RIGHT_LIMIT = 110;
    private static final double MOTOR_TICKS_PER_REV = 384.5;
    private static final double MOTOR_TO_TURRET_RATIO = 75.0 / 26.0;

    private static final double TURRET_DEG_PER_TICK =
            360.0 / (MOTOR_TICKS_PER_REV * MOTOR_TO_TURRET_RATIO);
    double turretX = 0.0;
    double turretY = 0.0;
    double robotX = 0.0;
    double robotY = 0.0;
    double robotHeading = 0.0;
    double flywheelTargetRPM = 0;
    boolean rpmProducesValidTrajectory = false;
    int currentTurretTicks = 0;
    double currentTurretDeg = 0;
    double targetTurretDeg = 0;
    double rpm = 0.0;
    double v0 = 0;
    double trajectoryAngle = 70;
    private static final double RPM_TOL = 40;
//    private static final long RPM_STABLE_MS = 80;
    private static final double TURRET_TOL = 0.5;
//    private long rpmInRangeSinceMs = 0;
    boolean turretOnTarget = false;
    boolean forceEnableLauncher = false;
    double angleOffset = 0;
    double[] velocityCompensation = new double[2];
    public Shooter(HardwareMap hwMap, String flywheelName, String turretName, String servoName){
        flywheel = hwMap.get(DcMotorEx.class,flywheelName);
        turret = hwMap.get(DcMotorEx.class,turretName);
        trajectoryAngleModifier = hwMap.get(Servo.class,servoName);
    }
    public void setTurretAngleOffset(double offset){
        angleOffset = offset;
    }
    public void init(Telemetry telemetry,boolean isAuto) {
        if(isAuto){
            flywheel.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        }
        flywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(30, 0, 0, 14.0));
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        trajectoryAngleModifier.setPosition(0);
        this.telemetry = telemetry;
    }
    public void update(Pose pose,Pose velocity, double targetX, double targetY, boolean isShooting){
        this.targetX = targetX;
        this.targetY = targetY;
        this.pose = pose;
        this.velocity = velocity;
        //=== INPUT ===
        updateTurretInputs();
        updateRobotPosition();
        //=== COMPUTE ===
        disableIfNotInLaunchZone();
        updateVelocityCompensation();
        computeParameters();
        //=== OUTPUT ===
        updateTurretAim();
        updateTrajectoryAngle();
        updateFlywheel();
        //=== TELEMETRY ===
        updateTelemetry();
    }
    private void updateVelocityCompensation() {
        v0 = (rpm / 60.0) * 2 * Math.PI * flywheelRadius * launcherEfficiency;
        velocityCompensation = computeVelocityCompensation();
        v0 += velocityCompensation[1];
    }
    private void updateTurretInputs() {
        currentTurretTicks = turret.getCurrentPosition();
        currentTurretDeg = normalizeAngle(currentTurretTicks * TURRET_DEG_PER_TICK + startTurretAngle + angleOffset);
        rpm = flywheel.getVelocity() / FLYWHEEL_TICKS_PER_REV * 60.0;
    }

    private void updateRobotPosition(){
        robotX = pose.getX();
        robotY = pose.getY();
        robotHeading = pose.getHeading();
        turretX = robotX + turretOffsetX * Math.cos(robotHeading) - turretOffsetY * Math.sin(robotHeading);
        turretY = robotY + turretOffsetX * Math.sin(robotHeading) + turretOffsetY * Math.cos(robotHeading);
    }
    private void updateTrajectoryAngle(){
        setTrajectoryAngle(trajectoryAngle);
    }
    private void setTrajectoryAngle(double angle){
        angle = Range.clip(angle,minTrajectoryAngle,maxTrajectoryAngle);
        double position = (angle - minTrajectoryAngle)*trajectoryAnglePosPerDegree;
        position = Range.clip(position,0,1);
        trajectoryAngleModifier.setPosition(position);
    }
    public boolean isRPMInRange(){
        return Math.abs(flywheelTargetRPM - rpm) <= RPM_TOL || rpmProducesValidTrajectory;
    }
    public boolean isTurretOnTarget(){
        return turretOnTarget;
    }
//    private boolean rpmInRangeStable() {
//        boolean inRange = (rpm >= (flywheelTargetRPM - RPM_TOL)) && (rpm <= (flywheelTargetRPM + RPM_TOL));
//        long now = System.currentTimeMillis();
//
//        if (!inRange) {
//            rpmInRangeSinceMs = 0;
//            return false;
//        }
//        if (rpmInRangeSinceMs == 0) rpmInRangeSinceMs = now;
//        return (now - rpmInRangeSinceMs) >= RPM_STABLE_MS;
//    }
    private void disableIfNotInLaunchZone(){
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
            aimingEnabled = false;
            zone = 0;
        } else {
            if (robotY > 48) {
                zone = 1;
            } else {
                zone = 2;
            }
            aimingEnabled = true;
        }
    }

    private double sign(double x1,double y1,double x2,double y2,double x3,double y3){
        return (x1-x3)*(y2-y3)-(x2-x3)*(y1-y3);
    }
    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }
    double[] computeVelocityCompensation() {

        // Robot velocity in field frame
        double vRobot = Math.hypot(velocity.getX(), velocity.getY());
        if (vRobot < 0.05) return new double[]{0, 0};

        double turretHeading = Math.toRadians(normalizeAngle(Math.toDegrees(pose.getHeading()) + currentTurretDeg));
        double thetaRad = Math.toRadians(trajectoryAngle);

        double vXY = v0 * Math.cos(thetaRad);
        double vZ  = v0 * Math.sin(thetaRad);

        double vBallX = vXY * Math.cos(turretHeading);
        double vBallY = vXY * Math.sin(turretHeading);

        double vCompX = vBallX + velocity.getX();
        double vCompY = vBallY + velocity.getY();

        double vXYComp = Math.hypot(vCompX, vCompY);
        double newYaw =
                Math.atan2(vCompY, vCompX);
        double yawCorrection =
                normalizeAngle(Math.toDegrees(newYaw - turretHeading));
        yawCorrection = Range.clip(yawCorrection, -6.0, 6.0);
        double newV0 = Math.hypot(vZ, vXYComp);

        return new double[] { yawCorrection, newV0 - v0 };
    }


    private void computeParameters() {
        rpmProducesValidTrajectory = false;
        if(!aimingEnabled) return;
        if(zone == 1){
            flywheelTargetRPM = 2150;
            maxFlywheelRPM = 2500;
            trajectoryAngle = maxTrajectoryAngle;
        } else if(zone == 2){
            maxFlywheelRPM = 3150;
            flywheelTargetRPM = 3100;
            trajectoryAngle = maxTrajectoryAngle;
        } else {
            flywheelTargetRPM = 1000;
            maxFlywheelRPM = minFlywheelRPM;
            trajectoryAngle = maxTrajectoryAngle;
            return;
        }
        double d = Math.hypot(targetX - turretX, targetY - turretY) * 0.0254;
        if (d <= 0) {
            return;
        }
        if (v0 > 1.0) {
            double a = g * d * d / (2 * v0 * v0);
            double b = -d;
            double c = relativeHeight + a;
            double discriminant = b * b - 4 * a * c;
            if (discriminant >= 0) {
                double tanTheta1 = (-b + Math.sqrt(discriminant)) / (2 * a);
                double tanTheta2 = (-b - Math.sqrt(discriminant)) / (2 * a);
                double angle1 = Math.toDegrees(Math.atan(tanTheta1));
                double angle2 = Math.toDegrees(Math.atan(tanTheta2));
                double angleHigh = Math.max(angle1, angle2);
                double angleLow  = Math.min(angle1, angle2);

                boolean highValid = angleHigh >= minTrajectoryAngle && angleHigh <= maxTrajectoryAngle;
                boolean lowValid  = angleLow  >= minTrajectoryAngle && angleLow  <= maxTrajectoryAngle;
                if(highValid || lowValid) {
                    if(lowValid){
                        trajectoryAngle = angleLow;
                    } else {
                        trajectoryAngle = angleHigh;
                    }
                    rpmProducesValidTrajectory = true;
                }
            }
        }
    }
private void updateTurretAim() {
    if(aimingEnabled) {
        double dx = targetX - turretX;
        double dy = targetY - turretY;
        double fieldAngle = Math.atan2(dy, dx);
        targetTurretDeg = normalizeAngle(Math.toDegrees(fieldAngle - robotHeading));
    } else {
        targetTurretDeg = startTurretAngle;
    }
    if(targetTurretDeg < 0 && targetTurretDeg > LEFT_LIMIT){ // intentional inversion, turret is rear-mounted
        targetTurretDeg = LEFT_LIMIT;
    } else if (targetTurretDeg >= 0 && targetTurretDeg < RIGHT_LIMIT) {
        targetTurretDeg = RIGHT_LIMIT;
    }
    double error = normalizeAngle(targetTurretDeg - currentTurretDeg + velocityCompensation[0]);
    double power = error * kP;
    turretOnTarget = Math.abs(error) <= TURRET_TOL;
    power += Math.signum(error) * kF;
    power = Range.clip(power, -MAX_POWER_TURRET, MAX_POWER_TURRET);
    turret.setPower(power);
}
    public double getCurrentTurretDeg(){
        return currentTurretDeg;
    }
    public void enableLauncher(){
        if(forceEnableLauncher) return;
        forceEnableLauncher = true;
        launcherEnabled = true;
    }

    public void disableLauncher(){
        if(!forceEnableLauncher) return;
        forceEnableLauncher = false;
        launcherEnabled = false;
    }
    public static double kP_v = 30.0;     // try 20–35
    public static double kI_v = 0.0;      // keep 0 for fastest transient
    public static double kD_v = 0.0;      // add small later only if it overshoots/oscillates
    public final double kF_v = 14;
    private void updateFlywheel() {
        if(rpm > flywheelTargetRPM) {
            flywheel.setPower(0);
        } else if (Math.abs(rpm - flywheelTargetRPM) < RPM_TOL) {
            double target = flywheelTargetRPM;
            double targetTPS = target * FLYWHEEL_TICKS_PER_REV / 60.0;
            flywheel.setVelocityPIDFCoefficients(kP_v, kI_v, kD_v, kF_v);
            flywheel.setVelocity(targetTPS);
        } else if (rpm < flywheelTargetRPM){ // bang bang control works best for speed, in testing we observed no oscillations
            flywheel.setPower(1);
        } else {
            flywheel.setPower(0);
        }
    }
    private void updateTelemetry() {
        if (telemetry == null) return;

        telemetry.addLine("=== Shooter ===");

        // --- Flywheel ---
        telemetry.addData("Flywheel RPM", "%.0f / %.0f", rpm, flywheelTargetRPM);
        telemetry.addData("RPM In Range", Math.abs(flywheelTargetRPM - rpm) <= RPM_TOL);

        // --- Trajectory ---
        telemetry.addData("Trajectory Angle (deg)", "%.1f", trajectoryAngle);
        telemetry.addData("Launch Enabled", launcherEnabled);
        telemetry.addData("Valid Trajectory", rpmProducesValidTrajectory);

        // --- Turret ---
        telemetry.addData("Turret Angle (deg)", "%.1f", currentTurretDeg);
        telemetry.addData("Target Angle (deg)", targetTurretDeg);
        telemetry.addData("Turret On Target", turretOnTarget);
        telemetry.addData("Yaw Comp (deg)", velocityCompensation[0]);

        // --- Target ---
        telemetry.addData("Target X/Y", "%.1f , %.1f", targetX, targetY);

        // --- Robot ---
        telemetry.addData("Robot X/Y", "%.1f , %.1f", pose.getX(), pose.getY());
        telemetry.addData("Robot Heading (deg)", "%.1f", Math.toDegrees(pose.getHeading()));
        telemetry.addData("Robot Vel X/Y", "%.2f , %.2f", velocity.getX(), velocity.getY());

    }

}
