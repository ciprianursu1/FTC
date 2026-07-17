package org.firstinspires.ftc.teamcode.TeleOp.Main;

import com.pedropathing.geometry.Pose;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.RobotConfig;

public class Shooter {
    public static final double targetXSecondary = 72;
    public static final double targetYSecondary = 0;
    public static final double targetXBlue = 0;
    public static final double targetXRed = 144;
    public static final double targetYPrimary = 288;
    DcMotorEx flywheel;
    DcMotorEx turret;
    Servo trajectoryAngleModifier;
    Pose pose;
    Pose velocity;
    Telemetry telemetry;
    final double maxTrajectoryAngle = 70;
    final double minTrajectoryAngle = 50;
    static final double FLYWHEEL_TICKS_PER_REV = 28;
    double targetX = 0;
    double targetY = 144;
    double minFlywheelRPM = 1000/1.4;
    final double trajectoryAngleModifierGearRatio = 127/15.0;
    final double trajectoryAnglerMaxTravel = 300.0;
    final double trajectoryAnglePosPerDegree = trajectoryAngleModifierGearRatio/trajectoryAnglerMaxTravel;
    static final double g = 9.81;
    static final double flywheelRadius = 0.048;
    static final double launcherEfficiency = 0.43;
    public static final double turretOffsetX = 0.0;
    public static final double turretOffsetY = 52/1000.0;
    final double[][] launchZoneBig = {{-4,290},{148,290},{72,212}};
    final double[][] launchZoneSmall = {{32,76},{112,76},{72,0}};
    private boolean aimingEnabled = false;
    private boolean launcherEnabled = false;
    final double targetZ = RobotConfig.TARGET_Z;
    public static double turretMaxPower = 0.6;
    final double startTurretAngle = -180.0;
    public static double turretCcwLimitDeg = 270.0;
    public static double turretCwLimitDeg = 120.0;
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
    double currentTurretTravelDeg = 0;
    double currentTurretDeg = 0;
    double targetTurretTravelDeg = 0;
    double targetTurretDeg = 0;
    double rpm = 0.0;
    double v0 = 0;
    double trajectoryAngle = 70;
    double ballisticDistance = 0;
    double targetLaunchSpeed = 0;
    private static final double RPM_TOL = 40;
//    private static final long RPM_STABLE_MS = 80;
    public static double turretKp = 0.01;
    public static double turretKi = 0.000075;
    public static double turretKd = 0.00015;
    public static double turretKf = 0.008;
    public static double turretTolerance = 0.75;
    public static double turretIntegralMax = 5;
    public static double turretIntegralMin = -5;
    public static boolean turretResetIntegralOnSignChange = true;
//    private long rpmInRangeSinceMs = 0;
    boolean turretOnTarget = false;
    boolean forceEnableLauncher = false;
    double angleOffset = 0;
    double[] velocityCompensation = new double[2];
    double turretError = 0.0;
    double turretPower = 0.0;
    double turretErrorSum = 0.0;
    double turretLastError = 0.0;
    double turretLastMeasurement = 0.0;
    double turretLastDt = 0.0;
    double turretDerivative = 0.0;
    String turretIntegralState = "reset";
    boolean turretPidFirstRun = true;
    boolean turretTargetLimited = false;
    long turretLastUpdateNanos = 0;
    public Shooter(HardwareMap hwMap, String flywheelName, String turretName, String servoName){
        flywheel = hwMap.get(DcMotorEx.class,flywheelName);
        turret = hwMap.get(DcMotorEx.class,turretName);
        trajectoryAngleModifier = hwMap.get(Servo.class,servoName);
    }
    public void setTurretAngleOffset(double offset){
        if(angleOffset != offset) {
            turretErrorSum = 0.0;
            turretLastError = 0.0;
            turretLastMeasurement = 0.0;
            turretLastDt = 0.0;
            turretDerivative = 0.0;
            turretIntegralState = "angle offset reset";
            turretPidFirstRun = true;
            turretLastUpdateNanos = 0;
        }
        angleOffset = offset;
    }
    public void init(Telemetry telemetry,boolean isAuto) {
        if(isAuto){
            flywheel.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        }
        flywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(kP_v,kI_v,kD_v,kF_v));
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretErrorSum = 0.0;
        turretLastError = 0.0;
        turretLastMeasurement = 0.0;
        turretLastDt = 0.0;
        turretDerivative = 0.0;
        turretIntegralState = "init reset";
        turretPidFirstRun = true;
        turretLastUpdateNanos = 0;
        trajectoryAngleModifier.setPosition(0);
        this.telemetry = telemetry;
    }
    public void update(Pose pose,Pose velocity, double targetX, double targetY, boolean shootRequested){
        this.targetX = targetX;
        this.targetY = targetY;
        this.pose = pose;
        this.velocity = velocity;
        //=== INPUT ===
        updateTurretInputs();
        updateRobotPosition();
        //=== COMPUTE ===
        disableIfNotInLaunchZone(shootRequested);
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
        currentTurretTravelDeg = currentTurretTicks * TURRET_DEG_PER_TICK;
        currentTurretDeg = normalizeAngle(currentTurretTravelDeg + startTurretAngle + angleOffset);
        rpm = flywheel.getVelocity() / FLYWHEEL_TICKS_PER_REV * 60.0;
    }

    private void updateRobotPosition(){
        robotX = pose.getX();
        robotY = pose.getY();
        robotHeading = pose.getHeading();
        turretX = robotX + (turretOffsetX * Math.cos(robotHeading) - turretOffsetY * Math.sin(robotHeading)) * RobotConfig.INCHES_PER_METER;
        turretY = robotY + (turretOffsetX * Math.sin(robotHeading) + turretOffsetY * Math.cos(robotHeading)) * RobotConfig.INCHES_PER_METER;
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
    private void disableIfNotInLaunchZone(boolean targetOverride){
        for (int i = 0; i < RobotConfig.autoAimZones.length; i++) {
            if (isInLaunchZone(RobotConfig.autoAimZones[i])) {
                aimingEnabled = true;
                if (!targetOverride) {
                    double[] zoneTarget = selectLaunchZoneTarget(RobotConfig.autoAimZoneTargets[i]);
                    targetX = zoneTarget[0];
                    targetY = zoneTarget[1];
                }
                return;
            }
        }
        aimingEnabled = targetOverride;
    }
    private boolean isInLaunchZone(double[][] launchZone) {
        double d1 = sign(robotX, robotY, launchZone[0][0], launchZone[0][1], launchZone[1][0], launchZone[1][1]);
        double d2 = sign(robotX, robotY, launchZone[1][0], launchZone[1][1], launchZone[2][0], launchZone[2][1]);
        double d3 = sign(robotX, robotY, launchZone[2][0], launchZone[2][1], launchZone[0][0], launchZone[0][1]);
        boolean has_neg = d1 < 0 || d2 < 0 || d3 < 0;
        boolean has_pos = d1 > 0 || d2 > 0 || d3 > 0;
        return !(has_neg && has_pos);
    }
    private double[] selectLaunchZoneTarget(double[][] zoneTargets) {
        if (zoneTargets.length == 1) {
            return zoneTargets[0];
        }

        double[] bestTarget = zoneTargets[0];
        double bestDistance = Double.MAX_VALUE;
        for (double[] zoneTarget : zoneTargets) {
            double distance = Math.hypot(zoneTarget[0] - targetX, zoneTarget[1] - targetY);
            if (distance < bestDistance) {
                bestTarget = zoneTarget;
                bestDistance = distance;
            }
        }
        return bestTarget;
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
        if(!aimingEnabled) {
            flywheelTargetRPM = minFlywheelRPM;
            return;
        }

        ballisticDistance = Math.hypot(targetX - turretX, targetY - turretY) * 0.0254;
        if (ballisticDistance < 0.05) {
            flywheelTargetRPM = minFlywheelRPM;
            return;
        }

        double middleAngle = Math.toRadians((maxTrajectoryAngle + minTrajectoryAngle) / 2.0);
        double denominator = 2 * (ballisticDistance * Math.tan(middleAngle) - targetZ);
        if (denominator <= 0 || Math.abs(Math.cos(middleAngle)) < 1e-6) {
            flywheelTargetRPM = minFlywheelRPM;
            return;
        }

        targetLaunchSpeed = ballisticDistance / Math.cos(middleAngle) * Math.sqrt(g / denominator);
        flywheelTargetRPM = targetLaunchSpeed * 60.0
                / (2 * Math.PI * flywheelRadius * launcherEfficiency);

        double currentLaunchSpeed = (rpm / 60.0)
                * 2 * Math.PI * flywheelRadius * launcherEfficiency;
        if (currentLaunchSpeed <= 0.1) return;

        double v2 = currentLaunchSpeed * currentLaunchSpeed;
        double v4 = v2 * v2;
        double rootInner = v4
                - (2 * g * v2 * targetZ)
                - (g * g * ballisticDistance * ballisticDistance);
        if (rootInner >= 0) {
            double tanTheta = (v2 - Math.sqrt(rootInner)) / (g * ballisticDistance);
            trajectoryAngle = Math.toDegrees(Math.atan(tanTheta));
            rpmProducesValidTrajectory = trajectoryAngle >= minTrajectoryAngle
                    && trajectoryAngle <= maxTrajectoryAngle;
        }
    }
    private void updateTurretAim() {
        if(aimingEnabled) {
            double dx = targetX - turretX;
            double dy = targetY - turretY;
            double fieldAngle = Math.atan2(dy, dx);
            targetTurretDeg = normalizeAngle(Math.toDegrees(fieldAngle - robotHeading));
        } else {
            turretLastUpdateNanos = System.nanoTime();
            turretErrorSum = 0;
            turret.setPower(0);
            return;
        }
        double compensatedTargetDeg = normalizeAngle(targetTurretDeg + velocityCompensation[0]);
        double desiredTravelDeg = selectReachableTurretTravel(compensatedTargetDeg);
        targetTurretTravelDeg = Range.clip(desiredTravelDeg, getTurretMinTravelDeg(), getTurretMaxTravelDeg());
        turretTargetLimited = targetTurretTravelDeg != desiredTravelDeg;
        targetTurretDeg = normalizeAngle(targetTurretTravelDeg + startTurretAngle + angleOffset);
        turretError = targetTurretTravelDeg - currentTurretTravelDeg;
        long now = System.nanoTime();
        double dt = turretLastUpdateNanos == 0 ? 0.0 : (now - turretLastUpdateNanos) / 1e9;
        turretLastUpdateNanos = now;
        turretLastDt = dt;

        if (Math.abs(turretError) <= turretTolerance) {
            turretErrorSum = 0.0;
            turretDerivative = 0.0;
            turretIntegralState = "deadband reset";
            turretPower = 0.0;
        } else {
            if (dt > 1e-6) {
                if (turretResetIntegralOnSignChange
                        && Math.signum(turretError) != Math.signum(turretLastError)
                        && turretLastError != 0.0) {
                    turretErrorSum = 0.0;
                    turretIntegralState = "sign reset";
                }

                double unclampedErrorSum = turretErrorSum + turretError * dt;
                turretErrorSum = Range.clip(unclampedErrorSum, turretIntegralMin, turretIntegralMax);
                if (turretErrorSum != unclampedErrorSum) {
                    turretIntegralState = "clamped";
                } else if(!"sign reset".equals(turretIntegralState)) {
                    turretIntegralState = "accumulating";
                }

                if (!turretPidFirstRun) {
                    turretDerivative = (currentTurretTravelDeg - turretLastMeasurement) / dt;
                } else {
                    turretDerivative = 0.0;
                    turretPidFirstRun = false;
                }
            }

            turretPower = turretKp * turretError
                    + turretKi * turretErrorSum
                    - turretKd * turretDerivative
                    + turretKf * Math.signum(turretError);
        }

        turretLastError = turretError;
        turretLastMeasurement = currentTurretTravelDeg;
        turretPower = Range.clip(turretPower, -turretMaxPower, turretMaxPower);
        if (currentTurretTravelDeg >= getTurretCwLimitTravelDeg() && turretPower > 0.0) {
            turretPower = 0.0;
            turretErrorSum = 0.0;
            turretIntegralState = "cw limit hold";
        } else if (currentTurretTravelDeg <= getTurretCcwLimitTravelDeg() && turretPower < 0.0) {
            turretPower = 0.0;
            turretErrorSum = 0.0;
            turretIntegralState = "ccw limit hold";
        }
        turretOnTarget = Math.abs(turretError) <= turretTolerance;
        turret.setPower(turretPower);
    }
    private double getTurretCwLimitTravelDeg() {
        return Math.abs(turretCwLimitDeg);
    }
    private double getTurretCcwLimitTravelDeg() {
        return -Math.abs(turretCcwLimitDeg);
    }
    private double getTurretMinTravelDeg() {
        return Math.min(getTurretCwLimitTravelDeg(), getTurretCcwLimitTravelDeg());
    }
    private double getTurretMaxTravelDeg() {
        return Math.max(getTurretCwLimitTravelDeg(), getTurretCcwLimitTravelDeg());
    }
    private double selectReachableTurretTravel(double targetAngleDeg) {
        double baseTravelDeg = normalizeAngle(targetAngleDeg - startTurretAngle - angleOffset);
        double bestTravelDeg = 0.0;
        double bestDistance = Double.MAX_VALUE;
        boolean foundReachableTarget = false;

        for (int i = -2; i <= 2; i++) {
            double candidateTravelDeg = baseTravelDeg + i * 360.0;
            if (candidateTravelDeg < getTurretMinTravelDeg()
                    || candidateTravelDeg > getTurretMaxTravelDeg()) {
                continue;
            }

            double distance = Math.abs(candidateTravelDeg - currentTurretTravelDeg);
            if (distance < bestDistance) {
                bestTravelDeg = candidateTravelDeg;
                bestDistance = distance;
                foundReachableTarget = true;
            }
        }

        if (foundReachableTarget) {
            return bestTravelDeg;
        }

        return currentTurretTravelDeg + normalizeAngle(targetAngleDeg - currentTurretDeg);
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
    public static double kP_v = 120;     // try 20–35
    public static double kI_v = 9;      // keep 0 for fastest transient
    public static double kD_v = 10;      // add small later only if it overshoots/oscillates
    public static double kF_v = 0;
    private void updateFlywheel() {
            double target = flywheelTargetRPM;
            double targetTPS = target * FLYWHEEL_TICKS_PER_REV / 60.0;
            flywheel.setVelocityPIDFCoefficients(kP_v, kI_v, kD_v, kF_v);
            flywheel.setVelocity(targetTPS);
    }
    private void updateTelemetry() {
        if (telemetry == null) return;

        telemetry.addLine("=== Shooter ===");

        // --- Flywheel ---
        telemetry.addData("Flywheel RPM", "%.0f / %.0f", rpm, flywheelTargetRPM);
        telemetry.addData("RPM In Range", Math.abs(flywheelTargetRPM - rpm) <= RPM_TOL);

        // --- Trajectory ---
        telemetry.addData("Trajectory Angle (deg)", "%.1f", trajectoryAngle);
        telemetry.addData("Ballistic Distance (m)", "%.2f", ballisticDistance);
        telemetry.addData("Target Launch Speed (m/s)", "%.2f", targetLaunchSpeed);
        telemetry.addData("Launch Enabled", launcherEnabled);
        telemetry.addData("Valid Trajectory", rpmProducesValidTrajectory);

        // --- Turret ---
        telemetry.addData("Turret Angle (deg)", "%.1f", currentTurretDeg);
        telemetry.addData("Target Angle (deg)", targetTurretDeg);
        telemetry.addData("Turret Travel (deg)", "%.1f / %.1f", currentTurretTravelDeg, targetTurretTravelDeg);
        telemetry.addData("Turret Travel Limits CW/CCW", "%.1f / %.1f", getTurretCwLimitTravelDeg(), getTurretCcwLimitTravelDeg());
        telemetry.addData("Turret Target Limited", turretTargetLimited);
        telemetry.addData("Turret Error (deg)", "%.2f", turretError);
        telemetry.addData("Turret Power", "%.3f", turretPower);
        telemetry.addData("Turret On Target", turretOnTarget);
        telemetry.addData("Yaw Comp (deg)", velocityCompensation[0]);
        telemetry.addData("Turret PIDF", "P %.4f I %.4f D %.4f F %.4f", turretKp, turretKi, turretKd, turretKf);
        telemetry.addData("Turret PID State", "err %.2f sum %.3f der %.3f out %.3f dt %.3f", turretError, turretErrorSum, turretDerivative, turretPower, turretLastDt);
        telemetry.addData("Turret Integral State", turretIntegralState);

        // --- Target ---
        telemetry.addData("Target X/Y", "%.1f , %.1f", targetX, targetY);

        // --- Robot ---
        telemetry.addData("Robot X/Y", "%.1f , %.1f", pose.getX(), pose.getY());
        telemetry.addData("Robot Heading (deg)", "%.1f", Math.toDegrees(pose.getHeading()));
        telemetry.addData("Robot Vel X/Y", "%.2f , %.2f", velocity.getX(), velocity.getY());

    }
    public void appendPanelsTelemetry(TelemetryManager telemetryM) {
        if (telemetryM == null) return;

        telemetryM.addData("Shooter Turret Current", currentTurretDeg);
        telemetryM.addData("Shooter Turret Target", targetTurretDeg);
        telemetryM.addData("Shooter Turret Current Travel", currentTurretTravelDeg);
        telemetryM.addData("Shooter Turret Target Travel", targetTurretTravelDeg);
        telemetryM.addData("Shooter Turret CW Limit", getTurretCwLimitTravelDeg());
        telemetryM.addData("Shooter Turret CCW Limit", getTurretCcwLimitTravelDeg());
        telemetryM.addData("Shooter Turret Target Limited", turretTargetLimited);
        telemetryM.addData("Shooter Turret Error", turretError);
        telemetryM.addData("Shooter Turret Output", turretPower);
        telemetryM.addData("Shooter Turret Error Sum", turretErrorSum);
        telemetryM.addData("Shooter Turret Derivative", turretDerivative);
        telemetryM.addData("Shooter Turret Dt", turretLastDt);
        telemetryM.addData("Shooter Turret kP", turretKp);
        telemetryM.addData("Shooter Turret kI", turretKi);
        telemetryM.addData("Shooter Turret kD", turretKd);
        telemetryM.addData("Shooter Turret kF", turretKf);
        telemetryM.addData("Shooter Turret Max Power", turretMaxPower);

        telemetryM.addData("Shooter Flywheel Current RPM", rpm);
        telemetryM.addData("Shooter Flywheel Target RPM", flywheelTargetRPM);
        telemetryM.addData("Shooter Flywheel Error RPM", flywheelTargetRPM - rpm);
        telemetryM.addData("Shooter Flywheel kP", kP_v);
        telemetryM.addData("Shooter Flywheel kI", kI_v);
        telemetryM.addData("Shooter Flywheel kD", kD_v);
        telemetryM.addData("Shooter Flywheel kF", kF_v);
    }

}
