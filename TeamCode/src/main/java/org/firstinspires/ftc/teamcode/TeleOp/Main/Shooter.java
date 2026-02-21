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
    static final double launcherEfficiency = 0.425;
    public static final double turretOffsetX = 0.0;
    public static final double turretOffsetY = 52/1000.0;
    final double[][] launchZoneBig = {{-18,144},{162,144},{72,55}};
    final double[][] launchZoneSmall = {{40,0},{102,0},{72,40}};
    private boolean aimingEnabled = false;
    private boolean launcherEnabled = false;
    final double absoluteTurretHeight = 0.25; //meters
    final double absoluteTargetHeight = 1.05; //meters
    final double relativeHeight = absoluteTargetHeight - absoluteTurretHeight;
    final double preferredMaxHeightThrow = relativeHeight + 0.3; //meters (relative to turret height)
    final double kP = 0.015;
    final double kD = 0.002;
    int zone = 0;
    final double MAX_POWER_TURRET = 0.4;
    final double startTurretAngle = -180.0;
    final double LEFT_LIMIT = -110;
    final double RIGHT_LIMIT = 110;
    private static final double MOTOR_TICKS_PER_REV = 384.5;
    private static final double MOTOR_TO_TURRET_RATIO = 75.0 / 26.0;
    private static final double MAX_RPM_ACCELERATION = 2000; // RPM per second, adjust based on your flywheel

    private static final double TURRET_DEG_PER_TICK =
            360.0 / (MOTOR_TICKS_PER_REV * MOTOR_TO_TURRET_RATIO);
    final double FORBIDDEN_ANGLE = -67;
    public final double TICKS_PER_DEGREE = 1.0 / TURRET_DEG_PER_TICK;
    double turretX = 0.0;
    double turretY = 0.0;
    double flywheelTargetRPM = 0;
    boolean foundSolution = false;
    boolean rpmTooLow = false;
    double currentTurretDeg = 0;
    double currentOffset = 0;
    double rpm = 0.0;
    double v0 = 0;
    double trajectoryAngle = 70;
    private static final double RPM_TOL = 25;
    private static final long RPM_STABLE_MS = 80;
    private static final double TURRET_TOL = 2.0;
    private long rpmInRangeSinceMs = 0;
    boolean turretOnTarget = false;
    double lastError = 0;
    boolean forceEnableLauncher = false;
    private static final double RPM_UP_STEP   = 120; // rpm per cycle when infeasible
    private static final double RPM_DOWN_STEP = 40;  // rpm per cycle when feasible
    private static final int RPM_DECAY_LOOPS  = 3;   // cycles of feasibility before decay

    private int rpmFeasibleCounter = 0;
    public Shooter(HardwareMap hwMap, String flywheelName, String turretName, String servoName){
        flywheel = hwMap.get(DcMotorEx.class,flywheelName);
        turret = hwMap.get(DcMotorEx.class,turretName);
        trajectoryAngleModifier = hwMap.get(Servo.class,servoName);
    }
    public void init(Telemetry telemetry) {
        flywheel.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(30, 2.3, 4, 14.0));
        turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        trajectoryAngleModifier.setDirection(Servo.Direction.REVERSE);
        trajectoryAngleModifier.setPosition(0);
        this.telemetry = telemetry;
    }
    public void update(Pose pose,Pose velocity, double targetX, double targetY, boolean isShooting){
        this.targetX = targetX;
        this.targetY = targetY;
        this.pose = pose;
        this.velocity = velocity;
        currentTurretDeg = turret.getCurrentPosition()/TICKS_PER_DEGREE + startTurretAngle;
        updateFlywheel();
        updateTurretAim();
        if(aimingEnabled) {
            computeParameters();
            if (zone == 1) {
                adaptRPM(isShooting);
            }
            updateTrajectoryAngle();
        }
        updateTelemetry();
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
        return Math.abs(flywheelTargetRPM - rpm) < RPM_TOL;
    }
    public boolean isTurretOnTarget(){
        return turretOnTarget;
    }
    private boolean rpmInRangeStable() {
        boolean inRange = (rpm >= (flywheelTargetRPM - RPM_TOL)) && (rpm <= (flywheelTargetRPM + RPM_TOL));
        long now = System.currentTimeMillis();

        if (!inRange) {
            rpmInRangeSinceMs = 0;
            return false;
        }
        if (rpmInRangeSinceMs == 0) rpmInRangeSinceMs = now;
        return (now - rpmInRangeSinceMs) >= RPM_STABLE_MS;
    }
    private void disableIfNotInLaunchZone(){
        double robotX = pose.getX();
        double robotY = pose.getY();
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
            zone = 0;
        } else {
            if (robotY > 48) {
                zone = 1;
            } else {
                zone = 2;
            }
            if(!aimingEnabled) enableAiming();
        }
    }

    private double sign(double x1,double y1,double x2,double y2,double x3,double y3){
        return (x1-x3)*(y2-y3)-(x2-x3)*(y1-y3);
    }
    private double estimateTimeToShot() {
        double rpmError = Math.abs(flywheelTargetRPM - rpm);
        double spinUpTime = rpmError / MAX_RPM_ACCELERATION;
        return Math.max(0.1, spinUpTime);
    }


    private double estimateFlightTime(double distance, double angleDegrees, double velocity) {
        double theta = Math.toRadians(angleDegrees);
        return distance / (velocity * Math.cos(theta));
    }
    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }
    double[] computeVelocityCompensation() {

        // Robot velocity in field frame
        double vRobot = Math.hypot(velocity.getX(), velocity.getY());
        if (vRobot < 0.05) return new double[]{0, v0};

        double turretHeading =
                velocity.getHeading() + Math.toRadians(currentTurretDeg);

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

        return new double[] { yawCorrection, newV0 };
    }


    private void computeParameters() {
        if(zone == 1){
            maxFlywheelRPM = 2500;
        } else if(zone == 2){
            maxFlywheelRPM = 3150;
        } else {
            maxFlywheelRPM = minFlywheelRPM;
        }
        if(zone == 2){
            flywheelTargetRPM = 3100;
            trajectoryAngle = maxTrajectoryAngle;
            foundSolution = true;
            rpmTooLow = rpm < flywheelTargetRPM - RPM_TOL;
            return;
        }
        if(zone==0){
            flywheelTargetRPM = 1000;
            foundSolution = true;
            rpmTooLow = false;
            return;

        }
        foundSolution = false;
        rpmTooLow = false;
        double d = Math.hypot(targetX - turretX, targetY - turretY) * 0.0254;

        if (d <= 0) {
            flywheelTargetRPM = 2000;
            trajectoryAngle = maxTrajectoryAngle;
            return;
        }
        v0 = (rpm / 60.0) * 2 * Math.PI * flywheelRadius * launcherEfficiency;
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
                    foundSolution = true;
                    if(highValid){
                        trajectoryAngle = angleHigh;
                    } else {
                        trajectoryAngle = angleLow;
                    }
                } else {
                    foundSolution = false;
                    rpmTooLow = (angleHigh > maxTrajectoryAngle);
                }
            } else {
                foundSolution = false;
                rpmTooLow = true;
            }
        } else {
            foundSolution = false;
            rpmTooLow = true;
        }
            flywheelTargetRPM = Range.clip(flywheelTargetRPM,minFlywheelRPM,maxFlywheelRPM);
    }
//    private void updateTurretAim() {
//        disableIfNotInLaunchZone();
//        double targetTurretDeg;
//        if (aimingEnabled) {
//            double robotX = pose.getX();
//            double robotY = pose.getY();
//            double robotHeading = pose.getHeading();
//            double robotHeadingDeg = Math.toDegrees(robotHeading);
//            double timeToShot = estimateTimeToShot();
//            double futureRobotX = robotX + velocity.getX() * timeToShot;
//            double futureRobotY = robotY + velocity.getY() * timeToShot;
//            double futureTurretX = futureRobotX
//                    + turretOffsetX * Math.cos(robotHeading)
//                    - turretOffsetY * Math.sin(robotHeading);
//            double futureTurretY = futureRobotY
//                    + turretOffsetX * Math.sin(robotHeading)
//                    + turretOffsetY * Math.cos(robotHeading);
//            double dxTarget = targetX - futureTurretX;
//            double dyTarget = targetY - futureTurretY;
//            double fieldAngle = Math.toDegrees(Math.atan2(dyTarget, dxTarget));
//            targetTurretDeg = normalizeAngle(fieldAngle - robotHeadingDeg);
//        } else {
//            targetTurretDeg = startTurretAngle;
//        }
//        if (targetTurretDeg < 0 && targetTurretDeg > LEFT_LIMIT) {
//            targetTurretDeg = LEFT_LIMIT;
//        } else if (targetTurretDeg >= 0 && targetTurretDeg < RIGHT_LIMIT) {
//            targetTurretDeg = RIGHT_LIMIT;
//        }
//        double error = normalizeAngle(targetTurretDeg - currentTurretDeg)/TURRET_DEG_PER_TICK;
//        double derivative = error - lastError;
//        lastError = error;
//        double power = error * kP + derivative * kD;
//        turretOnTarget = Math.abs(error) <= TURRET_TOL_DEG;
//        power = Range.clip(power, -MAX_POWER_TURRET, MAX_POWER_TURRET);
//        turret.setPower(power);
//    }
double targetTurretDeg = 0;
//    private void updateTurretAim() {
//        disableIfNotInLaunchZone();
//        if(aimingEnabled) {
//            double robotX = pose.getX();
//            double robotY = pose.getY();
//            double robotHeading = pose.getHeading();
//            double robotHeadingDeg = Math.toDegrees(robotHeading);
//            turretX = robotX + turretOffsetX * Math.cos(robotHeading) - turretOffsetY * Math.sin(robotHeading);
//            turretY = robotY + turretOffsetX * Math.sin(robotHeading) + turretOffsetY * Math.cos(robotHeading);
//            double dx = targetX - turretX;
//            double dy = targetY - turretY;
//
//            double fieldAngle = Math.toDegrees(Math.atan2(dy, dx));
//            targetTurretDeg = normalizeAngle(fieldAngle - robotHeadingDeg);
//        } else {
//            targetTurretDeg = startTurretAngle;
//        }
//        if(targetTurretDeg < -30 && targetTurretDeg > -50) {
//            targetTurretDeg = -30;
//        } else if(targetTurretDeg > -70 && targetTurretDeg < -50){
//            targetTurretDeg = -70;
//        }
//        double error = computeSafeTurretError(currentTurretDeg, targetTurretDeg)/TURRET_DEG_PER_TICK;
//
//        double power = error * kP;
//        turretOnTarget = Math.abs(error) <= TURRET_TOL;
//        power = Range.clip(power, -MAX_POWER_TURRET, MAX_POWER_TURRET);
//        turret.setPower(power);
//    }
private void updateTurretAim() {
    disableIfNotInLaunchZone();
    double targetTurretDeg;
    double currentTurretDeg;
    if(aimingEnabled) {
        double robotX = pose.getX();
        double robotY = pose.getY();
        double robotHeading = pose.getHeading();

        double robotHeadingDeg = Math.toDegrees(robotHeading);
        turretX = robotX + turretOffsetX * Math.cos(robotHeading) - turretOffsetY * Math.sin(robotHeading);
        turretY = robotY + turretOffsetX * Math.sin(robotHeading) + turretOffsetY * Math.cos(robotHeading);
        double dx = targetX - turretX;
        double dy = targetY - turretY;

        double fieldAngle = Math.toDegrees(Math.atan2(dy, dx));
        currentTurretDeg = turret.getCurrentPosition() * TURRET_DEG_PER_TICK + startTurretAngle;
        targetTurretDeg = normalizeAngle(fieldAngle - robotHeadingDeg);
    } else {
        currentTurretDeg = turret.getCurrentPosition() * TURRET_DEG_PER_TICK + startTurretAngle;
        targetTurretDeg = startTurretAngle;
    }
    currentTurretDeg=normalizeAngle(currentTurretDeg);
    if(targetTurretDeg < 0 && targetTurretDeg > LEFT_LIMIT){
        targetTurretDeg = LEFT_LIMIT;
    } else if (targetTurretDeg >= 0 && targetTurretDeg < RIGHT_LIMIT) {
        targetTurretDeg = RIGHT_LIMIT;
    }

    double error = normalizeAngle(targetTurretDeg - currentTurretDeg);

    double power = error * kP;
    turretOnTarget = Math.abs(error) <= TURRET_TOL;
    power = Range.clip(power, -MAX_POWER_TURRET, MAX_POWER_TURRET);
    turret.setPower(power);
}
    private double computeSafeTurretError(double from, double to) {
        double error = normalizeAngle(to - from);

        // Check if shortest path crosses forbidden zone
        if (crossesForbidden(from, to)) {
            // Reverse rotation direction to avoid forbidden zone
            if (error > 0) {
                error = error - 360;
            } else {
                error = error + 360;
            }
        }

        return error;
    }

    private boolean crossesForbidden(double a, double b) {
        // Handles modular 360° forbidden check
        double start = normalizeAngle(a);
        double end = normalizeAngle(b);
        double forbiddenStart = -70;
        double forbiddenEnd = -30;

        // Normalize forbidden range to 0-360
        forbiddenStart = normalizeAngle(forbiddenStart);
        forbiddenEnd = normalizeAngle(forbiddenEnd);

        if (forbiddenStart < forbiddenEnd) {
            return (start < forbiddenEnd && end > forbiddenStart &&
                    (start < forbiddenStart || end > forbiddenEnd || start > end));
        } else { // forbidden zone wraps around
            return (start > forbiddenStart || end < forbiddenEnd);
        }
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
    // Correct ballpark for 6000rpm Yellow Jacket (28tpr): ~11.7 at 12V no-load
    public final double kF_v = 14;
//    private void updateFlywheel() {
//        // Measure RPM
//        rpm = flywheel.getVelocity() / FLYWHEEL_TICKS_PER_REV * 60.0;
//
//        double target = flywheelTargetRPM;
//
//        // Convert target to ticks/sec for RUN_USING_ENCODER velocity
//        double targetTPS = target * FLYWHEEL_TICKS_PER_REV / 60.0;
//
//        // === Control policy ===
//        // In-band (±100rpm): PID velocity hold
//        // Below band: full power 1.0
//        // Above band: power 0.0
//        if (foundSolution) {
//                if (flywheel.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
//                    flywheel.setPower(0); // clear any open-loop command
//                    flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                }
//                flywheel.setVelocityPIDFCoefficients(kP_v, kI_v, kD_v, kF_v);
//                flywheel.setVelocity(targetTPS);
//        } else {
//            if (flywheel.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
//                flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            }
//            flywheel.setPower(rpmTooLow ? 1 : 0);
//        }
//
//
//        // Refresh rpm after command (optional but nice for telemetry stability)
//        rpm = flywheel.getVelocity() / FLYWHEEL_TICKS_PER_REV * 60.0;
//    }
    private void updateFlywheel() {

        rpm = flywheel.getVelocity() / FLYWHEEL_TICKS_PER_REV * 60.0;
        if(rpm > flywheelTargetRPM) {
            flywheel.setPower(0);
        } else if (Math.abs(rpm - flywheelTargetRPM) < RPM_TOL) {
            double target = flywheelTargetRPM;
            double targetTPS = target * FLYWHEEL_TICKS_PER_REV / 60.0;
            flywheel.setVelocityPIDFCoefficients(kP_v, kI_v, kD_v, kF_v);
            flywheel.setVelocity(targetTPS);
        } else if (rpm < flywheelTargetRPM){
            flywheel.setPower(1);
        } else {
            flywheel.setPower(0);
        }
    }
    private void adaptRPM(boolean isShooting) {
        if (foundSolution) {
            if(!isShooting) {
                rpmFeasibleCounter++;

                if (rpmFeasibleCounter >= RPM_DECAY_LOOPS) {
                    double angleNorm =
                            (trajectoryAngle - minTrajectoryAngle) /
                                    (maxTrajectoryAngle - minTrajectoryAngle);

                    double decayScale = 1.0 - angleNorm; // low angle → more decay

                    flywheelTargetRPM -= RPM_DOWN_STEP * decayScale;
                }
            }
        } else {
            rpmFeasibleCounter = 0;

            if (rpmTooLow) {
                flywheelTargetRPM += RPM_UP_STEP;
            }
        }
        flywheelTargetRPM = Range.clip(flywheelTargetRPM, minFlywheelRPM, maxFlywheelRPM);
    }
    public void enableAiming(){
        aimingEnabled = true;
    }

    public void disableAiming(){
        aimingEnabled = false;
    }
    private void updateTelemetry() {
        if (telemetry == null) return;

        telemetry.addLine("=== Shooter ===");

        // --- Flywheel ---
        telemetry.addData("Flywheel RPM", "%.0f / %.0f", rpm, flywheelTargetRPM);
        telemetry.addData("RPM In Range", isRPMInRange());

        // --- Trajectory ---
        telemetry.addData("Trajectory Angle (deg)", "%.1f", trajectoryAngle);
        telemetry.addData("Launch Enabled", launcherEnabled);

        // --- Turret ---
        telemetry.addData("Turret Angle (deg)", "%.1f", currentTurretDeg);
        telemetry.addData("Turret On Target", turretOnTarget);

        // --- Target ---
        telemetry.addData("Target X/Y", "%.1f , %.1f", targetX, targetY);

        // --- Robot ---
        telemetry.addData("Robot X/Y", "%.1f , %.1f", pose.getX(), pose.getY());
        telemetry.addData("Robot Heading (deg)", "%.1f", Math.toDegrees(pose.getHeading()));
        telemetry.addData("Robot Vel X/Y", "%.2f , %.2f", velocity.getX(), velocity.getY());
        telemetry.addData("Target",targetTurretDeg);
    }

}
