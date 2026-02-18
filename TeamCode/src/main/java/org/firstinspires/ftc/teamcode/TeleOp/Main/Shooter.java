package org.firstinspires.ftc.teamcode.TeleOp.Main;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class Shooter {
    DcMotorEx flywheel;
    DcMotorEx turret;
    Servo trajectoryAngleModifier;
    Pose pose;
    Pose velocity;
    final double maxTrajectoryAngle = 70;
    final double minTrajectoryAngle = 50;
    static final double FLYWHEEL_TICKS_PER_REV = 28;
    double targetX = 0;
    double targetY = 144;
    static final double minFlywheelRPM = 1000;
    static final double maxFlywheelRPM = 6000;
    final double trajectoryAngleModifierGearRatio = 127/15.0;
    final double trajectoryAnglerMaxTravel = 300.0;
    final double trajectoryAnglePosPerDegree = trajectoryAngleModifierGearRatio/trajectoryAnglerMaxTravel;
    static final double g = 9.81;
    static final double flywheelRadius = 0.048;
    static final double launcherEfficiency = 0.43;
    public static final double turretOffsetX = 0.0;
    public static final double turretOffsetY = 52/1000.0;
    final double[][] launchZoneBig = {{-18,144},{162,144},{72,55}};
    final double[][] launchZoneSmall = {{40,0},{102,0},{72,32}};
    private boolean aimingEnabled = false;
    private boolean launcherEnabled = false;
    final double absoluteTurretHeight = 0.25; //meters
    final double absoluteTargetHeight = 0.9; //meters
    final double relativeHeight = absoluteTargetHeight - absoluteTurretHeight;
    final double preferredMaxHeightThrow = relativeHeight + 0.3; //meters (relative to turret height)
    final double kP = 0.015;
    final double kD = 0.002;
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
    double currentTurretDeg = 0;
    double rpm = 0.0;
    double v0 = 0;
    double trajectoryAngle = 70;
    private static final double RPM_TOL = 150;
    private static final long RPM_STABLE_MS = 80;
    private static final double TURRET_TOL = 2.0;
    private long rpmInRangeSinceMs = 0;
    boolean turretOnTarget = false;
    double lastError = 0;
    boolean forceEnableLauncher = false;
    public Shooter(HardwareMap hwMap, String flywheelName, String turretName, String servoName){
        flywheel = hwMap.get(DcMotorEx.class,flywheelName);
        turret = hwMap.get(DcMotorEx.class,turretName);
        trajectoryAngleModifier = hwMap.get(Servo.class,servoName);
    }
    public void init() {
        flywheel.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(30, 2.3, 4, 14.0));
        turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void update(Pose pose,Pose velocity, double targetX, double targetY){
        this.targetX = targetX;
        this.targetY = targetY;
        this.pose = pose;
        this.velocity = velocity;
        currentTurretDeg = turret.getCurrentPosition()/TICKS_PER_DEGREE + startTurretAngle;
        updateFlywheel();
        updateTurretAim();
        if(aimingEnabled) {
            computeParameters();
            updateTrajectoryAngle();
        }
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
        return rpmInRangeStable();
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
        } else {
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
        double d = Math.hypot(targetX - turretX, targetY - turretY) * 0.0254;

        if (d <= 0) {
            flywheelTargetRPM = 2000;
            trajectoryAngle = maxTrajectoryAngle;
            return;
        }
        v0 = (rpm / 60.0) * 2 * Math.PI * flywheelRadius * launcherEfficiency;
        double[] compensation = computeVelocityCompensation();
        v0 = compensation[1];

        boolean foundSolution = false;
        if(v0 > 1.0) {
            double a = g * d * d / (2 * v0 * v0);
            double b = -d;
            double c = relativeHeight + a;
            double discriminant = b * b - 4 * a * c;
            if (discriminant >= 0) {
                double tanTheta1 = (-b + Math.sqrt(discriminant)) / (2 * a);
                double tanTheta2 = (-b - Math.sqrt(discriminant)) / (2 * a);
                double angle1 = Math.toDegrees(Math.atan(tanTheta1));
                double angle2 = Math.toDegrees(Math.atan(tanTheta2));
                boolean angle1Valid = angle1 >= minTrajectoryAngle && angle1 <= maxTrajectoryAngle;
                boolean angle2Valid = angle2 >= minTrajectoryAngle && angle2 <= maxTrajectoryAngle;
                if (angle1Valid || angle2Valid) {
                    foundSolution = true;
                    flywheelTargetRPM = rpm;
                    if (angle1Valid) {
                        trajectoryAngle = angle1;
                    } else {
                        trajectoryAngle = angle2;
                    }
                }
            }
        }
        if (!foundSolution) {
            double k = (4.0 * preferredMaxHeightThrow / d)
                    * (1.0 - Math.sqrt(1.0 - relativeHeight / preferredMaxHeightThrow));

            double idealAngle = Math.toDegrees(Math.atan(k));
            boolean constrained = (idealAngle > maxTrajectoryAngle || idealAngle < minTrajectoryAngle);

            if (constrained) {
                trajectoryAngle = (idealAngle > maxTrajectoryAngle) ? maxTrajectoryAngle : minTrajectoryAngle;

                double thetaRad = Math.toRadians(trajectoryAngle);
                double cosTheta = Math.cos(thetaRad);
                double tanTheta = Math.tan(thetaRad);

                if (Math.abs(cosTheta) < 1e-6) {
                    return;
                }

                double denominator = d * tanTheta - relativeHeight;

                if (denominator <= 0) {
                    flywheelTargetRPM = minFlywheelRPM;
                    return;
                }

                double exitVelocity = Math.sqrt((g * d * d) /
                        (2 * cosTheta * cosTheta * denominator));

                flywheelTargetRPM = (int) (60 * exitVelocity /
                        (2 * Math.PI * flywheelRadius * launcherEfficiency));

            } else {
                trajectoryAngle = idealAngle;

                double thetaRad = Math.toRadians(trajectoryAngle);
                double sinTheta = Math.sin(thetaRad);

                if (Math.abs(sinTheta) < 1e-6) {
                    return;
                }

                v0 = Math.sqrt(2 * g * preferredMaxHeightThrow) / sinTheta;
                compensation = computeVelocityCompensation();
                v0 = compensation[1];
                flywheelTargetRPM = (int) (60 * v0 / (2 * Math.PI * flywheelRadius * launcherEfficiency));
            }
        }

        flywheelTargetRPM = Math.max(minFlywheelRPM, Math.min(flywheelTargetRPM, maxFlywheelRPM));
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

    private void updateTurretAim() {
        disableIfNotInLaunchZone();

        double targetTurretDeg;

        if (aimingEnabled) {
            // ----- Robot pose -----
            double robotX = pose.getX();
            double robotY = pose.getY();
            double robotHeading = pose.getHeading();
            double robotHeadingDeg = Math.toDegrees(robotHeading);

            // ----- Prediction -----
            double timeToShot = estimateTimeToShot();
            double futureRobotX = robotX + velocity.getX() * timeToShot;
            double futureRobotY = robotY + velocity.getY() * timeToShot;

            // ----- Turret offset -----
            double futureTurretX =
                    futureRobotX
                            + turretOffsetX * Math.cos(robotHeading)
                            - turretOffsetY * Math.sin(robotHeading);

            double futureTurretY =
                    futureRobotY
                            + turretOffsetX * Math.sin(robotHeading)
                            + turretOffsetY * Math.cos(robotHeading);

            // ----- Target vector -----
            double dxTarget = targetX - futureTurretX;
            double dyTarget = targetY - futureTurretY;

            double fieldAngle = Math.toDegrees(Math.atan2(dyTarget, dxTarget));

            // Convert field → turret frame
            targetTurretDeg = normalizeAngle(fieldAngle - robotHeadingDeg);
            double[] compensation = computeVelocityCompensation();
            targetTurretDeg = normalizeAngle(targetTurretDeg + compensation[0]);
        } else {
            targetTurretDeg = startTurretAngle;
        }

        // ----- Current turret angle (from encoder) -----
        double currentTurretTicks = turret.getCurrentPosition();
        double currentTurretDeg = currentTurretTicks / TICKS_PER_DEGREE;

        // ----- Angle-space error -----
        double angleError = normalizeAngle(targetTurretDeg - currentTurretDeg);

        // ----- Forbidden angle handling -----
        if (shortestPathCrossesForbidden(
                currentTurretDeg,
                targetTurretDeg
        )) {

            if (angleError > 0) {
                angleError -= 360;
            } else {
                angleError += 360;
            }
        }

        // ----- Convert to ticks -----
        double errorTicks = angleError * TICKS_PER_DEGREE;

        // ----- PID in ticks -----
        double derivative = errorTicks - lastError;
        lastError = errorTicks;

        double power = errorTicks * kP + derivative * kD;

        turretOnTarget =
                Math.abs(errorTicks) <= (TURRET_TOL);

        power = Range.clip(power, -MAX_POWER_TURRET, MAX_POWER_TURRET);
        turret.setPower(power);
    }
    private boolean shortestPathCrossesForbidden(double from, double to) {
        double delta = normalizeAngle(to - from);
        double step = Math.signum(delta);

        double current = from;
        double remaining = Math.abs(delta);

        while (remaining > 0) {
            double next = normalizeAngle(current + step);
            if (crosses(current, next)) {
                return true;
            }
            current = next;
            remaining -= 1.0;
        }
        return false;
    }

    private boolean crosses(double a, double b) {
        if (a <= b) {
            return FORBIDDEN_ANGLE >= a && FORBIDDEN_ANGLE <= b;
        } else {
            return FORBIDDEN_ANGLE >= a || FORBIDDEN_ANGLE <= b;
        }
    }
    public double getCurrentTurretDeg(){
        return currentTurretDeg;
    }
    public void enableLauncher(){
        if(forceEnableLauncher) return;
        forceEnableLauncher = true;
        launcherEnabled = true;
        updateLauncher();
    }

    public void disableLauncher(){
        if(!forceEnableLauncher) return;
        forceEnableLauncher = false;
        launcherEnabled = false;
        updateLauncher();
    }
    public static double kP_v = 30.0;     // try 20–35
    public static double kI_v = 0.0;      // keep 0 for fastest transient
    public static double kD_v = 0.0;      // add small later only if it overshoots/oscillates
    // Correct ballpark for 6000rpm Yellow Jacket (28tpr): ~11.7 at 12V no-load
    public static double kF_v = 14.0;
    private void updateFlywheel() {
        // Measure RPM
        rpm = flywheel.getVelocity() / FLYWHEEL_TICKS_PER_REV * 60.0;

        double target = flywheelTargetRPM;

        // Convert target to ticks/sec for RUN_USING_ENCODER velocity
        double targetTPS = target * FLYWHEEL_TICKS_PER_REV / 60.0;

        // === Control policy ===
        // In-band (±100rpm): PID velocity hold
        // Below band: full power 1.0
        // Above band: power 0.0
        if (Math.abs(rpm - target) <= RPM_TOL) {
            // PID HOLD
            if (flywheel.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
                flywheel.setPower(0); // clear any open-loop command
                flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            flywheel.setVelocityPIDFCoefficients(kP_v, kI_v, kD_v, kF_v);
            flywheel.setVelocity(targetTPS);
        } else if (rpm < (target - RPM_TOL)) {
            if (flywheel.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
                flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            flywheel.setPower(1.0);
        } else {
            // TOO FAST -> CUT POWER
            if (flywheel.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
                flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            flywheel.setPower(0.0);
        }

        // Refresh rpm after command (optional but nice for telemetry stability)
        rpm = flywheel.getVelocity() / FLYWHEEL_TICKS_PER_REV * 60.0;
    }
    private void updateLauncher(){
        double ticksPerSecond;
        if(launcherEnabled || forceEnableLauncher){
            ticksPerSecond = flywheelTargetRPM * FLYWHEEL_TICKS_PER_REV / 60.0;
        } else {
            ticksPerSecond = 2000 * FLYWHEEL_TICKS_PER_REV / 60.0;
        }
        flywheel.setVelocity(ticksPerSecond);
        rpm = flywheel.getVelocity() / FLYWHEEL_TICKS_PER_REV * 60.0;
    }

    public void enableAiming(){
        aimingEnabled = true;
    }

    public void disableAiming(){
        aimingEnabled = false;
    }
}
