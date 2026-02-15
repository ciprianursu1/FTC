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
    static final double turretOffsetX = 0.0;
    static final double turretOffsetY = 52/1000.0;
    final double[][] launchZoneBig = {{-18,144},{162,144},{72,55}};
    final double[][] launchZoneSmall = {{40,0},{102,0},{72,32}};
    private boolean aimingEnabled = false;
    private boolean launcherEnabled = false;
    final double absoluteTurretHeight = 0.25; //meters
    final double absoluteTargetHeight = 0.9; //meters
    final double relativeHeight = absoluteTargetHeight - absoluteTurretHeight;
    final double preferredMaxHeightThrow = relativeHeight + 0.3; //meters (relative to turret height)
    final double kP = 0.015;
    final double MAX_POWER_TURRET = 0.4;
    final double startTurretAngle = -180.0;
    final double LEFT_LIMIT = -110;
    final double RIGHT_LIMIT = 110;
    private static final double MOTOR_TICKS_PER_REV = 384.5;
    private static final double MOTOR_TO_TURRET_RATIO = 75.0 / 26.0;
    private static final double MAX_RPM_ACCELERATION = 2000; // RPM per second, adjust based on your flywheel

    private static final double TURRET_DEG_PER_TICK =
            360.0 / (MOTOR_TICKS_PER_REV * MOTOR_TO_TURRET_RATIO);

    double turretX = 0.0;
    double turretY = 0.0;
    double flywheelTargetRPM = 0;
    double rpm = 0.0;
    double trajectoryAngle = 70;
    private static final double RPM_TOL = 150;
    private static final long RPM_STABLE_MS = 80;
    private static final double TURRET_TOL_DEG = 2.0;
    private long rpmInRangeSinceMs = 0;
    boolean turretOnTarget = false;
    double targetTurretDeg = 0;
    boolean usingVelocityCompensation = false;
    boolean forceEnableLauncher = false;
    public Shooter(HardwareMap hwMap, String flywheelName, String turretName, String servoName){
        flywheel = hwMap.get(DcMotorEx.class,flywheelName);
        turret = hwMap.get(DcMotorEx.class,turretName);
        trajectoryAngleModifier = hwMap.get(Servo.class,servoName);
    }
    public void init() {
        flywheel.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(14, 2.3, 4, 14.0));
        turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        turret.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(5, 0, 0, 5));
        setTrajectoryAngle(minTrajectoryAngle);
    }
    public void update(Pose pose,Pose velocity){
        this.pose = pose;
        this.velocity = velocity;
        updateLauncher();
        if(usingVelocityCompensation && aimingEnabled){
            updateTurretAimMoving();
            computeParametersMoving();
            updateTrajectoryAngle();
        } else {
            updateTurretAimRunToPosition();
            if(aimingEnabled) {
                computeParameters();
                updateTrajectoryAngle();
            }
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
    private void computeParametersMoving() {
        // Convert distance from inches to meters
        double rawDistance = Math.hypot(targetX - turretX, targetY - turretY) * 0.0254;

        // Handle edge cases
        if (rawDistance <= 0.01) {
            flywheelTargetRPM = minFlywheelRPM;
            trajectoryAngle = maxTrajectoryAngle;
            return;
        }

        // ========== GET ROBOT VELOCITY ==========
        double speedTowardTarget = getSpeedTowardTarget(rawDistance);

        // Estimate flight time (iterative for better accuracy)
        double effectiveDistance = rawDistance;
        double flightTime = 0.3; // Initial guess

        // Iterate to converge on correct flight time (2-3 iterations is enough)
        for (int i = 0; i < 3; i++) {
            // Adjust distance based on movement during flight
            effectiveDistance = rawDistance - (speedTowardTarget * flightTime);
            if (effectiveDistance <= 0.01) {
                effectiveDistance = 0.01;
                break;
            }

            // Re-estimate flight time based on current solution
            double[] result = estimateLaunchParameters(effectiveDistance);
            double angle = result[0];
            double velocity = result[1];
            flightTime = estimateFlightTime(effectiveDistance, angle, velocity);
        }

        // ========== USE EFFECTIVE DISTANCE FOR BALLISTICS ==========
        double d = effectiveDistance;

        // Try to use current RPM first
        double currentVelocity = (rpm / 60.0) * 2 * Math.PI * flywheelRadius * launcherEfficiency;
        boolean foundSolution = false;

        // Solve quadratic for angle given current velocity
        double a = (g * d * d) / (2 * currentVelocity * currentVelocity);
        double b = -d;
        double c = a + relativeHeight;

        double discriminant = b * b - 4 * a * c;

        if (discriminant >= 0) {
            double tanTheta1 = (-b + Math.sqrt(discriminant)) / (2 * a);
            double tanTheta2 = (-b - Math.sqrt(discriminant)) / (2 * a);

            double angle1 = Math.toDegrees(Math.atan(tanTheta1));
            double angle2 = Math.toDegrees(Math.atan(tanTheta2));

            boolean angle1Valid = angle1 >= minTrajectoryAngle && angle1 <= maxTrajectoryAngle;
            boolean angle2Valid = angle2 >= minTrajectoryAngle && angle2 <= maxTrajectoryAngle;

            if (angle1Valid || angle2Valid) {
                double midAngle = (minTrajectoryAngle + maxTrajectoryAngle) / 2;

                if (angle1Valid && angle2Valid) {
                    trajectoryAngle = (Math.abs(angle1 - midAngle) < Math.abs(angle2 - midAngle)) ? angle1 : angle2;
                } else if (angle1Valid) {
                    trajectoryAngle = angle1;
                } else {
                    trajectoryAngle = angle2;
                }

                flywheelTargetRPM = rpm;
                foundSolution = true;
            }
        }

        // Original logic if no solution with current RPM
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

                if (Math.abs(cosTheta) < 1e-6) return;

                double denominator = d * tanTheta - relativeHeight;

                if (denominator <= 0) {
                    flywheelTargetRPM = minFlywheelRPM;
                    return;
                }

                double exitVelocity = Math.sqrt((g * d * d) /
                        (2 * cosTheta * cosTheta * denominator));

                flywheelTargetRPM = (int)(60 * exitVelocity /
                        (2 * Math.PI * flywheelRadius * launcherEfficiency));

            } else {
                trajectoryAngle = idealAngle;

                double thetaRad = Math.toRadians(trajectoryAngle);
                double sinTheta = Math.sin(thetaRad);

                if (Math.abs(sinTheta) < 1e-6) return;

                double v0 = Math.sqrt(2 * g * preferredMaxHeightThrow) / sinTheta;

                flywheelTargetRPM = (int)(60 * v0 / (2 * Math.PI * flywheelRadius * launcherEfficiency));
            }
        }

        // Apply limits
        flywheelTargetRPM = Math.max(minFlywheelRPM, Math.min(flywheelTargetRPM, maxFlywheelRPM));
        trajectoryAngle = Math.max(minTrajectoryAngle, Math.min(trajectoryAngle, maxTrajectoryAngle));
    }
    private double estimateTimeToShot() {
        double rpmError = Math.abs(flywheelTargetRPM - rpm);
        double spinUpTime = rpmError / MAX_RPM_ACCELERATION; // You need MAX_RPM_ACCELERATION
        return Math.max(0.1, spinUpTime); // At least 100ms
    }

    private double[] estimateLaunchParameters(double distance) {
        double k = (4.0 * preferredMaxHeightThrow / distance)
                * (1.0 - Math.sqrt(1.0 - relativeHeight / preferredMaxHeightThrow));
        double[] result = new double[2];
        double idealAngle = Math.toDegrees(Math.atan(k));
        double angle = clampAngle(idealAngle);
        result[0] = angle;
        if (angle == idealAngle) {
            // Unconstrained case - use max height formula
            double sinTheta = Math.sin(Math.toRadians(angle));
            result[1] = Math.sqrt(2 * g * preferredMaxHeightThrow) / sinTheta;
            return result;
        } else {
            // Constrained case - solve for velocity given angle
            double thetaRad = Math.toRadians(angle);
            double cosTheta = Math.cos(thetaRad);
            double tanTheta = Math.tan(thetaRad);

            double denominator = distance * tanTheta - relativeHeight;
            if (denominator <= 0) {
                result[1] = Math.sqrt(g * distance / Math.sin(2 * thetaRad));
                return result;
            }
            result[1] = Math.sqrt((g * distance * distance) /
                    (2 * cosTheta * cosTheta * denominator));
            return result;
        }
    }



    private double clampAngle(double idealAngle) {
        return Math.max(minTrajectoryAngle, Math.min(idealAngle, maxTrajectoryAngle));
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

    private double getSpeedTowardTarget(double rawDistance) {
        double robotVx = velocity.getX();
        double robotVy = velocity.getY();

        // ========== CALCULATE EFFECTIVE DISTANCE WITH MOVEMENT COMPENSATION ==========
        // Direction to target
        double dx = targetX - turretX;
        double dy = targetY - turretY;
        double targetDirX = dx / rawDistance;
        double targetDirY = dy / rawDistance;

        // Robot speed toward/away from target
        return robotVx * targetDirX + robotVy * targetDirY;
    }
    private void updateTurretAimMoving() {
        disableIfNotInLaunchZone();
        double targetTurretDeg;
        double currentTurretDeg;

        if(aimingEnabled) {
            // Get robot pose
            double robotX = pose.getX();
            double robotY = pose.getY();
            double robotHeading = pose.getHeading();
            double robotHeadingDeg = Math.toDegrees(robotHeading);

            // Calculate turret position on robot
            turretX = robotX + turretOffsetX * Math.cos(robotHeading) - turretOffsetY * Math.sin(robotHeading);
            turretY = robotY + turretOffsetX * Math.sin(robotHeading) + turretOffsetY * Math.cos(robotHeading);

            // Get robot velocity
            double robotVx = velocity.getX();
            double robotVy = velocity.getY();

            // Calculate time until shot (estimate based on flywheel spin-up)
            double timeToShot = estimateTimeToShot();

            // Predict where robot WILL BE when shot actually happens
            double futureRobotX = robotX + robotVx * timeToShot;
            double futureRobotY = robotY + robotVy * timeToShot;

            // Calculate future turret position
            double futureTurretX = futureRobotX + turretOffsetX * Math.cos(robotHeading) - turretOffsetY * Math.sin(robotHeading);
            double futureTurretY = futureRobotY + turretOffsetX * Math.sin(robotHeading) + turretOffsetY * Math.cos(robotHeading);

            // Calculate flight time from ballistics
            double distance = Math.hypot(targetX - futureTurretX, targetY - futureTurretY) * 0.0254;
            double flightTime = estimateFlightTime(distance, trajectoryAngle, flywheelTargetRPM);
            double dx = targetX - futureTurretX;
            double dy = targetY - futureTurretY;

            double fieldAngle = Math.toDegrees(Math.atan2(dy, dx));
            currentTurretDeg = turret.getCurrentPosition() * TURRET_DEG_PER_TICK + startTurretAngle;

            // Target angle relative to robot heading
            targetTurretDeg = normalizeAngle(fieldAngle - robotHeadingDeg);

            // ========== ADD LEAD FOR LATERAL MOVEMENT ==========
            // Calculate lateral speed (perpendicular to shot direction)
            double shotDirX = Math.cos(Math.toRadians(fieldAngle));
            double shotDirY = Math.sin(Math.toRadians(fieldAngle));

            // Robot lateral speed relative to shot direction
            double robotLateralSpeed = Math.abs(robotVx * shotDirY - robotVy * shotDirX);

            // Add lead angle if moving laterally (small adjustment, usually < 5 degrees)
            if (robotLateralSpeed > 0.1) {
                double leadAngle = Math.toDegrees(Math.atan2(robotLateralSpeed * flightTime, distance));
                // Lead in opposite direction of movement
                double lateralDirection = Math.signum(robotVx * shotDirY - robotVy * shotDirX);
                targetTurretDeg += lateralDirection * leadAngle * 0.5; // 0.5 = fudge factor, tune this
            }

        } else {
            currentTurretDeg = turret.getCurrentPosition() * TURRET_DEG_PER_TICK + startTurretAngle;
            targetTurretDeg = startTurretAngle;
        }

        // Normalize and apply limits
        currentTurretDeg = normalizeAngle(currentTurretDeg);

        // Apply turret angle limits
        if(targetTurretDeg < 0 && targetTurretDeg > LEFT_LIMIT){
            targetTurretDeg = LEFT_LIMIT;
        } else if (targetTurretDeg >= 0 && targetTurretDeg < RIGHT_LIMIT) {
            targetTurretDeg = RIGHT_LIMIT;
        }

        // Calculate error with shortest path
        double error = normalizeAngle(targetTurretDeg - currentTurretDeg);

        // PID control
        double power = error * kP;
        turretOnTarget = Math.abs(error) <= TURRET_TOL_DEG;
        power = Range.clip(power, -MAX_POWER_TURRET, MAX_POWER_TURRET);
        turret.setPower(power);
    }
    private void computeParameters() {
        double d = Math.hypot(targetX - turretX, targetY - turretY) * 0.0254;

        if (d <= 0) {
            flywheelTargetRPM = 2000;
            trajectoryAngle = maxTrajectoryAngle;
            return;
        }

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

            flywheelTargetRPM = (int)(60 * exitVelocity /
                    (2 * Math.PI * flywheelRadius * launcherEfficiency));

        } else {
            trajectoryAngle = idealAngle;

            double thetaRad = Math.toRadians(trajectoryAngle);
            double sinTheta = Math.sin(thetaRad);

            if (Math.abs(sinTheta) < 1e-6) {
                return;
            }

            double v0 = Math.sqrt(2 * g * preferredMaxHeightThrow) / sinTheta;

            flywheelTargetRPM = (int)(60 * v0 / (2 * Math.PI * flywheelRadius * launcherEfficiency));
        }

        flywheelTargetRPM = Math.max(minFlywheelRPM, Math.min(flywheelTargetRPM, maxFlywheelRPM));
    }
//    private void updateTurretAim() {
//        disableIfNotInLaunchZone();
//        double targetTurretDeg;
//        double currentTurretDeg;
//        if (aimingEnabled) {
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
//            currentTurretDeg = turret.getCurrentPosition() * TURRET_DEG_PER_TICK + startTurretAngle;
//            targetTurretDeg = normalizeAngle(fieldAngle - robotHeadingDeg);
//        } else {
//            currentTurretDeg = turret.getCurrentPosition() * TURRET_DEG_PER_TICK + startTurretAngle;
//            targetTurretDeg = startTurretAngle;
//        }
//        currentTurretDeg = normalizeAngle(currentTurretDeg);
//        if (targetTurretDeg < 0 && targetTurretDeg > LEFT_LIMIT) {
//            targetTurretDeg = LEFT_LIMIT;
//        } else if (targetTurretDeg >= 0 && targetTurretDeg < RIGHT_LIMIT) {
//            targetTurretDeg = RIGHT_LIMIT;
//        }
//
//        double error = normalizeAngle(targetTurretDeg - currentTurretDeg);
//
//        double power = error * kP;
//        turretOnTarget = Math.abs(error) <= TURRET_TOL_DEG;
//        power = Range.clip(power, -MAX_POWER_TURRET, MAX_POWER_TURRET);
//        turret.setPower(power);
//    }
        private void updateTurretAimRunToPosition() {
        disableIfNotInLaunchZone();

        if(aimingEnabled) {
            double robotX = pose.getX();
            double robotY = pose.getY();
            double robotHeading = pose.getHeading();
            double robotHeadingDeg = Math.toDegrees(robotHeading);

            // Calculate turret position on field
            turretX = robotX + turretOffsetX * Math.cos(robotHeading) - turretOffsetY * Math.sin(robotHeading);
            turretY = robotY + turretOffsetX * Math.sin(robotHeading) + turretOffsetY * Math.cos(robotHeading);

            // Calculate angle to target
            double dx = targetX - turretX;
            double dy = targetY - turretY;
            double fieldAngle = Math.toDegrees(Math.atan2(dy, dx));

            // Desired turret angle (relative to robot)
            double desiredTurretDeg = normalizeAngle(fieldAngle - robotHeadingDeg);

            // Get current position
            int currentPosition = turret.getCurrentPosition();
            double currentTurretDeg = currentPosition * TURRET_DEG_PER_TICK + startTurretAngle;
            currentTurretDeg = normalizeAngle(currentTurretDeg);

            // Calculate shortest path error
            double error = normalizeAngle(desiredTurretDeg - currentTurretDeg);

            // Check if shortest path would hit the limits
            double testAngle = normalizeAngle(currentTurretDeg + error/2); // Midpoint of path
            boolean pathThroughForbidden = (testAngle < 0 && testAngle > LEFT_LIMIT) ||
                    (testAngle >= 0 && testAngle < RIGHT_LIMIT);

            if (pathThroughForbidden) {
                // Shortest path is forbidden - go the long way around
                // Force it to go the opposite direction by setting an intermediate target
                if (error > 0) {
                    // Wanted to go positive, but that's blocked - go negative instead
                    targetTurretDeg = normalizeAngle(currentTurretDeg - 180); // Go 180° the other way
                } else {
                    // Wanted to go negative, but that's blocked - go positive instead
                    targetTurretDeg = normalizeAngle(currentTurretDeg + 180); // Go 180° the other way
                }
            } else {
                // Shortest path is safe
                targetTurretDeg = desiredTurretDeg;
            }

            // Convert target angle to encoder ticks
            int targetPosition = (int)((targetTurretDeg - startTurretAngle) / TURRET_DEG_PER_TICK);

            // Set the target position
            turret.setTargetPosition(targetPosition);

            turret.setPower(MAX_POWER_TURRET);

        } else {
            turret.setPower(0);
            targetTurretDeg = startTurretAngle;
        }

        // Check if on target
        int currentPosition = turret.getCurrentPosition();
        double currentTurretDeg = currentPosition * TURRET_DEG_PER_TICK + startTurretAngle;
        currentTurretDeg = normalizeAngle(currentTurretDeg);

        double error = normalizeAngle(targetTurretDeg - currentTurretDeg);
        turretOnTarget = (Math.abs(error) <= TURRET_TOL_DEG) && !turret.isBusy();
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
    public void usingVelocityCompensation(boolean usingVelocityCompensation){
        this.usingVelocityCompensation = usingVelocityCompensation;
    }

}
