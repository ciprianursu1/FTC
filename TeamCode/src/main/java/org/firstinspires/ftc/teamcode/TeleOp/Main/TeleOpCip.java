package org.firstinspires.ftc.teamcode.TeleOp.Main;

import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp (name = "Cip", group = "Alpha")
public class TeleOpCip extends OpMode {
    DCSpindexer spinner;
    DcMotorEx turret;
    DcMotorEx flywheel;
    DcMotorSimple intake;
    Servo trajectoryAngleModifier;
    DcMotor front_left;
    DcMotor front_right;
    DcMotor back_left;
    DcMotor back_right;
    Limelight3A limelight;
    PinpointLocalizer pinpoint;
    Pose startPose;
    Pose pose;
    IMU imu;
    static final double INCH_PER_METER = 100/2.54;
    static final double FLYWHEEL_TICKS_PER_REV = 28;
    static final double targetX = 0;
    static final double targetY = 144;
    static final double minFlywheelRPM = 1000;
    static final double maxFlywheelRPM = 6000;
    static final double maxTrajectoryAngle = 70;
    static final double minTrajectoryAngle = 50.2;
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
    private static final double LL_OFFSET_X = -65.5/1000;
    private static final double LL_OFFSET_Y = 181/1000.0;
    private long rpmInRangeSinceMs = 0;
    boolean turretOnTarget = false;
    enum IntakeState{
        ON,OFF,REVERSE
    }
    IntakeState intakeState = IntakeState.OFF;
    IntakeState prevIntakeState = IntakeState.OFF;


    public void init() {
        turret = hardwareMap.get(DcMotorEx.class, "tureta");
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(14, 2.3, 4, 14.45);
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        trajectoryAngleModifier = hardwareMap.get(Servo.class, "unghituretaoy");
        spinner = new DCSpindexer(hardwareMap,"Color1","Color2","Color3","spinner","ejector");
        intake = hardwareMap.get(DcMotorSimple.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        front_left = hardwareMap.dcMotor.get("lf");
        front_right = hardwareMap.dcMotor.get("rf");
        back_left = hardwareMap.dcMotor.get("lr");
        back_right = hardwareMap.dcMotor.get("rr");
        pinpoint = new PinpointLocalizer(hardwareMap, Constants.localizerConstants);
        startPose = new Pose(64.3, 15.74/2.0, Math.toRadians(90));
        pinpoint.setStartPose(startPose);
        front_right.setDirection(DcMotorSimple.Direction.FORWARD);
        back_right.setDirection(DcMotorSimple.Direction.FORWARD);
        front_left.setDirection(DcMotorSimple.Direction.REVERSE);
        back_left.setDirection(DcMotorSimple.Direction.REVERSE);
        spinner.setMotif(21);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        );
        imu.initialize(new IMU.Parameters(orientation));
        limelight.pipelineSwitch(4);
        limelight.start();
    }
    public void init_loop() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.getFiducialResults() != null) {
            int tagID = result.getFiducialResults().get(0).getFiducialId();
            if (tagID == 21 || tagID == 22 || tagID == 23) {
                spinner.setMotif(tagID);
            }
        }
    }
    public void start(){
        limelight.pipelineSwitch(5);
    }
    public void loop(){
        pinpoint.update();
        pose = pinpoint.getPose();
        velocity = pinpoint.getVelocity();
        LLResult result = limelight.getLatestResult();
        if (result != null && result.getBotpose_MT2() != null) {
            Pose3D LLPose = result.getBotpose_MT2();
            Pose pedroPose = PoseConverter.pose2DToPose(new Pose2D(DistanceUnit.INCH,LLPose.getPosition().x*INCH_PER_METER,LLPose.getPosition().y*INCH_PER_METER,AngleUnit.DEGREES,LLPose.getOrientation().getYaw()), InvertedFTCCoordinates.INSTANCE);
            pedroPose = processPedroPose(pedroPose);
            pinpoint.setPose(pedroPose);
        }
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(normalizeAngle(orientation.getYaw() - Math.toDegrees(startPose.getHeading())));
        pinpoint.setHeading(normalizeAngle(orientation.getYaw(AngleUnit.RADIANS) - startPose.getHeading()));
        Drive();
        if(aimingEnabled || spinner.spindexerFull()) {
            enableLauncher();
            if(aimingEnabled) computeParameters();
        } else {
            disableLauncher();
        }
        updateLauncher();
        updateTrajectoryAngle();
        updateTurretAim();
        switch (intakeState){
            case ON:
                intake.setPower(1);
                break;
            case OFF:
                intake.setPower(0);
                break;
            case REVERSE:
                intake.setPower(-1);
                break;
        }
        if(spinner.requestingOuttake){
            spinner.setReady(rpmInRangeStable() && turretOnTarget);
        }
        spinner.update();
        if(gamepad2.circleWasPressed()){
            spinner.cancelOuttake();
        }
        if(gamepad2.triangleWasPressed() && intakeState != IntakeState.REVERSE){
            intakeState = intakeState == IntakeState.ON ? IntakeState.OFF : IntakeState.ON;
        }
        if(gamepad2.right_trigger > 0.8){
            spinner.requestOuttake();
        }
        if(gamepad2.cross && intakeState != IntakeState.REVERSE){
            prevIntakeState = intakeState;
            intakeState = IntakeState.REVERSE;
        } else {
            intakeState = prevIntakeState;
        }
        if(gamepad2.optionsWasPressed() && gamepad2.shareWasPressed()){
            pinpoint.setPose(startPose);
            gamepad2.rumble(100);
        }
        if(gamepad2.touchpadWasPressed()){
            spinner.enabledSorting = !spinner.enabledSorting;
            gamepad2.rumbleBlips(spinner.enabledSorting ? 2 : 1);
        }
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

    private double getSpeedTowardTarget(double rawDistance) {
        double robotVx = getRobotVelocityX();
        double robotVy = getRobotVelocityY();

        // ========== CALCULATE EFFECTIVE DISTANCE WITH MOVEMENT COMPENSATION ==========
        // Direction to target
        double dx = targetX - turretX;
        double dy = targetY - turretY;
        double targetDirX = dx / rawDistance;
        double targetDirY = dy / rawDistance;

        // Robot speed toward/away from target
        return robotVx * targetDirX + robotVy * targetDirY;
    }

    private Pose processPedroPose(Pose pedroPose){
        double x = pedroPose.getX();
        double y = pedroPose.getY();
        double heading = pedroPose.getHeading();
        double currentTurretAngle = turret.getCurrentPosition() * TURRET_DEG_PER_TICK + startTurretAngle;
        heading = normalizeAngle(heading - currentTurretAngle);
        x += (Math.cos(heading) * LL_OFFSET_X - Math.sin(heading) * LL_OFFSET_Y)*INCH_PER_METER;
        y += (Math.sin(heading) * LL_OFFSET_X + Math.cos(heading) * LL_OFFSET_Y)*INCH_PER_METER;
        x -= (turretOffsetX * Math.cos(heading) - turretOffsetY * Math.sin(heading))*INCH_PER_METER;
        y -= (turretOffsetX * Math.sin(heading) + turretOffsetY * Math.cos(heading))*INCH_PER_METER;
        pedroPose = new Pose(x,y,heading);
        return pedroPose;
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
            double robotVx = getRobotVelocityX();
            double robotVy = getRobotVelocityY();

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
        turretOnTarget = Math.abs(error) <= TURRET_TOL_DEG;
        power = Range.clip(power, -MAX_POWER_TURRET, MAX_POWER_TURRET);
        turret.setPower(power);
    }
//    private void updateTurretAimRunToPosition() {
//        disableIfNotInLaunchZone();
//
//        if(aimingEnabled) {
//            double robotX = pose.getX();
//            double robotY = pose.getY();
//            double robotHeading = pose.getHeading();
//            double robotHeadingDeg = Math.toDegrees(robotHeading);
//
//            // Calculate turret position on field
//            turretX = robotX + turretOffsetX * Math.cos(robotHeading) - turretOffsetY * Math.sin(robotHeading);
//            turretY = robotY + turretOffsetX * Math.sin(robotHeading) + turretOffsetY * Math.cos(robotHeading);
//
//            // Calculate angle to target
//            double dx = targetX - turretX;
//            double dy = targetY - turretY;
//            double fieldAngle = Math.toDegrees(Math.atan2(dy, dx));
//
//            // Desired turret angle (relative to robot)
//            double desiredTurretDeg = normalizeAngle(fieldAngle - robotHeadingDeg);
//
//            // Get current position
//            int currentPosition = turret.getCurrentPosition();
//            double currentTurretDeg = currentPosition * TURRET_DEG_PER_TICK + startTurretAngle;
//            currentTurretDeg = normalizeAngle(currentTurretDeg);
//
//            // Calculate shortest path error
//            double error = normalizeAngle(desiredTurretDeg - currentTurretDeg);
//
//            // Check if shortest path would hit the limits
//            double testAngle = normalizeAngle(currentTurretDeg + error/2); // Midpoint of path
//            boolean pathThroughForbidden = (testAngle < 0 && testAngle > LEFT_LIMIT) ||
//                    (testAngle >= 0 && testAngle < RIGHT_LIMIT);
//
//            if (pathThroughForbidden) {
//                // Shortest path is forbidden - go the long way around
//                // Force it to go the opposite direction by setting an intermediate target
//                if (error > 0) {
//                    // Wanted to go positive, but that's blocked - go negative instead
//                    targetTurretDeg = normalizeAngle(currentTurretDeg - 180); // Go 180° the other way
//                } else {
//                    // Wanted to go negative, but that's blocked - go positive instead
//                    targetTurretDeg = normalizeAngle(currentTurretDeg + 180); // Go 180° the other way
//                }
//            } else {
//                // Shortest path is safe
//                targetTurretDeg = desiredTurretDeg;
//            }
//
//            // Convert target angle to encoder ticks
//            int targetPosition = (int)((targetTurretDeg - startTurretAngle) / TURRET_DEG_PER_TICK);
//
//            // Set the target position
//            turret.setTargetPosition(targetPosition);
//
//            turret.setPower(MAX_POWER_TURRET);
//
//        } else {
//            turret.setPower(0);
//        }
//
//        // Check if on target
//        int currentPosition = turret.getCurrentPosition();
//        double currentTurretDeg = currentPosition * TURRET_DEG_PER_TICK + startTurretAngle;
//        currentTurretDeg = normalizeAngle(currentTurretDeg);
//
//        double error = normalizeAngle(targetTurretDeg - currentTurretDeg);
//        turretOnTarget = (Math.abs(error) <= TURRET_TOL_DEG) && !turret.isBusy();
//    }

// ========== HELPER FUNCTIONS YOU NEED TO ADD ==========
    Pose velocity;

    private double getRobotVelocityX() {
        return velocity.getX();
    }

    private double getRobotVelocityY() {
        return velocity.getY();
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
    private void updateLauncher(){
        double ticksPerSecond;
        if(launcherEnabled){
            ticksPerSecond = flywheelTargetRPM * FLYWHEEL_TICKS_PER_REV / 60.0;
        } else {
            ticksPerSecond = 2000 * FLYWHEEL_TICKS_PER_REV / 60.0;
        }
        flywheel.setVelocity(ticksPerSecond);
        rpm = flywheel.getVelocity() / FLYWHEEL_TICKS_PER_REV * 60.0;
    }

    private void updateTrajectoryAngle(){
        setTrajectoryAngle(trajectoryAngle);
    }



    private void setTrajectoryAngle(double angle){
        angle = Range.clip(angle,minTrajectoryAngle,maxTrajectoryAngle);
        double position = (angle - minTrajectoryAngle)*trajectoryAnglePosPerDegree;
        trajectoryAngleModifier.setPosition(position);
    }

    public void disableIfNotInLaunchZone(){
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

    public void enableLauncher(){
        if(launcherEnabled) return;
        launcherEnabled = true;
        flywheelTargetRPM = 2000;
        updateLauncher();
    }

    public void disableLauncher(){
        if(!launcherEnabled) return;
        launcherEnabled = false;
        flywheelTargetRPM = 2000;
        updateLauncher();
    }

    public void enableAiming(){
        aimingEnabled = true;
    }

    public void disableAiming(){
        aimingEnabled = false;
    }
    private void Drive() {
        double left_x = gamepad2.left_stick_x;
        double left_y = -gamepad2.left_stick_y;
        double right_x = gamepad2.right_stick_x;

        double front_left_pw  = left_y + left_x + right_x;
        double back_left_pw   = left_y - left_x + right_x;
        double front_right_pw = left_y - left_x - right_x;
        double back_right_pw  = left_y + left_x - right_x;

        double max = Math.max(Math.abs(front_left_pw),
                Math.max(Math.abs(back_left_pw),
                        Math.max(Math.abs(front_right_pw), Math.abs(back_right_pw))));
        if (max > 1.0) {
            front_left_pw  /= max;
            back_left_pw   /= max;
            front_right_pw /= max;
            back_right_pw  /= max;
        }

        front_left.setPower(front_left_pw);
        back_left.setPower(back_left_pw);
        front_right.setPower(front_right_pw);
        back_right.setPower(back_right_pw);
    }
}
