package org.firstinspires.ftc.teamcode.TeleOp.Main;

import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.Arrays;

@Autonomous(name = "Auto_Far_Blue_v1_Stefan", group = "Test")
public class StefanVibeCode extends OpMode {

    /* ===================== TIMING / GATING ===================== */
    static final long DETECT_DELAY_MS = 0;       // immediate once stable
    static final long SERVO_MOVE_LOCK_MS = 45;   // lock while spindexer moves
    private long colorStartTimeMs = 0;
    private long servoMoveStartMs = 0;

    /* ===================== COLOR SMOOTHING (SLOT SENSORS) ===================== */
    private final int[] last5Sensor1 = new int[5];
    private final int[] last5Sensor2 = new int[5];
    private final int[] last5Sensor3 = new int[5];
    private int indexSensor1 = 0, indexSensor2 = 0, indexSensor3 = 0;

    /* ===================== ULTRA-FAST INTAKE SMOOTHING ===================== */
    private final int[] lastNIntake = new int[5];
    private int idxIntake = 0;

    /* ===================== INVENTORY ===================== */
    private final int[] logicalSlots = new int[3];
    private int slotIntakeIndex = 0;

    private boolean spinnerMoving = false;
    private boolean detectionLocked = false;
    private boolean colorPending = false;
    private boolean waitingForClear = false;
    private int lastStableIntakeColor = 0;

    /* ===================== MODES ===================== */
    private boolean intakeMode = false;
    private boolean outtakeMode = false;
    private boolean spinIntake = false;

    private boolean aimingEnabled = false;     // turret aims at target only when enabled + in launch zone
    private boolean launcherEnabled = false;   // flywheel velocity control enabled
    private boolean enabledSorting = false;

    /* ===================== HARDWARE (based strictly on your TeleOp naming) ===================== */
    private ColorSensor colorsensorSLot1, colorsensorSLot2, colorsensorSLot3;
    private DcMotor intake;
    private DcMotorEx tureta;
    private DcMotorEx flywheel;
    private Servo ejector;
    private Servo trajectoryAngleModifier;
    private Servo spinnerCLose, spinnerFar;

    /* ===================== PEDRO ===================== */
    Follower follower;
    Paths paths;

    int stage = 0;
    boolean pathStarted = false;

    /* ===================== LOCALIZATION (MATCH TELEOP) ===================== */
    private PinpointLocalizer pinpoint;
    private Pose pose;
    private Pose startPose;

    /* ===================== EJECTOR ===================== */
    private final double ejectorDown = 0.18;
    private final double ejectorUp   = 0.02;

    /* ===================== SPINNER POSITIONS ===================== */
    private final double[] slotPositionsIntake = {0.0, 0.19, 0.38};

    private static final double OUTTAKE_POS_1 = 0.095;
    private static final double OUTTAKE_POS_2 = 0.285;
    private static final double OUTTAKE_POS_3 = 0.475;

    private static final double OUTTAKE_INITIAL_DELAY_MS = 150;
    private static final double OUTTAKE_EJECTOR_UP_MS     = 250;
    private static final double OUTTAKE_EJECTOR_DOWN_MS   = 250;
    private static final double OUTTAKE_SPINNER_MOVE_MS   = 150;

    private int outtakeStep = 0;
    private long stepStartMs = 0;

    /* ===================== SPINNER SERVO CENTERING + OVERSHOOT ===================== */
    private static final double SPINNER_FAR_OFFSET   = -0.010;
    private static final double SPINNER_CLOSE_OFFSET = -0.010;

    private static final double SPINNER_OVERSHOOT = 0.010;
    private static final double SPINNER_OVERSHOOT_TIME_S = 0.06;
    private static final double SPINNER_TARGET_EPS = 0.0015;

    private double spinnerBaseTarget = 0.0;
    private double spinnerTarget = 0.0;
    private double spinnerCmd = 0.0;
    private double spinnerLastTarget = 0.0;

    private boolean spinnerInOvershoot = false;
    private double spinnerOvershootCmd = 0.0;
    private final ElapsedTime spinnerMoveTimer = new ElapsedTime();

    private static final double SPINNER_TRIM_MIN = 0.00;
    private static final double SPINNER_TRIM_MAX = 0.20;
    private static final double SPINNER_TRIM_RATE_PER_S = 0.06;
    private double spinnerTrim = 0.0;
    private final ElapsedTime trimTimer = new ElapsedTime();

    /* ===================== FLYWHEEL VELOCITY CONTROL ===================== */
    static final double FLYWHEEL_TICKS_PER_REV = 28.0;

    static final double kP_v = 12.0;
    static final double kI_v = 0.0;
    static final double kD_v = 0.0;
    static final double kF_v = 14.0;

    static final double KICK_POWER = 1.0;
    static final double KICK_TIME_S = 0.20;

    private boolean kicking = false;
    private final ElapsedTime flyKickTimer = new ElapsedTime();

    private static final double RPM_TOL = 150;
    private static final long RPM_STABLE_MS = 80;
    private long rpmInRangeSinceMs = 0;
    private double rpm = 0.0;

    /* ===================== TURRET AIM (FIXED) ===================== */
    private static final double MOTOR_TICKS_PER_REV = 384.5;
    private static final double MOTOR_TO_TURRET_RATIO = 75.0 / 26.0;

    private static final double DEG_PER_TICK_TURETA =
            360.0 / (MOTOR_TICKS_PER_REV * MOTOR_TO_TURRET_RATIO);

    private static final double LEFT_LIMIT  = -110;
    private static final double RIGHT_LIMIT = 110;

    // Keep your proportional controller, but make it actually converge
    private static final double kP_t = 0.015;
    private static final double kD_t = 0.000;         // leave 0 by default
    private static final double MAX_POWER_TURETA = 0.2;
    private static final double STATIC_FF = 0.03;      // helps overcome stiction (tune 0.02..0.06)

    // Geometry (inches in field frame, offsets in inches too)
    private final double turretOffsetX = 0.0;
    private final double turretOffsetY = 52.0 / 1000.0;
    private final double startTurretAngle = -180.0;

    // Target (inches)
    private double targetX = 10;
    private double targetY = 137.5;

    private double turretX = 0.0, turretY = 0.0;

    // Debug/state
    private double fieldAngleDeg = 0.0;
    private double rawTargetDeg = 0.0;
    private double targetTurretDeg = 0.0;
    private double currentTurretDeg = 0.0;
    private double errorDeg = 0.0;
    private double turretPower = 0.0;

    // Derivative on error (optional)
    private double lastErrorDeg = 0.0;
    private long lastTurretUpdateMs = 0;

    /* ===================== TRAJECTORY / AUTO COMPUTE ===================== */
    private double trajectoryAngle = 70;

    private final double absoluteTurretHeight = 0.25;
    private final double absoluteTargetHeight = 0.9;
    private final double relativeHeight = absoluteTargetHeight - absoluteTurretHeight;
    private final double prefferedMaxHeightThrow = relativeHeight + 0.3;

    private final double launcherEfficiency = 0.43;
    private final double flywheelRadius = 0.048;
    private final double g = 9.81;

    private final double trajectoryAngleModifierGearRatio = 127 / 15.0;
    private final double trajectoryAnglerMaxTravel = 300.0;
    private final double trajectoryAnglePosPerDegree = trajectoryAngleModifierGearRatio / trajectoryAnglerMaxTravel;
    private final double minTrajectoryAngle = 50.2;
    private final double maxTrajectoryAngle = 70;

    private final int minFlywheelRPM = 1000;
    private final int maxFlywheelRPM = 6000;
    private int flywheelTargetRPM = 0;

    /* ===================== LAUNCH ZONE ===================== */
    private final double[][] launchZoneBig = {{-18,144},{162,144},{72,55}};
    private final double[][] launchZoneSmall = {{40,0},{102,0},{72,32}};

    /* ===================== SETTLE GATE (STOP SHOOTING WHILE SLIDING) ===================== */
    private long settleStartMs = 0;
    private static final long SETTLE_MS = 200;
    private static final double POS_EPS_IN = 1.5;     // inches
    private static final double HEAD_EPS_DEG = 3.0;   // degrees

    // shoot pose (your path ends at 57,21,299deg for all 3 shoots)
    private static final double SHOOT_X = 57.0;
    private static final double SHOOT_Y = 21.0;
    private static final double SHOOT_H = Math.toRadians(299);

    /* ===================== AUTO MODES ===================== */
    private boolean launchPrepActive = false;

    // latch to block until outtake finishes
    private boolean shootRequested = false;

    /* ===================== TELEMETRY THROTTLE ===================== */
    private final int telemetryDelay = 200;
    private final ElapsedTime telemetryTimer = new ElapsedTime();

    /* ===================== 1s STABILIZE WAITS ===================== */
    private static final long STABILIZE_WAIT_MS = 1000;
    private long stabilizeStartMs = 0;

    /* ========================================================================================= */

    private void startStep(int newStep) {
        outtakeStep = newStep;
        stepStartMs = System.currentTimeMillis();
    }

    private void setSpinnerTarget(double target) {
        spinnerBaseTarget = Range.clip(target, 0.0, 1.0);
    }

    private void updateSpinnerTrim() {
        double dt = trimTimer.seconds();
        trimTimer.reset();

        // Remove gamepad controls for autonomous
        spinnerTrim = Range.clip(spinnerTrim, SPINNER_TRIM_MIN, SPINNER_TRIM_MAX);
    }

    private boolean atPose(Pose p, double x, double y, double headingRad) {
        if (p == null) return false;
        double posErr = Math.hypot(p.getX() - x, p.getY() - y);
        double headErrDeg = Math.toDegrees(p.getHeading() - headingRad);
        while (headErrDeg > 180) headErrDeg -= 360;
        while (headErrDeg < -180) headErrDeg += 360;
        headErrDeg = Math.abs(headErrDeg);
        return posErr <= POS_EPS_IN && headErrDeg <= HEAD_EPS_DEG;
    }

    private boolean settledAt(Pose p, double x, double y, double headingRad) {
        if (!atPose(p, x, y, headingRad)) {
            settleStartMs = 0;
            return false;
        }
        long now = System.currentTimeMillis();
        if (settleStartMs == 0) settleStartMs = now;
        return (now - settleStartMs) >= SETTLE_MS;
    }

    private void startStabilizeWait() {
        stabilizeStartMs = System.currentTimeMillis();
    }

    private boolean stabilizeDone() {
        return (System.currentTimeMillis() - stabilizeStartMs) >= STABILIZE_WAIT_MS;
    }

    private double normalizeAngleDeg(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    private double sign(double x1,double y1,double x2,double y2,double x3,double y3){
        return (x1-x3)*(y2-y3)-(x2-x3)*(y1-y3);
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
        } else {
            d1 = sign(robotX, robotY, launchZoneSmall[0][0], launchZoneSmall[0][1], launchZoneSmall[1][0], launchZoneSmall[1][1]);
            d2 = sign(robotX, robotY, launchZoneSmall[1][0], launchZoneSmall[1][1], launchZoneSmall[2][0], launchZoneSmall[2][1]);
            d3 = sign(robotX, robotY, launchZoneSmall[2][0], launchZoneSmall[2][1], launchZoneSmall[0][0], launchZoneSmall[0][1]);
        }

        has_neg = d1 < 0 || d2 < 0 || d3 < 0;
        has_pos = d1 > 0 || d2 > 0 || d3 > 0;

        aimingEnabled = !(has_neg && has_pos);
    }

    /* ===================== SPINNER TARGET + TRIM + OVERSHOOT ===================== */
    private void updateSpinnerServos() {
        spinnerTarget = Range.clip(spinnerBaseTarget + spinnerTrim, 0.0, 1.0);

        if (Math.abs(spinnerTarget - spinnerLastTarget) > SPINNER_TARGET_EPS) {
            spinnerLastTarget = spinnerTarget;

            double dir = Math.signum(spinnerTarget - spinnerCmd);
            if (dir == 0) dir = 1.0;

            spinnerOvershootCmd = Range.clip(spinnerTarget + dir * SPINNER_OVERSHOOT, 0.0, 1.0);
            spinnerInOvershoot = true;
            spinnerMoveTimer.reset();
        }

        if (spinnerInOvershoot) {
            spinnerCmd = spinnerOvershootCmd;
            if (spinnerMoveTimer.seconds() >= SPINNER_OVERSHOOT_TIME_S) {
                spinnerInOvershoot = false;
                spinnerCmd = spinnerTarget;
            }
        } else {
            spinnerCmd = spinnerTarget;
        }

        double farPos = Range.clip(spinnerCmd + SPINNER_FAR_OFFSET, 0.0, 1.0);
        double closePos = Range.clip(spinnerCmd + SPINNER_CLOSE_OFFSET, 0.0, 1.0);

        spinnerFar.setPosition(farPos);
        spinnerCLose.setPosition(closePos);
    }

    /* ===================== FLYWHEEL ===================== */
    private double getFlywheelRPM() {
        return flywheel.getVelocity() / FLYWHEEL_TICKS_PER_REV * 60.0;
    }

    private boolean rpmInRangeStable() {
        boolean inRange = (rpm >= (flywheelTargetRPM - RPM_TOL)) && (rpm <= (flywheelTargetRPM + 150));
        long now = System.currentTimeMillis();

        if (!inRange) {
            rpmInRangeSinceMs = 0;
            return false;
        }
        if (rpmInRangeSinceMs == 0) rpmInRangeSinceMs = now;
        return (now - rpmInRangeSinceMs) >= RPM_STABLE_MS;
    }

    private void enableLauncher() {
        if (launcherEnabled) return;
        launcherEnabled = true;
        kicking = true;
        flyKickTimer.reset();
    }

    private void disableLauncher() {
        if (!launcherEnabled) return;
        launcherEnabled = false;
        kicking = false;
        flywheel.setPower(0);
        flywheelTargetRPM = 0;
    }

    private void updateLauncher() {
        if (!launcherEnabled) {
            flywheel.setPower(0);
            rpm = getFlywheelRPM();
            return;
        }

        if (kicking) {
            flywheel.setPower(KICK_POWER);
            if (flyKickTimer.seconds() >= KICK_TIME_S) kicking = false;
        } else {
            double tps = flywheelTargetRPM * FLYWHEEL_TICKS_PER_REV / 60.0;
            flywheel.setVelocity(tps);
        }

        rpm = getFlywheelRPM();
    }

    /* ===================== TRAJECTORY ===================== */
    private void setTrajectoryAngle(double angle){
        angle = Range.clip(angle, minTrajectoryAngle, maxTrajectoryAngle);
        double position = (angle - minTrajectoryAngle) * trajectoryAnglePosPerDegree;
        trajectoryAngleModifier.setPosition(Range.clip(position, 0.0, 1.0));
    }

    private void computeParameters() {
        double robotHeading = pose.getHeading();
        double robotX = pose.getX();
        double robotY = pose.getY();

        turretX = robotX + turretOffsetX*Math.cos(robotHeading) - turretOffsetY*Math.sin(robotHeading);
        turretY = robotY + turretOffsetX*Math.sin(robotHeading) + turretOffsetY*Math.cos(robotHeading);

        double d = Math.hypot(targetX - turretX, targetY - turretY) * 0.0254;

        if (d <= 1e-6) {
            flywheelTargetRPM = 2000;
            trajectoryAngle = maxTrajectoryAngle;
            return;
        }

        double k = (4.0 * prefferedMaxHeightThrow / d)
                * (1.0 - Math.sqrt(1.0 - relativeHeight / prefferedMaxHeightThrow));

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

            double exitVelocity = Math.sqrt((g * d * d) / (2 * cosTheta * cosTheta * denominator));
            flywheelTargetRPM = (int)(60 * exitVelocity / (2 * Math.PI * flywheelRadius * launcherEfficiency));
        } else {
            trajectoryAngle = idealAngle;

            double thetaRad = Math.toRadians(trajectoryAngle);
            double sinTheta = Math.sin(thetaRad);
            if (Math.abs(sinTheta) < 1e-6) return;

            double v0 = Math.sqrt(2 * g * prefferedMaxHeightThrow) / sinTheta;
            flywheelTargetRPM = (int)(60 * v0 / (2 * Math.PI * flywheelRadius * launcherEfficiency));
        }

        flywheelTargetRPM = Math.max(minFlywheelRPM, Math.min(flywheelTargetRPM, maxFlywheelRPM));
        setTrajectoryAngle(trajectoryAngle);
    }

    /* ===================== TURRET AIM (ACTUALLY CORRECT NOW) ===================== */
    private void updateTurretAim() {
        double robotX = pose.getX();
        double robotY = pose.getY();
        double robotHeading = pose.getHeading();
        double robotHeadingDeg = Math.toDegrees(robotHeading);
        turretX = robotX + turretOffsetX*Math.cos(robotHeading) - turretOffsetY*Math.sin(robotHeading);
        turretY = robotY + turretOffsetX*Math.sin(robotHeading) + turretOffsetY*Math.cos(robotHeading);
        double dx = targetX - turretX;
        double dy = targetY - turretY;

        double fieldAngle = Math.toDegrees(Math.atan2(dy, dx));

        currentTurretDeg = tureta.getCurrentPosition() * DEG_PER_TICK_TURETA + startTurretAngle;
        if(aimingEnabled) {
            targetTurretDeg = normalizeAngleDeg(fieldAngle - robotHeadingDeg);
        } else {
            targetTurretDeg = startTurretAngle;
        }
        currentTurretDeg=normalizeAngleDeg(currentTurretDeg);
        if(targetTurretDeg < 0 && targetTurretDeg > LEFT_LIMIT){
            targetTurretDeg = LEFT_LIMIT;
        } else if (targetTurretDeg >= 0 && targetTurretDeg < RIGHT_LIMIT) {
            targetTurretDeg = RIGHT_LIMIT;
        }

        double error = normalizeAngleDeg(targetTurretDeg - currentTurretDeg);

        double power = error * kP_t;
        power = Range.clip(power, -MAX_POWER_TURETA, MAX_POWER_TURETA);
        tureta.setPower(power);
    }

    /* ===================== COLOR HELPERS ===================== */
    private double getHue(int r, int g, int b) {
        int max = Math.max(r, Math.max(g, b));
        int min = Math.min(r, Math.min(g, b));
        if (max == min) return 0.0;

        double chroma = max - min;
        double h;

        if (max == r) h = (double) (g - b) / chroma;
        else if (max == g) h = (double) (b - r) / chroma + 2.0;
        else h = (double) (r - g) / chroma + 4.0;

        h *= 60.0;
        if (h < 0) h += 360.0;
        return h;
    }

    private int smekerie1(ColorSensor colorSensor) {
        int r = colorSensor.red();
        int g = colorSensor.green();
        int b = colorSensor.blue();
        int alpha = colorSensor.alpha();
        double h = getHue(r, g, b);
        int detected;

        if (alpha < 100 && (h == 150 || h == 144)) detected = 0;
        else if ((h > 215) || (alpha < 100 && (h == 160 || h == 180))) detected = 2;
        else if (h > 135 && h < 160 && alpha > 100) detected = 1;
        else if ((h == 140 || h == 145) && alpha == 43) detected = 0;
        else if (h > 135 && h < 160 && alpha > 60) detected = 1;
        else if ((h == 210 || h == 220 || h == 225 || h == 200) && alpha < 100) detected = 2;
        else detected = 0;

        return detected;
    }

    private int CuloareFinala1(ColorSensor sensor, int[] last5, int index) {
        last5[index] = smekerie1(sensor);

        int count1 = 0, count2 = 0;
        for (int v : last5) {
            if (v == 1) count1++;
            else if (v == 2) count2++;
        }

        if (count1 >= 3) return 1;
        if (count2 >= 3) return 2;
        return 0;
    }

    private void updateCuloriLiveDebug() {
        int Color1 = CuloareFinala1(colorsensorSLot1, last5Sensor1, indexSensor1);
        indexSensor1 = (indexSensor1 + 1) % 5;

        int Color2 = CuloareFinala1(colorsensorSLot2, last5Sensor2, indexSensor2);
        indexSensor2 = (indexSensor2 + 1) % 5;

        int Color3 = CuloareFinala1(colorsensorSLot3, last5Sensor3, indexSensor3);
        indexSensor3 = (indexSensor3 + 1) % 5;

        telemetry.addData("Live Color1", Color1);
        telemetry.addData("Live Color2", Color2);
        telemetry.addData("Live Color3", Color3);
    }

    private int processIntakeSensor(ColorSensor sensor) {
        int detected = smekerie1(sensor);

        lastNIntake[idxIntake] = detected;
        idxIntake = (idxIntake + 1) % lastNIntake.length;

        int count1 = 0, count2 = 0;
        for (int v : lastNIntake) {
            if (v == 1) count1++;
            else if (v == 2) count2++;
        }

        if (count1 >= 2 && count1 > count2) return 1;
        if (count2 >= 2 && count2 > count1) return 2;
        return 0;
    }

    private boolean spinnerFull() {
        int count = 0;
        for (int v : logicalSlots) if (v != 0) count++;
        return count >= 3;
    }

    private void resetIntakeGatingAndFilters() {
        waitingForClear = false;
        detectionLocked = false;
        spinnerMoving = false;
        colorPending = false;
        lastStableIntakeColor = 0;

        Arrays.fill(lastNIntake, 0);
        idxIntake = 0;
    }

    private void colorDrivenSpinnerLogicServos() {
        if (spinnerMoving) {
            if (System.currentTimeMillis() - servoMoveStartMs >= SERVO_MOVE_LOCK_MS) {
                spinnerMoving = false;
                detectionLocked = false;
                lastStableIntakeColor = 0;
            } else {
                return;
            }
        }

        if (waitingForClear) {
            int intakeColorNow = processIntakeSensor(colorsensorSLot1);
            if (intakeColorNow == 0) {
                waitingForClear = false;
                lastStableIntakeColor = 0;
            }
            return;
        }

        if (spinnerFull()) {
            intakeMode = false;
            spinIntake = false;
            intake.setPower(0);
            return;
        }

        int intakeColor = processIntakeSensor(colorsensorSLot1);

        boolean newColorDetected = (intakeColor != 0 && lastStableIntakeColor == 0);
        if (newColorDetected) lastStableIntakeColor = intakeColor;

        if (newColorDetected && !colorPending && !detectionLocked) {
            colorStartTimeMs = System.currentTimeMillis();
            colorPending = true;
        }

        if (colorPending && (System.currentTimeMillis() - colorStartTimeMs >= DETECT_DELAY_MS)) {
            logicalSlots[slotIntakeIndex] = intakeColor;

            slotIntakeIndex = (slotIntakeIndex + 1) % 3;
            setSpinnerTarget(slotPositionsIntake[slotIntakeIndex]);

            waitingForClear = true;
            detectionLocked = true;
            spinnerMoving = true;
            servoMoveStartMs = System.currentTimeMillis();

            colorPending = false;
        }
    }

    /* ===================== INTAKE / OUTTAKE ===================== */
    private void runIntake() {
        colorDrivenSpinnerLogicServos();
    }

    private void runOuttake() {
        intake.setPower(1);

        long now = System.currentTimeMillis();
        long dt = now - stepStartMs;

        switch (outtakeStep) {
            case 0:
                Arrays.fill(logicalSlots, 0);
                setSpinnerTarget(OUTTAKE_POS_1);
                rpmInRangeSinceMs = 0;
                startStep(1);
                break;

            case 1:
                if (dt >= OUTTAKE_INITIAL_DELAY_MS) startStep(2);
                break;

            case 2:
                if (rpmInRangeStable()) {
                    ejector.setPosition(ejectorUp);
                    startStep(3);
                }
                break;

            case 3:
                if (dt >= OUTTAKE_EJECTOR_UP_MS) {
                    ejector.setPosition(ejectorDown);
                    startStep(4);
                }
                break;

            case 4:
                if (dt >= OUTTAKE_EJECTOR_DOWN_MS) {
                    setSpinnerTarget(OUTTAKE_POS_2);
                    startStep(5);
                }
                break;

            case 5:
                if (dt >= OUTTAKE_SPINNER_MOVE_MS) {
                    rpmInRangeSinceMs = 0;
                    startStep(6);
                }
                break;

            case 6:
                if (rpmInRangeStable()) {
                    ejector.setPosition(ejectorUp);
                    startStep(7);
                }
                break;

            case 7:
                if (dt >= OUTTAKE_EJECTOR_UP_MS) {
                    ejector.setPosition(ejectorDown);
                    startStep(8);
                }
                break;

            case 8:
                if (dt >= OUTTAKE_EJECTOR_DOWN_MS) {
                    setSpinnerTarget(OUTTAKE_POS_3);
                    startStep(9);
                }
                break;

            case 9:
                if (dt >= OUTTAKE_SPINNER_MOVE_MS) {
                    rpmInRangeSinceMs = 0;
                    startStep(10);
                }
                break;

            case 10:
                if (rpmInRangeStable()) {
                    ejector.setPosition(ejectorUp);
                    startStep(11);
                }
                break;

            case 11:
                if (dt >= OUTTAKE_EJECTOR_UP_MS) {
                    ejector.setPosition(ejectorDown);
                    startStep(12);
                }
                break;

            case 12:
                if (dt >= OUTTAKE_EJECTOR_DOWN_MS) {
                    setSpinnerTarget(0.0);

                    outtakeMode = false;
                    intakeMode = false;

                    spinIntake = false;
                    intake.setPower(0);

                    slotIntakeIndex = 0;
                    resetIntakeGatingAndFilters();

                    outtakeStep = 0;
                    stepStartMs = 0;
                    rpmInRangeSinceMs = 0;
                }
                break;
        }
    }

    private void spinnerMoveRequest(double newPos) {
        if (intake != null) {
            if (Math.abs(intake.getPower()) < 0.05) {
                intake.setPower(1);
            }
        }
        setSpinnerTarget(newPos);
    }

    private void startOuttake() {
        intakeMode = false;
        outtakeMode = true;
        launchPrepActive = false;

        intake.setPower(1);

        outtakeStep = 0;
        stepStartMs = System.currentTimeMillis();
        rpmInRangeSinceMs = 0;

        spinnerMoveRequest(OUTTAKE_POS_1);
        ejector.setPosition(ejectorDown);
    }

    private void autoLaunchPrepLogic() {
        if (outtakeMode) return;

        if (intakeMode && spinnerFull()) {
            launchPrepActive = true;
        }

        if (launchPrepActive) {
            spinnerMoveRequest(OUTTAKE_POS_1);
        }
    }

    /* ===================== INIT / START ===================== */
    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);

        startPose = new Pose(57, 9, Math.toRadians(90));
        pinpoint = new PinpointLocalizer(hardwareMap, Constants.localizerConstants);
        pinpoint.setStartPose(startPose);
        pinpoint.setPose(startPose);

        follower.setStartingPose(startPose);
        paths = new Paths(follower);

        intake = hardwareMap.get(DcMotor.class, "intake");
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");

        tureta = hardwareMap.get(DcMotorEx.class, "tureta");
        trajectoryAngleModifier = hardwareMap.get(Servo.class, "unghituretaoy");

        tureta.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        tureta.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        tureta.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        tureta.setPower(0);

        ejector = hardwareMap.get(Servo.class, "ejector");
        spinnerFar = hardwareMap.get(Servo.class, "SpinnerFar");
        spinnerCLose = hardwareMap.get(Servo.class, "SpinnerClose");

        colorsensorSLot1 = hardwareMap.colorSensor.get("Color1");
        colorsensorSLot2 = hardwareMap.colorSensor.get("Color2");
        colorsensorSLot3 = hardwareMap.colorSensor.get("Color3");

        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setPower(0);

        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setVelocityPIDFCoefficients(kP_v, kI_v, kD_v, kF_v);

        ejector.setDirection(Servo.Direction.REVERSE);
        ejector.setPosition(ejectorDown);

        spinnerTrim = 0.0;
        setSpinnerTarget(0.0);
        spinnerCmd = 0.0;
        spinnerLastTarget = 0.0;
        spinnerInOvershoot = false;
        updateSpinnerServos();

        trajectoryAngle = maxTrajectoryAngle;
        setTrajectoryAngle(trajectoryAngle);

        // preloaded with 3 balls
        logicalSlots[0] = 1;
        logicalSlots[1] = 1;
        logicalSlots[2] = 1;
        slotIntakeIndex = 0;

        // turret tracks all the time
        aimingEnabled = true;

        launcherEnabled = true;
        flywheelTargetRPM = 0;

        stage = 0;
        pathStarted = false;

        intakeMode = false;
        outtakeMode = false;
        spinIntake = false;
        launchPrepActive = false;
        shootRequested = false;

        settleStartMs = 0;

        resetIntakeGatingAndFilters();
    }

    @Override
    public void start() {
        launcherEnabled = true;
        kicking = true;
        flyKickTimer.reset();

        intake.setPower(1);
        spinnerMoveRequest(OUTTAKE_POS_1);
    }

    /* ===================== LOOP ===================== */
    @Override
    public void loop() {
        follower.update();

        // USE PINPOINT POSE FOR TURRET (MATCH TELEOP)
        pinpoint.update();
        pose = pinpoint.getPose();

        // in launch position for ballistics + for shooting
        boolean inLaunchPosition = (stage == 3 || stage == 7 || stage == 11 || outtakeMode);

        // turret tracks target all the time
        aimingEnabled = true;

        // ballistics only when we are actually at/using launch
        boolean ballisticsEnabled = inLaunchPosition;

        // Always update trim and spinner servos
        updateSpinnerTrim();
        updateSpinnerServos();

        // turret always tracks
        if (pose != null) {
            updateTurretAim();

            if (ballisticsEnabled) {
                disableIfNotInLaunchZone();
                if (aimingEnabled) {
                    computeParameters();
                }
                updateLauncher();
            } else {
                disableLauncher();
                updateLauncher();
            }
        } else {
            updateLauncher();
        }

        updateCuloriLiveDebug();

        if (intakeMode && !outtakeMode && !launchPrepActive) {
            runIntake();
        }
        autoLaunchPrepLogic();

        if (outtakeMode) {
            runOuttake();
        }

        switch (stage) {
            case 0:
                stage = 1;
                break;

            case 1: // Path9
                if (!outtakeMode && !pathStarted) {
                    follower.followPath(paths.Path9, 1.0, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    stage = 2;
                }
                break;

            case 2: // Path3 (shoot position)
                if (!outtakeMode && !pathStarted) {
                    follower.followPath(paths.Path3, 1.0, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    settleStartMs = 0; // reset settle timer on entry
                    stage = 3;
                }
                break;

            case 3: // SHOOT #1 (preloaded)  <-- REQUIRE SETTLE
                if (!shootRequested && !outtakeMode) {
                    if (pose != null && settledAt(pose, SHOOT_X, SHOOT_Y, SHOOT_H)) {
                        startOuttake();
                        shootRequested = true;
                    }
                }
                if (shootRequested && !outtakeMode) {
                    shootRequested = false;
                    stage = 4;
                }
                break;

            case 4: // Path1 (positions for intake)
                if (!outtakeMode) {
                    intakeMode = true;
                    spinIntake = true;
                    intake.setPower(1);
                }
                if (!outtakeMode && !pathStarted) {
                    follower.followPath(paths.Path1, 1.0, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    startStabilizeWait();
                    stage = 50;
                }
                break;

            case 50: // WAIT 1s (before Path2)
                if (stabilizeDone()) {
                    stage = 5;
                }
                break;

            case 5: // Path2
                if (!outtakeMode) {
                    intakeMode = true;
                    spinIntake = true;
                    intake.setPower(1);
                }
                if (!outtakeMode && !pathStarted) {
                    follower.followPath(paths.Path2, 1.0, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    stage = 6;
                }
                break;

            case 6: // Path4 (go to shooting position)
                if (!outtakeMode) {
                    intakeMode = true;
                    spinIntake = true;
                    intake.setPower(1);
                }
                if (!outtakeMode && !pathStarted) {
                    follower.followPath(paths.Path4, 1.0, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    settleStartMs = 0;
                    stage = 7;
                }
                break;

            case 7: // SHOOT #2  <-- REQUIRE SETTLE
                if (!shootRequested && !outtakeMode) {
                    if (pose != null && settledAt(pose, SHOOT_X, SHOOT_Y, SHOOT_H)) {
                        startOuttake();
                        shootRequested = true;
                    }
                }
                if (shootRequested && !outtakeMode) {
                    shootRequested = false;
                    stage = 8;
                }
                break;

            case 8: // Path5
                if (!outtakeMode) {
                    intakeMode = true;
                    spinIntake = true;
                    intake.setPower(1);
                }
                if (!outtakeMode && !pathStarted) {
                    follower.followPath(paths.Path5, 1.0, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    startStabilizeWait();
                    stage = 80;
                }
                break;

            case 80: // WAIT 1s (before Path6)
                if (stabilizeDone()) {
                    stage = 9;
                }
                break;

            case 9: // Path6
                if (!outtakeMode) {
                    intakeMode = true;
                    spinIntake = true;
                    intake.setPower(1);
                }
                if (!outtakeMode && !pathStarted) {
                    follower.followPath(paths.Path6, 1.0, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    stage = 10;
                }
                break;

            case 10: // Path7
                if (!outtakeMode) {
                    intakeMode = true;
                    spinIntake = true;
                    intake.setPower(1);
                }
                if (!outtakeMode && !pathStarted) {
                    follower.followPath(paths.Path7, 1.0, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    settleStartMs = 0;
                    stage = 11;
                }
                break;

            case 11: // SHOOT #3  <-- REQUIRE SETTLE
                if (!shootRequested && !outtakeMode) {
                    if (pose != null && settledAt(pose, SHOOT_X, SHOOT_Y, SHOOT_H)) {
                        startOuttake();
                        shootRequested = true;
                    }
                }
                if (shootRequested && !outtakeMode) {
                    shootRequested = false;
                    stage = 12;
                }
                break;

            case 12: // Path8 (park)
                intakeMode = false;
                spinIntake = false;
                launchPrepActive = false;
                intake.setPower(0);

                if (!outtakeMode && !pathStarted) {
                    follower.followPath(paths.Path8, 1.0, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    stage = 13;
                }
                break;

            case 13:
                launcherEnabled = false;
                updateLauncher();
                if (tureta != null) tureta.setPower(0);
                requestOpModeStop();
                break;
        }

        telemetry.addData("Stage", stage);
        telemetry.addData("Busy", follower.isBusy());

        telemetry.addData("RPM", rpm);
        telemetry.addData("FlywheelTargetRPM", flywheelTargetRPM);

        telemetry.addData("TurretTarget(deg)", targetTurretDeg);
        telemetry.addData("TurretCurrent(deg)", currentTurretDeg);
        telemetry.addData("TurretErr(deg)", errorDeg);
        telemetry.addData("TurretPower", turretPower);

        telemetry.addData("TrajectoryAngle", trajectoryAngle);

        if (pose != null) {
            telemetry.addData("PoseX", pose.getX());
            telemetry.addData("PoseY", pose.getY());
            telemetry.addData("PoseH(deg)", Math.toDegrees(pose.getHeading()));
            telemetry.addData("SettledAtShoot", settledAt(pose, SHOOT_X, SHOOT_Y, SHOOT_H));
        }

        telemetry.update();
    }

    /* ===================== PATHS (YOUR MODIFIED ONES) ===================== */
    public static class Paths {
        public PathChain Path9;
        public PathChain Path3;
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path8;

        public Paths(Follower follower) {
            Path9 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(57.000, 9.000),
                                    new Pose(57.000, 12.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(57.000, 12.000),
                                    new Pose(57.000, 21.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(299))
                    .build();

            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(57.000, 21.000),
                                    new Pose(42.000, 36.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(299), Math.toRadians(180))
                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(42.000, 36.000),
                                    new Pose(12.000, 36.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(12.000, 36.000),
                                    new Pose(57.000, 21.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(299))
                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(57.000, 21.000),
                                    new Pose(14.000, 21.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(299), Math.toRadians(200))
                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(14.000, 21.000),
                                    new Pose(14.000, 13.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(200), Math.toRadians(200))
                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(14.000, 13.000),
                                    new Pose(57.000, 21.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(200), Math.toRadians(299))
                    .build();

            Path8 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(57.000, 21.000),
                                    new Pose(39.000, 14.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(299), Math.toRadians(180))
                    .build();
        }
    }
}
