package org.firstinspires.ftc.teamcode.TeleOp;

import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "&TeleOp Stefan + Tureta Cip")
public class TeleOp_Stefan_Cip_Tureta extends LinearOpMode {

    /* ===================== SENSOR SMOOTHING (live debug) ===================== */
    int[] last5Sensor1 = new int[5];
    int[] last5Sensor2 = new int[5];
    int[] last5Sensor3 = new int[5];

    int indexSensor1 = 0;
    int indexSensor2 = 0;
    int indexSensor3 = 0;

    /* ===================== OUTTAKE SLOTS (existing) ===================== */
    int[] slots = new int[3];
    int[] totem = {2, 1, 2};

    /* ===================== LIVE COLOR READINGS (debug only) ===================== */
    int Color1 = 0;
    int Color2 = 0;
    int Color3 = 0;

    int detectedBalls = 0;
    int slotIntakeIndex = 0;
    double prev_t = 0;

    /* ===================== HARDWARE ===================== */
    ColorSensor colorsensorSLot1;
    ColorSensor colorsensorSLot2;
    ColorSensor colorsensorSLot3;

    DcMotor intake;
    DcMotorEx tureta;
    Servo ejector;

    DcMotor front_left;
    DcMotor front_right;
    DcMotor back_left;
    DcMotor back_right;

    DcMotorEx flywheel;

    Servo spinnerCLose;
    Servo spinnerFar;

    // turret trajectory angle servo (from old TeleOp)
    Servo trajectoryAngleModifier;

    /* ===================== OUTTAKE STEPS ===================== */
    boolean step1Done = false;
    boolean turetaDisabled = false;
    boolean step2Done = false;
    boolean step3Done = false;
    boolean step4Done = false;
    boolean step5Done = false;
    boolean step6Done = false;
    boolean step7Done = false;
    boolean step8Done = false;
    boolean step9Done = false;
    boolean step10Done = false;
    boolean step11Done = false;
    boolean spinIntake = false;

    /* ===================== LL / IMU ===================== */
    Limelight3A limelight;
    IMU imu;

    /* ===================== MODES ===================== */
    boolean intakeMode = false;
    boolean outtakeMode = false;

    private ElapsedTime spinnerTimeout = new ElapsedTime();
    private ElapsedTime outtakeTimeout = new ElapsedTime();

    final double ejectorDown = 0.214;
    final double ejectorUp = 0.03;

    final double[] slotPositionsIntake = {0, 0.19, 0.38};

    PinpointLocalizer pinpoint;
    Pose pose;
    double CoordX, CoordY, header;

    int ballsLoaded = 0;

    /* ===================== SPINDEXER OFFSETS / POSITIONS ===================== */
    private static final double SPINNER_LAUNCH_POS = 0.085; // launch/prep position
    private boolean launchPrepActive = false;

    /* ===================== FLYWHEEL (REV VELOCITY CONTROL) ===================== */
    static final double FLYWHEEL_TICKS_PER_REV = 28.0;
    static final double TARGET_RPM = 2471.0;
    static final double TARGET_TPS = TARGET_RPM * FLYWHEEL_TICKS_PER_REV / 60.0;

    static final double kP_v = 12.0;
    static final double kI_v = 0.0;
    static final double kD_v = 0.0;
    static final double kF_v = 14.0;

    static final double KICK_POWER = 1.0;
    static final double KICK_TIME_S = 0.20;

    boolean flywheelOn = false;
    ElapsedTime flyKickTimer = new ElapsedTime();
    boolean kicking = false;

    /* =======================================================================================
       ===================== TURRET + SMART LAUNCHER (ported from older TeleOp) ================
       ======================================================================================= */

    // Turret gearing + soft limits
    private static final double MOTOR_TICKS_PER_REV = 384.5;

    // Use old TeleOp ratio (75/26). If you KNOW it's 76/24 on your robot, change it.
    private static final double MOTOR_TO_TURRET_RATIO = 75.0 / 26.0;

    private static final double DEG_PER_TICK_TURETA =
            360.0 / (MOTOR_TICKS_PER_REV * MOTOR_TO_TURRET_RATIO);

    private static final double LEFT_LIMIT  = -110;
    private static final double RIGHT_LIMIT = 110;

    private static final double kP_TURETA = 0.015;
    private static final double MAX_POWER_TURETA = 0.2;

    // Field target (same as old)
    private double targetX = 10;
    private double targetY = 137.5;

    // turret offsets (inches) from robot center
    private final double turretOffsetX = 0.0;
    private final double turretOffsetY = 52 / 1000.0; // keep your old value

    private final double startTurretAngle = -180.0;

    private boolean aimingEnabled = false;
    private boolean launcherEnabled = false;

    // launch zones (ported)
    final double[][] launchZoneBig = {{-18,144},{162,144},{72,55}};
    final double[][] launchZoneSmall = {{40,0},{102,0},{72,32}};

    // physics parameters (ported)
    final double absoluteTurretHeight = 0.25; // meters
    final double absoluteTargetHeight = 0.9;  // meters
    final double relativeHeight = absoluteTargetHeight - absoluteTurretHeight;
    final double prefferedMaxHeightThrow = relativeHeight + 0.3; // meters
    final double launcherEfficiency = 0.43; // tune
    final double flywheelRadius = 0.048;    // meters
    final double g = 9.81;

    // trajectory angle servo mapping (ported)
    final double trajectoryAngleModifierGearRatio = 127 / 15.0;
    final double trajectoryAnglerMaxTravel = 300.0;
    final double trajectoryAnglePosPerDegree = trajectoryAngleModifierGearRatio / trajectoryAnglerMaxTravel;

    final double minTrajectoryAngle = 50.2;
    final double maxTrajectoryAngle = 65.0;

    final int minFlywheelRPM = 1000;
    final int maxFlywheelRPM = 6000;

    private double turretX = 0.0;
    private double turretY = 0.0;

    private double trajectoryAngle = 60.0;
    private int flywheelTargetRPM = 0;

    // RPM gating for shooter sequence (matches your old stable gate concept)
    private static final double RPM_TOL = 150;
    private static final long RPM_STABLE_MS = 80;
    private long rpmInRangeSinceMs = 0;

    private double currentTurretDeg = 0.0;
    private double targetTurretDeg = 0.0;
    private double turretErrorDeg = 0.0;
    private double turretPowerCmd = 0.0;

    private double getFlywheelRPM() {
        return flywheel.getVelocity() / FLYWHEEL_TICKS_PER_REV * 60.0;
    }

    private boolean rpmInRangeStable(double targetRpm) {
        double rpm = getFlywheelRPM();
        boolean inRange = (rpm >= (targetRpm - RPM_TOL)) && (rpm <= (targetRpm + 150));
        long now = System.currentTimeMillis();

        if (!inRange) {
            rpmInRangeSinceMs = 0;
            return false;
        }
        if (rpmInRangeSinceMs == 0) rpmInRangeSinceMs = now;
        return (now - rpmInRangeSinceMs) >= RPM_STABLE_MS;
    }

    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    private double sign(double x1,double y1,double x2,double y2,double x3,double y3){
        return (x1-x3)*(y2-y3)-(x2-x3)*(y1-y3);
    }

    private void disableIfNotInLaunchZone() {
        // Uses current pose
        double robotX = pose.getX();
        double robotY = pose.getY();

        double d1,d2,d3;
        if(robotY > 48) {
            d1 = sign(robotX, robotY, launchZoneBig[0][0], launchZoneBig[0][1], launchZoneBig[1][0], launchZoneBig[1][1]);
            d2 = sign(robotX, robotY, launchZoneBig[1][0], launchZoneBig[1][1], launchZoneBig[2][0], launchZoneBig[2][1]);
            d3 = sign(robotX, robotY, launchZoneBig[2][0], launchZoneBig[2][1], launchZoneBig[0][0], launchZoneBig[0][1]);
        } else {
            d1 = sign(robotX, robotY, launchZoneSmall[0][0], launchZoneSmall[0][1], launchZoneSmall[1][0], launchZoneSmall[1][1]);
            d2 = sign(robotX, robotY, launchZoneSmall[1][0], launchZoneSmall[1][1], launchZoneSmall[2][0], launchZoneSmall[2][1]);
            d3 = sign(robotX, robotY, launchZoneSmall[2][0], launchZoneSmall[2][1], launchZoneSmall[0][0], launchZoneSmall[0][1]);
        }

        boolean has_neg = d1 < 0 || d2 < 0 || d3 < 0;
        boolean has_pos = d1 > 0 || d2 > 0 || d3 > 0;

        // inside triangle => all same sign => !(has_neg && has_pos)
        if (has_neg && has_pos) {
            aimingEnabled = false;
        } else {
            aimingEnabled = true;
        }
    }

    private void setTrajectoryAngle(double angleDeg) {
        double angle = Range.clip(angleDeg, minTrajectoryAngle, maxTrajectoryAngle);
        double position = (angle - minTrajectoryAngle) * trajectoryAnglePosPerDegree;
        trajectoryAngleModifier.setPosition(Range.clip(position, 0.0, 1.0));
    }

    private void computeParameters() {
        // Distance (inches -> meters)
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
            double sinTheta = Math.sin(thetaRad);
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
    }

    private void updateTurretAimSmart() {
        // pose must be updated before calling this
        double robotX = pose.getX();
        double robotY = pose.getY();
        double robotHeadingDeg = Math.toDegrees(pose.getHeading());

        // turret position in field (inches)
        turretX = robotX + turretOffsetX * Math.cos(pose.getHeading()) - turretOffsetY * Math.sin(pose.getHeading());
        turretY = robotY + turretOffsetX * Math.sin(pose.getHeading()) + turretOffsetY * Math.cos(pose.getHeading());

        double dx = targetX - turretX;
        double dy = targetY - turretY;

        double fieldAngleDeg = Math.toDegrees(Math.atan2(dy, dx));

        // turret current angle
        currentTurretDeg = tureta.getCurrentPosition() * DEG_PER_TICK_TURETA + startTurretAngle;
        currentTurretDeg = normalizeAngle(currentTurretDeg);

        // desired angle
        if (aimingEnabled && !turetaDisabled) {
            targetTurretDeg = normalizeAngle(fieldAngleDeg - robotHeadingDeg);
        } else {
            targetTurretDeg = startTurretAngle;
        }

        // soft limits
        if (targetTurretDeg < 0 && targetTurretDeg > LEFT_LIMIT) {
            targetTurretDeg = LEFT_LIMIT;
        } else if (targetTurretDeg >= 0 && targetTurretDeg < RIGHT_LIMIT) {
            targetTurretDeg = RIGHT_LIMIT;
        }

        turretErrorDeg = normalizeAngle(targetTurretDeg - currentTurretDeg);

        // power
        turretPowerCmd = Range.clip(turretErrorDeg * kP_TURETA, -MAX_POWER_TURETA, MAX_POWER_TURETA);

        if (turetaDisabled) {
            tureta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            tureta.setPower(0);
        } else {
            tureta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            tureta.setPower(turretPowerCmd);
        }
    }

    private boolean spinnerFull() {
        return logicalSlots[0] != 0 && logicalSlots[1] != 0 && logicalSlots[2] != 0;
    }

    private void enableLauncher() {
        if (launcherEnabled) return;
        launcherEnabled = true;

        // baseline if not aiming (match old behavior)
        flywheelTargetRPM = 2000;
    }

    private void disableLauncher() {
        if (!launcherEnabled) return;
        launcherEnabled = false;
        flywheelTargetRPM = 0;
    }

    private void updateSmartLauncherIfActive() {
        // If aiming enabled -> compute RPM + angle, command both
        if (aimingEnabled) {
            computeParameters();
            setTrajectoryAngle(trajectoryAngle);
            setFlywheelVelocityRPM(flywheelTargetRPM);
            return;
        }

        // If not aiming, but full -> spin at baseline rpm (like old enableLauncher)
        if (launcherEnabled) {
            setTrajectoryAngle(maxTrajectoryAngle);
            setFlywheelVelocityRPM(flywheelTargetRPM);
            return;
        }

        // else: do nothing (manual flywheelLogic handles it)
    }

    private void setFlywheelVelocityRPM(double rpmTarget) {
        double tps = rpmTarget * FLYWHEEL_TICKS_PER_REV / 60.0;
        flywheel.setVelocity(tps);
    }

    /* ======================================================================================= */

    /* ===================== LOGICAL INVENTORY ===================== */
    int[] logicalSlots = new int[3];

    // ULTRA-FAST intake smoothing (3 samples, need 2)
    int[] lastNIntake = new int[3];
    int idxIntake = 0;

    int lastStableIntakeColor = 0;

    boolean colorPending = false;
    long colorStartTimeMs = 0;

    boolean detectionLocked = false;
    boolean waitingForClear = false;
    boolean spinnerMoving = false;

    static final long DETECT_DELAY_MS = 0;
    static final long SERVO_MOVE_LOCK_MS = 45;
    long servoMoveStartMs = 0;

    /* =========================================================================================
       ===================== SPINNER SERVO CENTERING (deadzone/backlash fix) ====================
       ========================================================================================= */

    private static final double SPINNER_FAR_OFFSET   = -0.010;
    private static final double SPINNER_CLOSE_OFFSET = -0.010;

    private static final double SPINNER_OVERSHOOT = 0.010;
    private static final double SPINNER_OVERSHOOT_TIME_S = 0.06; // 60ms
    private static final double SPINNER_TARGET_EPS = 0.0015;

    // Base target (programmed). Trim gets added in updateSpinnerServos().
    private double spinnerBaseTarget = 0.0;

    // Effective target after trim (computed)
    private double spinnerTarget = 0.0;

    private double spinnerCmd = 0.0;
    private double spinnerLastTarget = 0.0;

    private boolean spinnerInOvershoot = false;
    private double spinnerOvershootCmd = 0.0;
    private final ElapsedTime spinnerMoveTimer = new ElapsedTime();

    /* ===================== LIVE TRIM (belt slip compensation) ===================== */
    // lower limit is 0.00 (no negative trim)
    private static final double SPINNER_TRIM_MIN = 0.00;
    private static final double SPINNER_TRIM_MAX = 0.20;

    // half speed (0.06 /s)
    private static final double SPINNER_TRIM_RATE_PER_S = 0.06;

    private double spinnerTrim = 0.0;
    private final ElapsedTime trimTimer = new ElapsedTime();

    private void setSpinnerTarget(double target) {
        spinnerBaseTarget = Range.clip(target, 0.0, 1.0);
    }

    private void updateSpinnerTrim() {
        double dt = trimTimer.seconds();
        trimTimer.reset();

        boolean inc = gamepad2.right_bumper;
        boolean dec = gamepad2.left_bumper;

        if (inc && !dec) {
            spinnerTrim += SPINNER_TRIM_RATE_PER_S * dt;
        } else if (dec && !inc) {
            spinnerTrim -= SPINNER_TRIM_RATE_PER_S * dt;
        }

        spinnerTrim = Range.clip(spinnerTrim, SPINNER_TRIM_MIN, SPINNER_TRIM_MAX);
    }

    private void updateSpinnerServos() {
        // Effective target (base + trim)
        spinnerTarget = Range.clip(spinnerBaseTarget + spinnerTrim, 0.0, 1.0);

        // retrigger overshoot only on real target changes
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

    /* ========================================================================================= */

    private void initLocalization() {
        pinpoint = new PinpointLocalizer(hardwareMap, Constants.localizerConstants);
        startPose = new Pose(22, 127, Math.toRadians(-36));
        pinpoint.setStartPose(startPose);
    }

    private Pose startPose;
    private void resetLocalization() {
        pinpoint.setPose(startPose);
    }

    private void InitWheels() {
        front_left = hardwareMap.dcMotor.get("lf");
        front_right = hardwareMap.dcMotor.get("rf");
        back_left = hardwareMap.dcMotor.get("lr");
        back_right = hardwareMap.dcMotor.get("rr");

        front_right.setDirection(DcMotorSimple.Direction.FORWARD);
        back_right.setDirection(DcMotorSimple.Direction.FORWARD);
        front_left.setDirection(DcMotorSimple.Direction.REVERSE);
        back_left.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    private void InitDc() {
        intake = hardwareMap.get(DcMotor.class, "intake");
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");

        tureta = hardwareMap.get(DcMotorEx.class, "tureta");
        tureta.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        tureta.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        tureta.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setVelocityPIDFCoefficients(kP_v, kI_v, kD_v, kF_v);
    }

    private void InitServo() {
        ejector = hardwareMap.get(Servo.class, "ejector");
        spinnerFar = hardwareMap.get(Servo.class, "SpinnerFar");
        spinnerCLose = hardwareMap.get(Servo.class, "SpinnerClose");

        // turret trajectory servo (old name)
        trajectoryAngleModifier = hardwareMap.get(Servo.class, "unghituretaoy");

        ejector.setDirection(Servo.Direction.REVERSE);
        ejector.setPosition(ejectorDown);

        setSpinnerTarget(0.0);
        spinnerCmd = 0.0;
        spinnerLastTarget = 0.0;
        spinnerInOvershoot = false;

        spinnerTrim = 0.0;
        trimTimer.reset();

        setTrajectoryAngle(maxTrajectoryAngle);

        updateSpinnerServos();
    }

    private void InitLL() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(5);
        limelight.start();

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
        );
        imu.initialize(new IMU.Parameters(orientation));
    }

    private void InitAux() {
        colorsensorSLot1 = hardwareMap.colorSensor.get("Color1");
        colorsensorSLot2 = hardwareMap.colorSensor.get("Color2");
        colorsensorSLot3 = hardwareMap.colorSensor.get("Color3");
    }

    // ===================== CHASSIS ON GAMEPAD 2 =====================
    private void SetWheelsPower() {
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

    private void updateCulori() {
        Color1 = CuloareFinala1(colorsensorSLot1, last5Sensor1, indexSensor1);
        indexSensor1 = (indexSensor1 + 1) % 5;

        Color2 = CuloareFinala1(colorsensorSLot2, last5Sensor2, indexSensor2);
        indexSensor2 = (indexSensor2 + 1) % 5;

        Color3 = CuloareFinala1(colorsensorSLot3, last5Sensor3, indexSensor3);
        indexSensor3 = (indexSensor3 + 1) % 5;
    }

    private void rotateLogicalSlotsRight() {
        int temp = logicalSlots[2];
        logicalSlots[2] = logicalSlots[1];
        logicalSlots[1] = logicalSlots[0];
        logicalSlots[0] = temp;
    }

    private void rotateLogicalSlotsLeft() {
        int temp = logicalSlots[0];
        logicalSlots[0] = logicalSlots[1];
        logicalSlots[1] = logicalSlots[2];
        logicalSlots[2] = temp;
    }

    private boolean isSpindexerFull() {
        return logicalSlots[0] != 0 && logicalSlots[1] != 0 && logicalSlots[2] != 0;
    }

    private void resetIntakeGatingAndFilters() {
        waitingForClear = false;
        detectionLocked = false;
        spinnerMoving = false;
        colorPending = false;
        lastStableIntakeColor = 0;

        for (int i = 0; i < lastNIntake.length; i++) lastNIntake[i] = 0;
        idxIntake = 0;
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

        int finalColor = 0;
        if (count1 >= 2 && count1 > count2) finalColor = 1;
        else if (count2 >= 2 && count2 > count1) finalColor = 2;

        return finalColor;
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

        int intakeColor = processIntakeSensor(colorsensorSLot1);

        boolean newColorDetected = (intakeColor != 0 && lastStableIntakeColor == 0);
        if (newColorDetected) lastStableIntakeColor = intakeColor;

        if (newColorDetected && !colorPending && !detectionLocked) {
            colorStartTimeMs = System.currentTimeMillis();
            colorPending = true;
        }

        if (colorPending && (System.currentTimeMillis() - colorStartTimeMs >= DETECT_DELAY_MS)) {

            logicalSlots[0] = intakeColor;
            rotateLogicalSlotsRight();

            slotIntakeIndex++;
            slotIntakeIndex = slotIntakeIndex % 3;

            setSpinnerTarget(slotPositionsIntake[slotIntakeIndex]);

            waitingForClear = true;
            detectionLocked = true;
            spinnerMoving = true;
            servoMoveStartMs = System.currentTimeMillis();

            colorPending = false;
        }
    }

    private void autoLaunchPrepLogic() {
        if (outtakeMode) return;

        if (intakeMode && isSpindexerFull()) {
            launchPrepActive = true;
        }

        if (launchPrepActive) {
            setSpinnerTarget(SPINNER_LAUNCH_POS);
        }
    }

    private void flywheelLogicManual() {
        // Manual toggle with share (kept exactly as your current code)
        if (gamepad1.shareWasPressed()) {
            flywheelOn = !flywheelOn;

            if (flywheelOn) {
                kicking = true;
                flyKickTimer.reset();
            } else {
                kicking = false;
                flywheel.setPower(0);
            }
        }

        if (!flywheelOn) {
            flywheel.setPower(0);
            return;
        }

        if (kicking) {
            flywheel.setPower(KICK_POWER);
            if (flyKickTimer.seconds() >= KICK_TIME_S) {
                kicking = false;
            }
        } else {
            flywheel.setVelocity(TARGET_TPS);
        }
    }

    private void servoLogic() {
        if (gamepad1.optionsWasReleased()) {
            ejector.setPosition(ejectorDown);
        } else if (gamepad1.optionsWasPressed()) {
            ejector.setPosition(ejectorUp);
        }

        if (gamepad1.touchpadWasPressed()) {
            setSpinnerTarget(0);
            slotIntakeIndex = 0;
        }

        if (gamepad1.dpadRightWasPressed()) {
            slotIntakeIndex++;
            slotIntakeIndex = slotIntakeIndex % 3;
            setSpinnerTarget(slotPositionsIntake[slotIntakeIndex]);
            rotateLogicalSlotsRight();
        }

        if (gamepad1.dpadLeftWasPressed()) {
            slotIntakeIndex--;
            if (slotIntakeIndex < 0) slotIntakeIndex = 2;
            setSpinnerTarget(slotPositionsIntake[slotIntakeIndex]);
            rotateLogicalSlotsLeft();
        }
    }

    private void updateTelemetry() {
        telemetry.addData("Live Color1", Color1);
        telemetry.addData("Live Color2", Color2);
        telemetry.addData("Live Color3", Color3);

        telemetry.addData("LogicalSlot 1", logicalSlots[0]);
        telemetry.addData("LogicalSlot 2", logicalSlots[1]);
        telemetry.addData("LogicalSlot 3", logicalSlots[2]);

        telemetry.addData("FULL", isSpindexerFull());
        telemetry.addData("launchPrepActive", launchPrepActive);

        telemetry.addData("waitingForClear", waitingForClear);
        telemetry.addData("colorPending", colorPending);
        telemetry.addData("spinnerMoving", spinnerMoving);

        telemetry.addData("spinnerBaseTarget", spinnerBaseTarget);
        telemetry.addData("spinnerTrim", spinnerTrim);
        telemetry.addData("spinnerTarget(eff)", spinnerTarget);

        telemetry.addData("spinnerCmd", spinnerCmd);
        telemetry.addData("spinnerOvershoot", spinnerInOvershoot);

        telemetry.addData("SpinnerFar pos", spinnerFar.getPosition());
        telemetry.addData("SpinnerClose pos", spinnerCLose.getPosition());
        telemetry.addData("slotIndex", slotIntakeIndex);

        telemetry.addData("Aiming", aimingEnabled);
        telemetry.addData("LauncherEnabled", launcherEnabled);
        telemetry.addData("Turret deg", "%.1f", currentTurretDeg);
        telemetry.addData("Turret tgt", "%.1f", targetTurretDeg);
        telemetry.addData("Turret err", "%.1f", turretErrorDeg);
        telemetry.addData("Turret pwr", "%.2f", turretPowerCmd);

        telemetry.addData("Flywheel RPM", "%.0f", getFlywheelRPM());
        telemetry.addData("Flywheel Target RPM", flywheelTargetRPM);

        telemetry.addData("trajectoryAngle", "%.1f", trajectoryAngle);

        telemetry.update();
    }

    private void runOuttake() {
        intake.setPower(-1);

        final int EJECTOR_UP_DELAY = 300;
        final int EJECTOR_DOWN_DELAY = 170;
        final int SPINNER_SLOT_CHANGE_DELAY = 300;
        final int INITIAL_DELAY = 400;

        slots[0] = logicalSlots[0];
        slots[1] = logicalSlots[1];
        slots[2] = logicalSlots[2];

        logicalSlots[0] = 0;
        logicalSlots[1] = 0;
        logicalSlots[2] = 0;

        Color1 = 0;
        Color2 = 0;
        Color3 = 0;

        double t = outtakeTimeout.milliseconds();

        if (t - prev_t >= 10 && !step1Done) {
            setSpinnerTarget(0.085);
            step1Done = true;
            prev_t = 10;
        }

        if (t >= prev_t + INITIAL_DELAY && !step2Done && step1Done) {
            ejector.setPosition(ejectorUp);
            step2Done = true;
            prev_t += INITIAL_DELAY;
        }

        if (t >= prev_t + EJECTOR_UP_DELAY && !step3Done && step2Done) {
            ejector.setPosition(ejectorDown);
            step3Done = true;
            prev_t += EJECTOR_UP_DELAY;
        }

        if (t >= prev_t + EJECTOR_DOWN_DELAY && !step4Done && step3Done) {
            setSpinnerTarget(0.28);
            step4Done = true;
            prev_t += EJECTOR_DOWN_DELAY;
        }

        if (t >= prev_t + SPINNER_SLOT_CHANGE_DELAY && !step5Done && step4Done) {
            ejector.setPosition(ejectorUp);
            step5Done = true;
            prev_t += SPINNER_SLOT_CHANGE_DELAY;
        }

        if (t >= prev_t + EJECTOR_UP_DELAY && !step6Done && step5Done) {
            ejector.setPosition(ejectorDown);
            step6Done = true;
            prev_t += EJECTOR_UP_DELAY;
        }

        if (t >= prev_t + EJECTOR_DOWN_DELAY && !step7Done && step6Done) {
            setSpinnerTarget(0.467);
            step7Done = true;
            prev_t += EJECTOR_DOWN_DELAY;
        }

        if (t >= prev_t + SPINNER_SLOT_CHANGE_DELAY && !step8Done && step7Done) {
            ejector.setPosition(ejectorUp);
            step8Done = true;
            prev_t += SPINNER_SLOT_CHANGE_DELAY;
        }

        if (t >= prev_t + EJECTOR_UP_DELAY && !step9Done && step8Done) {
            ejector.setPosition(ejectorDown);
            step9Done = true;
            prev_t += EJECTOR_UP_DELAY;
        }

        if (t >= prev_t + EJECTOR_DOWN_DELAY && !step10Done && step9Done) {
            setSpinnerTarget(0.0);
            step10Done = true;

            outtakeMode = false;

            intakeMode = true;
            spinIntake = true;
            intake.setPower(-1);

            slotIntakeIndex = 0;
            setSpinnerTarget(0.0);

            launchPrepActive = false;
            resetIntakeGatingAndFilters();

            ballsLoaded = 0;
            prev_t = 0;
        }
    }

    @Override
    public void runOpMode() {
        InitWheels();
        InitAux();
        InitDc();
        InitLL();
        InitServo();
        initLocalization();

        waitForStart();

        trimTimer.reset();

        while (opModeIsActive()) {

            // Smooth trim update (gamepad2 bumpers)
            updateSpinnerTrim();

            // gamepad2 Y -> reset trim to 0 AND force spinner to 0.00 (for tuning)
            if (gamepad2.y) {
                spinnerTrim = 0.0;
                launchPrepActive = false;
                setSpinnerTarget(0.0);
            }

            // gamepad1 Y -> force spinner to 0.00 (base target) and cancel launch hold
            if (gamepad1.yWasPressed()) {
                launchPrepActive = false;
                setSpinnerTarget(0.0);
            }

            // Launch hold (full) -> sets base target (trim still applies)
            autoLaunchPrepLogic();

            // Drive servos via controller every loop
            updateSpinnerServos();

            servoLogic();
            SetWheelsPower();

            // localization
            pinpoint.update();
            pose = pinpoint.getPose();

            // turret/launcher zone gating + aim
            disableIfNotInLaunchZone();
            updateTurretAimSmart();

            // launcher enable logic: same behavior as old code
            if (spinnerFull() && !aimingEnabled) {
                enableLauncher();
            } else {
                if (!spinnerFull() && launcherEnabled && !aimingEnabled) disableLauncher();
            }

            // SMART launcher overrides manual flywheel when active
            if (aimingEnabled || launcherEnabled) {
                updateSmartLauncherIfActive();
                // we also make sure manual flywheel doesn't fight it
                flywheelOn = false;
                kicking = false;
            } else {
                flywheelLogicManual();
            }

            // intake control (unchanged)
            if (gamepad1.crossWasPressed() && !gamepad1.crossWasReleased()) {
                intake.setPower(1);
            } else if (gamepad1.crossWasReleased()) {
                intake.setPower(spinIntake ? -1 : 0);
            }

            if (gamepad1.circleWasPressed()) {
                outtakeMode = false;
                intakeMode = true;

                spinIntake = !spinIntake;
                intake.setPower(spinIntake ? -1 : 0);

                ballsLoaded = 0;
                setSpinnerTarget(0);
                slotIntakeIndex = 0;

                logicalSlots[0] = 0;
                logicalSlots[1] = 0;
                logicalSlots[2] = 0;

                launchPrepActive = false;
                resetIntakeGatingAndFilters();
            }

            if (intakeMode && !outtakeMode && !launchPrepActive) {
                updateCulori();
                colorDrivenSpinnerLogicServos();
            } else if (intakeMode && launchPrepActive) {
                updateCulori();
            }

            if (gamepad1.psWasPressed()) {
                resetLocalization();
            }

            if (gamepad1.right_trigger > 0.8) {
                intakeMode = false;
                outtakeMode = true;
                intake.setPower(0);
                outtakeTimeout.reset();

                launchPrepActive = false;

                step1Done = false;
                step2Done = false;
                step3Done = false;
                step4Done = false;
                step5Done = false;
                step6Done = false;
                step7Done = false;
                step8Done = false;
                step9Done = false;
                step10Done = false;

                prev_t = 0;
            }

            // keep your disable turret control
            if (gamepad1.yWasPressed() && !gamepad1.yWasReleased()) {
                turetaDisabled = true;
            } else if (gamepad1.yWasReleased()) {
                turetaDisabled = false;
                tureta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }

            if (outtakeMode) {
                runOuttake();
            }

            updateTelemetry();
            idle();
        }
    }
}