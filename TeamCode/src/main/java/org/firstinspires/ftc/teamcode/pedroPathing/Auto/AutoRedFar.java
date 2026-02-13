package org.firstinspires.ftc.teamcode.pedroPathing.Auto;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.Arrays;

@Autonomous(name = "auto9RedFar", group = "Test")
public class AutoRedFar extends OpMode {

    /* ===================== TELEMETRY ===================== */
    private TelemetryManager panelsTelemetry;

    /* ===================== PEDRO ===================== */
    public Follower follower;
    private boolean pathStarted = false;
    private Paths paths;

    /* ===================== AUTONOMOUS STAGE ===================== */
    private int stage = 0;

    /* ===================== DELAY GATE ===================== */
    private final ElapsedTime autoDelay = new ElapsedTime();
    private boolean waiting = false;
    private static final double COMMAND_DELAY = 0.0;

    private boolean delayDone() {
        if (!waiting) {
            waiting = true;
            autoDelay.reset();
            return false;
        }
        if (autoDelay.seconds() >= COMMAND_DELAY) {
            waiting = false;
            return true;
        }
        return false;
    }

    /* ===================== HARDWARE ===================== */
    private DcMotor intake;
    private DcMotorEx tureta;

    private DcMotorEx flywheel;
    private Servo spinnerCLose, spinnerFar, ejector;
    private ColorSensor colorsensorSLot1, colorsensorSLot2, colorsensorSLot3;
    private Servo trajectoryAngleModifier;
    private VoltageSensor batteryVoltage;

    /* ===================== FLYWHEEL (PIDF VELOCITY) ===================== */
    private static final double FLYWHEEL_TICKS_PER_REV = 28.0;

    // baseline (fallback)
    public static double TARGET_RPM = 2940;

    // Velocity PIDF (aggressive spin-up policy)
    public static double kP_v = 30.0;
    public static double kI_v = 0.0;
    public static double kD_v = 0.0;
    public static double kF_v = 14.0;

    private double rpm = 0.0;

    // ======= TELEOP-STYLE LAUNCH MODEL (ported from Blue) =======
    private double trajectoryAngle = 70.0;
    private int flywheelTargetRPM = (int) TARGET_RPM;

    // physics/geometry (same as your Blue)
    private static final double absoluteTurretHeight = 0.25; // m
    private static final double absoluteTargetHeight = 1.0;  // m
    private static final double relativeHeight = absoluteTargetHeight - absoluteTurretHeight;
    private static final double prefferedMaxHeightThrow = relativeHeight + 0.3;

    private static final double launcherEfficiency = 0.43;
    private static final double flywheelRadius = 0.048; // m
    private static final double g = 9.81;

    // angle servo mapping
    private static final double trajectoryAngleModifierGearRatio = 127 / 15.0;
    private static final double trajectoryAnglerMaxTravel = 300.0;
    private static final double trajectoryAnglePosPerDegree =
            trajectoryAngleModifierGearRatio / trajectoryAnglerMaxTravel;

    private static final double minTrajectoryAngle = 50.2;
    private static final double maxTrajectoryAngle = 70.0;

    private static final int minFlywheelRPM = 1000;
    private static final int maxFlywheelRPM = 6000;

    /* ===================== AUTO TIMING ===================== */
    private static final double INTAKE_SWEEP_SPEED = 0.3;
    private static final double INTAKE_PASS2_SPEED = 0.35;
    private static final double AUTO_TOTAL_S = 30.0;
    private static final double PARK_IF_REMAIN_S = 2.0;

    /* ===================== SENSOR SMOOTHING ===================== */
    private final int[] last5Sensor1 = new int[5];
    private final int[] last5Sensor2 = new int[5];
    private final int[] last5Sensor3 = new int[5];

    private int indexSensor1 = 0;
    private int indexSensor2 = 0;
    private int indexSensor3 = 0;

    private int Color1 = 0;
    private int Color2 = 0;
    private int Color3 = 0;

    /* ===================== OUTTAKE SLOTS ===================== */
    private final int[] slots = new int[3];
    private int slotIntakeIndex = 0;

    /* ===================== MODES ===================== */
    private boolean intakeMode = false;
    private boolean outtakeMode = false;
    private boolean spinIntake = false;

    private static final double ejectorDown = 0.255;
    private static final double ejectorUp = 0.03;

    private static final double[] slotPositionsIntake = {0, 0.19, 0.38};

    /* ===================== SPINDEXER OFFSETS / POSITIONS ===================== */
    private static final double SPINNER_LAUNCH_POS = 0.085;
    private boolean launchPrepActive = false;

    /* ===================== LOGICAL INVENTORY ===================== */
    private final int[] logicalSlots = new int[3];

    // intake detect internal
    private final int[] lastNIntake = new int[3];
    private int idxIntake = 0;
    private int lastStableIntakeColor = 0;

    private boolean colorPending = false;
    private long colorStartTimeMs = 0;

    /* ===================== TURRET ===================== */
    private static final double MOTOR_TICKS_PER_REV = 384.5;
    private static final double MOTOR_TO_TURRET_RATIO = 75.0 / 26.0;

    private static final double DEG_PER_TICK_TURETA =
            360.0 / (MOTOR_TICKS_PER_REV * MOTOR_TO_TURRET_RATIO);

    // soft limits (kept identical)
    private static final double LEFT_LIMIT = -110;
    private static final double RIGHT_LIMIT = 110;

    // control
    private static final double kP = 0.015;
    private static final double MAX_POWER_TURETA = 0.4;

    // RED target (field coords)
    private double targetX = 134.0;
    private double targetY = 144.0;

    // TeleOp aiming geometry (ported)
    private static final double startTurretAngle = -180.0;
    private static final double turretOffsetX = 0.0;
    private static final double turretOffsetY = 52 / 1000.0; // 0.052m

    private static final double TURRET_TOL_DEG = 2.0;
    private boolean turretOnTarget = false;

    // internal aim
    private double turretX = 0.0;
    private double turretY = 0.0;

    private boolean aimingEnabled = true;

    private boolean detectionLocked = false;
    private boolean waitingForClear = false;
    private boolean spinnerMoving = false;

    private static final long DETECT_DELAY_MS = 0;
    private static final long SERVO_MOVE_LOCK_MS = 45;
    private long servoMoveStartMs = 0;

    /* ===================== SHOOTING FSM ===================== */
    private int outtakeStep = 0;
    private long stepStartMs = 0;

    private static final double RPM_TOL = 50.0;
    private static final long RPM_STABLE_MS = 150;
    private long rpmInRangeSinceMs = 0;

    private static final long OUTTAKE_INITIAL_DELAY_MS = 200;
    private static final long OUTTAKE_EJECTOR_UP_MS = 250;
    private static final long OUTTAKE_EJECTOR_DOWN_MS = 170;
    private static final long OUTTAKE_SPINNER_MOVE_MS = 200;

    private boolean shootStageStarted = false;

    /* ===================== LOCALIZATION ===================== */
    private Pose pose;
    private PinpointLocalizer pinpoint;

    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    /* ===================== TRAJECTORY MODEL ===================== */
    private void computeParameters() {
        // inches -> meters
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
            flywheelTargetRPM = (int) (60 * exitVelocity / (2 * Math.PI * flywheelRadius * launcherEfficiency));
        } else {
            trajectoryAngle = idealAngle;

            double thetaRad = Math.toRadians(trajectoryAngle);
            double sinTheta = Math.sin(thetaRad);
            if (Math.abs(sinTheta) < 1e-6) return;

            double v0 = Math.sqrt(2 * g * prefferedMaxHeightThrow) / sinTheta;
            flywheelTargetRPM = (int) (60 * v0 / (2 * Math.PI * flywheelRadius * launcherEfficiency));
        }

        flywheelTargetRPM = Math.max(minFlywheelRPM, Math.min(flywheelTargetRPM, maxFlywheelRPM));
    }

    private void setTrajectoryAngle(double angle) {
        if (trajectoryAngleModifier == null) return;
        angle = Range.clip(angle, minTrajectoryAngle, maxTrajectoryAngle);
        double position = (angle - minTrajectoryAngle) * trajectoryAnglePosPerDegree;
        trajectoryAngleModifier.setPosition(position);
    }

    private void updateTrajectoryAngle() {
        setTrajectoryAngle(trajectoryAngle);
    }

    /* ===================== TURRET AIM (TeleOp-style, with offset) ===================== */
    private void updateTurretAim() {
        if (pose == null) return;

        double robotX = pose.getX();
        double robotY = pose.getY();
        double robotHeading = pose.getHeading();
        double robotHeadingDeg = Math.toDegrees(robotHeading);

        // offset turret origin in field frame
        turretX = robotX + turretOffsetX * Math.cos(robotHeading) - turretOffsetY * Math.sin(robotHeading);
        turretY = robotY + turretOffsetX * Math.sin(robotHeading) + turretOffsetY * Math.cos(robotHeading);

        double dx = targetX - turretX;
        double dy = targetY - turretY;
        double fieldAngle = Math.toDegrees(Math.atan2(dy, dx));

        double currentTurretDeg = tureta.getCurrentPosition() * DEG_PER_TICK_TURETA + startTurretAngle;

        double targetTurretDeg;
        if (aimingEnabled) {
            targetTurretDeg = normalizeAngle(fieldAngle - robotHeadingDeg);
        } else {
            targetTurretDeg = startTurretAngle;
        }

        currentTurretDeg = normalizeAngle(currentTurretDeg);

        // keep SAME (even if it looks inverted) limit logic as your TeleOp/Blue
        if (targetTurretDeg < 0 && targetTurretDeg > LEFT_LIMIT) {
            targetTurretDeg = LEFT_LIMIT;
        } else if (targetTurretDeg >= 0 && targetTurretDeg < RIGHT_LIMIT) {
            targetTurretDeg = RIGHT_LIMIT;
        }

        double error = normalizeAngle(targetTurretDeg - currentTurretDeg);
        turretOnTarget = Math.abs(error) <= TURRET_TOL_DEG;

        double power = Range.clip(error * kP, -MAX_POWER_TURETA, MAX_POWER_TURETA);
        tureta.setPower(power);
    }

    /* ===================== SAFETY ABORT ===================== */
    private void abortOuttakeNow() {
        outtakeMode = false;
        shootStageStarted = false;
        waiting = false;

        ejector.setPosition(ejectorDown);
        setSpinnerTarget(0.0);

        intakeMode = false;
        spinIntake = false;
        if (intake != null) intake.setPower(0);

        outtakeStep = 0;
        stepStartMs = 0;
        rpmInRangeSinceMs = 0;

        launchPrepActive = false;
        resetIntakeGatingAndFilters();
    }

    /* ===================== RPM GATE ===================== */
    private boolean rpmInRangeStable() {
        double target = flywheelTargetRPM; // dynamic like Blue
        boolean inRange = (rpm >= (target - RPM_TOL)) && (rpm <= (target + RPM_TOL));
        long now = System.currentTimeMillis();

        if (!inRange) {
            rpmInRangeSinceMs = 0;
            return false;
        }
        if (rpmInRangeSinceMs == 0) rpmInRangeSinceMs = now;
        return (now - rpmInRangeSinceMs) >= RPM_STABLE_MS;
    }

    private void startStep(int newStep) {
        outtakeStep = newStep;
        stepStartMs = System.currentTimeMillis();
    }

    /* ===================== INIT ===================== */
    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(87, 9, Math.toRadians(90)));
        paths = new Paths(follower);

        pinpoint = new PinpointLocalizer(hardwareMap, Constants.localizerConstants);
        pinpoint.setStartPose(new Pose(87, 9, Math.toRadians(90)));

        intake = hardwareMap.get(DcMotor.class, "intake");
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        tureta = hardwareMap.get(DcMotorEx.class, "tureta");

        spinnerCLose = hardwareMap.get(Servo.class, "SpinnerClose");
        spinnerFar = hardwareMap.get(Servo.class, "SpinnerFar");
        ejector = hardwareMap.get(Servo.class, "ejector");
        trajectoryAngleModifier = hardwareMap.get(Servo.class, "unghituretaoy");

        colorsensorSLot1 = hardwareMap.colorSensor.get("Color1");
        colorsensorSLot2 = hardwareMap.colorSensor.get("Color2");
        colorsensorSLot3 = hardwareMap.colorSensor.get("Color3");

        batteryVoltage = hardwareMap.voltageSensor.iterator().next();

        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        tureta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tureta.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // you had REVERSE in your code
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        ejector.setDirection(Servo.Direction.REVERSE);
        ejector.setPosition(ejectorDown);

        spinnerFar.setPosition(0);
        spinnerCLose.setPosition(0);

        trajectoryAngleModifier.setPosition(0);

        autoDelay.reset();

        // Start state
        stage = 0;
        pathStarted = false;

        intakeMode = true;
        outtakeMode = false;
        spinIntake = true;

        setSpinnerTarget(0);
        slotIntakeIndex = 0;

        launchPrepActive = false;
        resetIntakeGatingAndFilters();

        logicalSlots[0] = logicalSlots[1] = logicalSlots[2] = 0;

        // shooter fsm reset
        outtakeStep = 0;
        stepStartMs = 0;
        rpmInRangeSinceMs = 0;

        shootStageStarted = false;
        waiting = false;
    }

    /* ===================== SPINNER SERVO CONTROLLER ===================== */
    private static final double SPINNER_FAR_OFFSET = -0.010;
    private static final double SPINNER_CLOSE_OFFSET = -0.010;

    private static final double SPINNER_OVERSHOOT = 0.010;
    private static final double SPINNER_OVERSHOOT_TIME_S = 0.06;
    private static final double SPINNER_TARGET_EPS = 0.0015;

    private double spinnerTarget = 0.0;
    private double spinnerCmd = 0.0;
    private double spinnerLastTarget = 0.0;

    private boolean spinnerInOvershoot = false;
    private double spinnerOvershootCmd = 0.0;
    private final ElapsedTime spinnerMoveTimer = new ElapsedTime();

    private void setSpinnerTarget(double target) {
        spinnerTarget = Range.clip(target, 0.0, 1.0);
    }

    private void updateSpinnerServos() {
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

    /* ===================== LOOP ===================== */
    @Override
    public void loop() {
        // update localization
        pinpoint.update();
        follower.update();
        pose = follower.getPose();

        // aim first so turretX/turretY are correct
        updateTurretAim();

        // compute dynamic RPM/angle only when aiming is enabled
        if (aimingEnabled) {
            computeParameters();
            updateTrajectoryAngle();
        } else {
            flywheelTargetRPM = (int) TARGET_RPM;
        }

        // now run flywheel control using flywheelTargetRPM
        updateFlywheel();

        // spindexer + sensors
        updateCulori();
        updateSpinnerServos();

        // time-based safety park
        double remain = AUTO_TOTAL_S - autoDelay.seconds();
        if (remain <= PARK_IF_REMAIN_S && stage < 11) {
            abortOuttakeNow();
            stage = 11;
            pathStarted = false;
        }

        // intake detection only while intaking and not shooting and not launch holding
        if (intakeMode && !outtakeMode && !launchPrepActive) {
            colorDrivenSpinnerLogicServos();
        }

        // auto hold launch position when full
        autoLaunchPrepLogic();

        // ===================== AUTONOMOUS FSM =====================
        switch (stage) {

            case 0: // INITIAL POSITION
                if (outtakeMode) break;
                if (!pathStarted) {
                    follower.followPath(paths.Path0, 1.0, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    waiting = false;
                    shootStageStarted = false;
                    stage = 1;
                }
                break;

            case 1: // GO TO SHOOT LINE
                aimingEnabled = true;
                if (!pathStarted) {
                    follower.followPath(paths.Path9, 1.0, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    waiting = false;
                    shootStageStarted = false;
                    stage = 2;
                }
                break;

            case 2: // SHOOT FIRST 3
                aimingEnabled = true;
                if (!shootStageStarted) {
                    if (delayDone()) {
                        startOuttake();
                        shootStageStarted = true;
                    }
                } else {
                    if (!outtakeMode) {
                        shootStageStarted = false;
                        waiting = false;
                        stage = 3;
                    }
                }
                break;

            case 3: // GO TO STACK
                aimingEnabled = false;
                if (outtakeMode) break;

                intakeMode = true;
                spinIntake = true;
                intake.setPower(1);

                if (!pathStarted) {
                    follower.followPath(paths.Path1, 1.0, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    stage = 4;
                }
                break;

            case 4: // SWEEP STACK
                if (outtakeMode) break;
                aimingEnabled = false;

                intakeMode = true;
                spinIntake = true;
                intake.setPower(1);

                if (!pathStarted) {
                    follower.followPath(paths.Path2, INTAKE_SWEEP_SPEED, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    setSpinnerTarget(SPINNER_LAUNCH_POS);
                    pathStarted = false;
                    stage = 5;
                }
                break;

            case 5: // RETURN TO SHOOT
                if (outtakeMode) break;
                aimingEnabled = true;

                intakeMode = true;
                spinIntake = true;
                intake.setPower(-0.567);

                if (!pathStarted) {
                    follower.followPath(paths.Path4, 1.0, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    waiting = false;
                    shootStageStarted = false;
                    stage = 6;
                }
                break;

            case 6: // SHOOT STACK
                aimingEnabled = true;
                if (!shootStageStarted) {
                    if (delayDone()) {
                        startOuttake();
                        shootStageStarted = true;
                    }
                } else {
                    if (!outtakeMode) {
                        shootStageStarted = false;
                        waiting = false;
                        stage = 7;
                    }
                }
                break;

            case 7: // SECOND STACK PASS (go)
                if (outtakeMode) break;
                aimingEnabled = false;

                intakeMode = true;
                spinIntake = true;
                intake.setPower(1);

                if (!pathStarted) {
                    follower.followPath(paths.Path5, 1.0, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    stage = 8;
                }
                break;

            case 8: // SECOND STACK PASS (sweep)
                if (outtakeMode) break;
                aimingEnabled = false;

                intakeMode = true;
                spinIntake = true;
                intake.setPower(1);

                if (!pathStarted) {
                    follower.followPath(paths.Path6, INTAKE_PASS2_SPEED, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    setSpinnerTarget(SPINNER_LAUNCH_POS);
                    pathStarted = false;
                    stage = 9;
                }
                break;

            case 9: // RETURN AGAIN
                if (outtakeMode) break;
                aimingEnabled = true;

                intakeMode = true;
                spinIntake = true;
                intake.setPower(-1);

                if (!pathStarted) {
                    follower.followPath(paths.Path7, 1.0, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    waiting = false;
                    shootStageStarted = false;
                    stage = 10;
                }
                break;

            case 10: // FINAL SHOOT
                aimingEnabled = true;
                if (!shootStageStarted) {
                    if (delayDone()) {
                        startOuttake();
                        shootStageStarted = true;
                    }
                } else {
                    if (!outtakeMode) {
                        shootStageStarted = false;
                        waiting = false;
                        stage = 11;
                    }
                }
                break;

            case 11: // PARK
                aimingEnabled = false;
                if (!pathStarted) {
                    follower.followPath(paths.Path8, 1.0, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    stage = 12;
                }
                break;

            case 12:
                requestOpModeStop();
                break;
        }

        // shooter FSM runs after stage logic
        if (outtakeMode) {
            runOuttake();
        }
    }

    /* ===================== START OUTTAKE ===================== */
    private void startOuttake() {
        intakeMode = false;
        outtakeMode = true;
        spinIntake = false;
        aimingEnabled = true;

        launchPrepActive = false;

        outtakeStep = 0;
        stepStartMs = System.currentTimeMillis();
        rpmInRangeSinceMs = 0;
    }

    /* ===================== FLYWHEEL CONTROL ===================== */
    private void updateFlywheel() {
        rpm = flywheel.getVelocity() / FLYWHEEL_TICKS_PER_REV * 60.0;

        double v = batteryVoltage.getVoltage();
        if (v < 1.0) v = 12.0;
        double kF_comp = kF_v * (12.0 / v);

        double target = flywheelTargetRPM;
        double targetTPS = target * FLYWHEEL_TICKS_PER_REV / 60.0;

        if (Math.abs(rpm - target) <= RPM_TOL) {
            if (flywheel.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
                flywheel.setPower(0);
                flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            flywheel.setVelocityPIDFCoefficients(kP_v, kI_v, kD_v, kF_comp);
            flywheel.setVelocity(targetTPS);
        } else if (rpm < (target - RPM_TOL)) {
            if (flywheel.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
                flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            flywheel.setPower(1.0);
        } else {
            if (flywheel.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
                flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            flywheel.setPower(0.0);
        }

        rpm = flywheel.getVelocity() / FLYWHEEL_TICKS_PER_REV * 60.0;
    }

    /* ===================== OUTTAKE SEQUENCE ===================== */
    private void runOuttake() {
        intake.setPower(1);

        long now = System.currentTimeMillis();
        long dt = now - stepStartMs;

        switch (outtakeStep) {
            case 0:
                slots[0] = logicalSlots[0];
                slots[1] = logicalSlots[1];
                slots[2] = logicalSlots[2];

                logicalSlots[0] = 0;
                logicalSlots[1] = 0;
                logicalSlots[2] = 0;

                Color1 = 0;
                Color2 = 0;
                Color3 = 0;

                setSpinnerTarget(SPINNER_LAUNCH_POS);

                rpmInRangeSinceMs = 0;
                startStep(1);
                break;

            case 1:
                if (dt >= OUTTAKE_INITIAL_DELAY_MS) startStep(2);
                break;

            case 2:
                if (rpmInRangeStable() && turretOnTarget) {
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
                    setSpinnerTarget(0.28);
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
                if (rpmInRangeStable() && turretOnTarget) {
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
                    setSpinnerTarget(0.46);
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
                if (rpmInRangeStable() && turretOnTarget) {
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

                    slotIntakeIndex = 0;
                    setSpinnerTarget(0.0);

                    launchPrepActive = false;
                    resetIntakeGatingAndFilters();

                    outtakeStep = 0;
                    stepStartMs = 0;
                    rpmInRangeSinceMs = 0;
                }
                break;
        }
    }

    /* ===================== COLOR DETECTION ===================== */
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

    private boolean inRange(double value, double min, double max) {
        return value >= min && value <= max;
    }

    private int smekerie1(ColorSensor colorSensor) {
        int r = colorSensor.red();
        int g = colorSensor.green();
        int b = colorSensor.blue();
        int alpha = colorSensor.alpha();
        double h = getHue(r, g, b);

        int detected;

        if (alpha < 90 && inRange(h, 140, 155)) detected = 0;
        else if (h > 210 || (alpha < 100 && inRange(h, 155, 185))) detected = 2;
        else if (inRange(h, 135, 160) && alpha > 90) detected = 1;
        else if (inRange(h, 195, 230) && alpha < 110) detected = 2;
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

    private boolean isSpindexerFull() {
        return logicalSlots[0] != 0 && logicalSlots[1] != 0 && logicalSlots[2] != 0;
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
            if (!outtakeMode) {
                intakeMode = true;
                spinIntake = true;
                intake.setPower(1.0);
            }

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

            slotIntakeIndex = (slotIntakeIndex + 1) % 3;
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

    /* ===================== PATHS (RED) ===================== */
    public static class Paths {
        public PathChain Path0, Path9, Path3, Path1, Path2, Path4, Path5, Path6, Path7, Path8;

        public Paths(Follower follower) {

            Path0 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(88.000, 8.000),
                            new Pose(82.164, 26.8194)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(90),
                            Math.toRadians(-113))
                    .build();

            Path9 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(87.000, 9.000),
                            new Pose(87.000, 12.000)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(-113),
                            Math.toRadians(-113))
                    .build();

            Path3 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(87.000, 12.000),
                            new Pose(87.000, 21.000)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(90),
                            Math.toRadians(-119))
                    .build();

            Path1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(87.000, 21.000),
                            new Pose(97.000, 33.000)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(-119),
                            Math.toRadians(0))
                    .build();

            Path2 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(97.000, 33.000),
                            new Pose(135.000, 33.000)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(0),
                            Math.toRadians(0))
                    .build();

            Path4 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(135.000, 33.000),
                            new Pose(87.000, 21.000)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(0),
                            Math.toRadians(-119))
                    .build();

            Path5 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(87.000, 21.000),
                            new Pose(123.000, 21.000)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(-119),
                            Math.toRadians(-20))
                    .build();

            Path6 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(123.000, 21.000),
                            new Pose(131.000, 12.000)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(-20),
                            Math.toRadians(-20))
                    .build();

            Path7 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(131.000, 12.000),
                            new Pose(87.000, 21.000)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(-20),
                            Math.toRadians(-119))
                    .build();

            Path8 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(87.000, 21.000),
                            new Pose(98.000, 21.000)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(-119),
                            Math.toRadians(-119))
                    .build();
        }
    }
}
