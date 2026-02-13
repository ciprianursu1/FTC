package org.firstinspires.ftc.teamcode.pedroPathing.Auto;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.Range;
@Autonomous(name = "auto9BlueFar", group = "Test")
public class Auto extends OpMode {

    /* ===================== TELEMETRY ===================== */
    private TelemetryManager panelsTelemetry;

    /* ===================== PEDRO ===================== */
    public Follower follower;
    private int pathState = 0;
    private boolean pathStarted = false;
    private Paths paths;

    /* ===================== AUTONOMOUS STAGE ===================== */
    int stage = 0;
    /* ===================== DELAY GATE ===================== */
    ElapsedTime autoDelay = new ElapsedTime();
    boolean waiting = false;
    final double COMMAND_DELAY = 0.0; // set to 0 for instant; change if you want a pause before each shoot

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
    DcMotor intake;
    DcMotorEx tureta;

    DcMotorEx flywheel;
    Servo spinnerCLose, spinnerFar, ejector;
    ColorSensor colorsensorSLot1, colorsensorSLot2, colorsensorSLot3;
    Servo trajectoryAngleModifier;
    VoltageSensor batteryVoltage;


    /* ===================== FLYWHEEL (PIDF VELOCITY) ===================== */
    static final double FLYWHEEL_TICKS_PER_REV = 28.0;

    // imported from TeleOp shooter
    public static double TARGET_RPM = 3150.0;

    // Start aggressive; tune after you get fast spin-up
    public static double kP_v = 30.0;     // try 20–35
    public static double kI_v = 0.0;      // keep 0 for fastest transient
    public static double kD_v = 0.0;      // add small later only if it overshoots/oscillates

    // Correct ballpark for 6000rpm Yellow Jacket (28tpr): ~11.7 at 12V no-load
    public static double kF_v = 14.0;     // try 12.0–14.0

    private double targetTPS;
    private double rpm = 0.0;
    // Slow only the actual intake/sweep segments
    private static final double INTAKE_SWEEP_SPEED = 0.25;   // case 4 (was 0.6)
    private static final double INTAKE_PASS2_SPEED = 0.35;   // case 8 (was 1.0)
    private static final double AUTO_TOTAL_S = 30.0;
    private static final double PARK_IF_REMAIN_S = 2.0;



    /* ===================== SENSOR SMOOTHING (debug) ===================== */
    int[] last5Sensor1 = new int[5];
    int[] last5Sensor2 = new int[5];
    int[] last5Sensor3 = new int[5];

    int indexSensor1 = 0;
    int indexSensor2 = 0;
    int indexSensor3 = 0;

    int Color1 = 0;
    int Color2 = 0;
    int Color3 = 0;

    /* ===================== OUTTAKE SLOTS ===================== */
    int[] slots = new int[3];
    int slotIntakeIndex = 0;

    /* ===================== MODES ===================== */
    boolean intakeMode = false;
    boolean outtakeMode = false;
    boolean spinIntake = false;

    final double ejectorDown = 0.255;
    final double ejectorUp = 0.03;

    final double[] slotPositionsIntake = {0, 0.19, 0.38};
    double Posspinner = 0;

    /* ===================== SPINDEXER OFFSETS / POSITIONS ===================== */
    private static final double SPINDEXER_OUTTAKE_OFFSET = -0.015;
    private static final double SPINDEXER_INTAKE_OFFSET = -0.01;
    private static final double SPINNER_LAUNCH_POS = 0.085;

    private boolean launchPrepActive = false;

    /* ===================== LOGICAL INVENTORY ===================== */
    int[] logicalSlots = new int[3];

    // intake detect internal
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

    /* =======================================================================================
       SHOOTING MECHANISM (PID + RPM GATE + FSM) IMPORTED FROM TeleOp
       ======================================================================================= */

    private int outtakeStep = 0;
    private long stepStartMs = 0;

    private static final double RPM_TOL = 100.0;
    private static final long RPM_STABLE_MS = 60;   // 40–80ms is fine in auto
    private long rpmInRangeSinceMs = 0;
    private Pose robotPose;


    private void abortOuttakeNow() {
        // put everything in a safe state and stop the shooter FSM
        outtakeMode = false;
        shootStageStarted = false;
        waiting = false;

        ejector.setPosition(ejectorDown);
        setSpinnerTarget(0.0);

        intakeMode = false;
        spinIntake = false;
        intake.setPower(0);

        outtakeStep = 0;
        stepStartMs = 0;
        rpmInRangeSinceMs = 0;

        launchPrepActive = false;
        resetIntakeGatingAndFilters();
    }

    private boolean rpmInRangeStable() {
        double target = TARGET_RPM; // in launch position we compute it; else it gets set to baseline
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

    private static final long OUTTAKE_INITIAL_DELAY_MS = 200;
    private static final long OUTTAKE_EJECTOR_UP_MS = 250;
    private static final long OUTTAKE_EJECTOR_DOWN_MS = 170;
    private static final long OUTTAKE_SPINNER_MOVE_MS = 200;

    // latch so shoot stages BLOCK until outtakeMode finishes
    private boolean shootStageStarted = false;

    // Pause after each guaranteed intake ball
    private static final double POST_BALL_PAUSE_S = 0.5;
    private final ElapsedTime postBallTimer = new ElapsedTime();
    private boolean postBallWaiting = false;

    private boolean postBallPauseDone() {
        if (!postBallWaiting) {
            postBallWaiting = true;
            postBallTimer.reset();
            return false;
        }
        if (postBallTimer.seconds() >= POST_BALL_PAUSE_S) {
            postBallWaiting = false;
            return true;
        }
        return false;
    }

    /* ===================== INIT ===================== */
    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(56, 8, Math.toRadians(90)));
        paths = new Paths(follower);

        intake = hardwareMap.get(DcMotor.class, "intake");
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        tureta = hardwareMap.get(DcMotorEx.class, "tureta");
        tureta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tureta.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        batteryVoltage = hardwareMap.voltageSensor.iterator().next();

        // you had REVERSE in your code
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        spinnerCLose = hardwareMap.get(Servo.class, "SpinnerClose");
        spinnerFar = hardwareMap.get(Servo.class, "SpinnerFar");
        ejector = hardwareMap.get(Servo.class, "ejector");

        trajectoryAngleModifier = hardwareMap.get(Servo.class, "unghituretaoy");
        trajectoryAngleModifier.setPosition(0);

        colorsensorSLot1 = hardwareMap.colorSensor.get("Color1");
        colorsensorSLot2 = hardwareMap.colorSensor.get("Color2");
        colorsensorSLot3 = hardwareMap.colorSensor.get("Color3");

        ejector.setDirection(Servo.Direction.REVERSE);
        ejector.setPosition(ejectorDown);

        spinnerFar.setPosition(0);
        spinnerCLose.setPosition(0);

        autoDelay.reset();

        // Start state
        stage = 0;
        pathState = 0;
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

        postBallWaiting = false;
        postBallTimer.reset();

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
        follower.update();

        updateFlywheel();
        updateCulori();
        updateSpinnerServos();

        robotPose = follower.getPose();

        double remain = AUTO_TOTAL_S - getRuntime();
        if (remain <= PARK_IF_REMAIN_S && stage < 11) {
            // force park ASAP
            abortOuttakeNow();
            stage = 11;
            pathStarted = false;     // so case 11 will start the park path immediately
        }

        // intake detection only while intaking and not shooting and not launch holding
        if (intakeMode && !outtakeMode && !launchPrepActive) {
            colorDrivenSpinnerLogicServos();
        }

        // auto park at launch when full
        autoLaunchPrepLogic();


        // ===================== AUTONOMOUS FSM =====================
        // ===================== AUTONOMOUS FSM =====================
        switch (stage) {

            // ================== INITIAL POSITION ==================
            case 0:
                if (outtakeMode) break;
                if (!pathStarted) {
                    follower.followPath(paths.Path0, 1, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    waiting = false;
                    shootStageStarted = false;
                    stage = 1;
                }
                break;

            // ================== GO TO SHOOT LINE ==================
            case 1:
                if (!pathStarted) {
                    follower.followPath(paths.Path9, 1.0, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    waiting = false;
                    shootStageStarted = false;
                    stage = 2;   // SHOOT
                }
                break;

            // ================== SHOOT FIRST 3 ==================
            case 2:
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

            // ================== GO TO STACK ==================
            case 3:
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

            // ================== SWEEP STACK ==================
            case 4:
                if (outtakeMode) break;

                intakeMode = true;
                spinIntake = true;
                intake.setPower(1);

                if (!pathStarted) {
                    follower.followPath(paths.Path2, INTAKE_SWEEP_SPEED, true);
                    pathStarted = true;
                }

                if (isSpindexerFull() || !follower.isBusy()) {
                    pathStarted = false;
                    stage = 5;
                }
                break;

            // ================== RETURN TO SHOOT ==================
            case 5:
                if (outtakeMode) break;

                intakeMode = true;
                spinIntake = true;
                intake.setPower(-1);
                setSpinnerTarget(0.085);

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

            // ================== SHOOT STACK ==================
            case 6:
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

            // ================== SECOND STACK PASS ==================
            case 7:
                if (outtakeMode) break;

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

            case 8:
                if (outtakeMode) break;

                intakeMode = true;
                spinIntake = true;
                intake.setPower(1);

                if (!pathStarted) {
                    follower.followPath(paths.Path6, INTAKE_PASS2_SPEED, true);
                    pathStarted = true;
                }

                if (isSpindexerFull() || !follower.isBusy()) {
                    pathStarted = false;
                    stage = 9;
                }
                break;

            // ================== RETURN AGAIN ==================
            case 9:
                if (outtakeMode) break;

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

            // ================== FINAL SHOOT ==================
            case 10:
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

            // ================== PARK ==================
            case 11:
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


        // run shooter FSM after stage logic
        if (outtakeMode) {
            runOuttake();
        }

        // ===================== TELEMETRY =====================
        panelsTelemetry.debug("Robot X", robotPose.getX());
        panelsTelemetry.debug("Robot Y", robotPose.getY());
        panelsTelemetry.debug("Robot Heading (deg)", Math.toDegrees(robotPose.getHeading()));


        panelsTelemetry.debug("Auto Stage", stage);
        panelsTelemetry.debug("Path Started", pathStarted);
        panelsTelemetry.debug("Follower Busy", follower.isBusy());

        panelsTelemetry.debug("Flywheel RPM", rpm);
        panelsTelemetry.debug("Flywheel Target", TARGET_RPM);
        panelsTelemetry.debug("rpmStable", rpmInRangeStable());


        panelsTelemetry.update(telemetry);
    }

    /* ===================== START OUTTAKE ===================== */
    private void startOuttake() {
        intakeMode = false;
        outtakeMode = true;
        spinIntake = false;

        // feeding during shooting (with REVERSE direction, power(1) is "reverse")

        // prevent launch-hold overwriting during sequence
        launchPrepActive = false;

        // reset shooter FSM
        outtakeStep = 0;
        stepStartMs = System.currentTimeMillis();
        rpmInRangeSinceMs = 0;

    }

    /* ===================== FLYWHEEL ===================== */
    /* ===================== FLYWHEEL ===================== */
    private void updateFlywheel() {
        // Measure RPM
        rpm = flywheel.getVelocity() / FLYWHEEL_TICKS_PER_REV * 60.0;

        // Voltage compensation for feedforward
        double v = batteryVoltage.getVoltage();
        if (v < 1.0) v = 12.0; // safety
        double kF_comp = kF_v * (12.0 / v);

        double target = TARGET_RPM;

        // Convert target to ticks/sec for RUN_USING_ENCODER velocity
        targetTPS = target * FLYWHEEL_TICKS_PER_REV / 60.0;

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
            flywheel.setVelocityPIDFCoefficients(kP_v, kI_v, kD_v, kF_comp);
            flywheel.setVelocity(targetTPS);
        } else if (rpm < (target - RPM_TOL)) {
            // KICK UP FAST
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





    /* ===================== OUTTAKE SEQUENCE (TeleOp shooter FSM) ===================== */
    private void runOuttake() {
        intake.setPower(1);

        long now = System.currentTimeMillis();
        long dt = now - stepStartMs;

        switch (outtakeStep) {

            case 0:
                // snapshot inventory once
                slots[0] = logicalSlots[0];
                slots[1] = logicalSlots[1];
                slots[2] = logicalSlots[2];

                // clear logical so intake recounts after
                logicalSlots[0] = 0;
                logicalSlots[1] = 0;
                logicalSlots[2] = 0;

                Color1 = 0;
                Color2 = 0;
                Color3 = 0;

                // start at launch position
                setSpinnerTarget(0.085);

                rpmInRangeSinceMs = 0;
                startStep(1);
                break;

            case 1:
                if (dt >= OUTTAKE_INITIAL_DELAY_MS) startStep(2);
                break;

            case 2:
                // SHOOT #1 only when rpm stable in range
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
                // SHOOT #2
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
                // SHOOT #3
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

                    // end
                    setSpinnerTarget(0);

                    outtakeMode = false;

                    intakeMode = false;
                    spinIntake = false;

                    slotIntakeIndex = 0;
                    setSpinnerTarget(0);

                    launchPrepActive = false;
                    resetIntakeGatingAndFilters();

                    // reset shooter FSM
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

    // ULTRA-FAST intake smoothing: 3 samples, need 2
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
            // Spinner should not move without intake pulling (friction reducer / ball seating)
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
        if (newColorDetected) {
            lastStableIntakeColor = intakeColor;
        }

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

    // When full -> park at launch position and STAY until outtake finishes
    private void autoLaunchPrepLogic() {
        if (outtakeMode) return;

        if (intakeMode && isSpindexerFull()) {
            launchPrepActive = true;
        }
        if (launchPrepActive) {
            setSpinnerTarget(SPINNER_LAUNCH_POS);
        }
    }

    /* ===================== PATHS ===================== */
    public static class Paths {
        public PathChain Path0;

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
            Path0 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(56, 8), new Pose(61.836, 26.8194)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(291.5))
                    .build();

            Path9 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(57.000, 9.000),
                            new Pose(57.000, 12.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(291.5), Math.toRadians(291.5))
                    .build();

            Path3 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(57.000, 12.000),
                            new Pose(57.000, 21.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(299))
                    .build();

            Path1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(57.000, 21.000),
                            new Pose(47.000, 36.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(299), Math.toRadians(180))
                    .build();

            Path2 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(42.000, 36.000),
                            new Pose(9.000, 36.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path4 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(9.000, 36.000),
                            new Pose(57.000, 21.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(299))
                    .build();

            Path5 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(57.000, 21.000),
                            new Pose(12.000, 21.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(299), Math.toRadians(200))
                    .build();

            Path6 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(12.000, 21.000),
                            new Pose(12.000, 12.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(200), Math.toRadians(200))
                    .build();

            Path7 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(12.000, 12.000),
                            new Pose(57.000, 21.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(200), Math.toRadians(299))
                    .build();

            Path8 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(57.000, 21.000),
                            new Pose(39.000, 12.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(299), Math.toRadians(180))
                    .build();
        }
    }
}