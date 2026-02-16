package org.firstinspires.ftc.teamcode.pedroPathing.Auto;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.TeleOp.Main.DCSpindexer;
import org.firstinspires.ftc.teamcode.TeleOp.Main.PoseStorage;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name = "auto9BlueFarDC", group = "Test")
public class AutoDC extends OpMode {


    /* ===================== PEDRO ===================== */
    public Follower follower;
    private boolean pathStarted = false;
    private Paths paths;

    /* ===================== AUTONOMOUS STAGE ===================== */
    int stage = 0;
    /* ===================== DELAY GATE ===================== */
    ElapsedTime autoDelay = new ElapsedTime();
    boolean waiting = false;

    /* ===================== HARDWARE ===================== */
    DcMotor intake;
    DcMotorEx tureta;

    DcMotorEx flywheel;
    Servo trajectoryAngleModifier;
    VoltageSensor batteryVoltage;
    DCSpindexer spinner;


    /* ===================== FLYWHEEL (PIDF VELOCITY) ===================== */
    static final double FLYWHEEL_TICKS_PER_REV = 28.0;

    // imported from TeleOp shooter
    public static double TARGET_RPM = 2940 ;
    // Start aggressive; tune after you get fast spin-up
    public static double kP_v = 30.0;     // try 20–35
    public static double kI_v = 0.0;      // keep 0 for fastest transient
    public static double kD_v = 0.0;      // add small later only if it overshoots/oscillates

    // Correct ballpark for 6000rpm Yellow Jacket (28tpr): ~11.7 at 12V no-load
    public static double kF_v = 14.0;     // try 12.0–14.0

    private double rpm = 0.0;
    // Slow only the actual intake/sweep segments
    private static final double INTAKE_SWEEP_SPEED = 0.3;   // case 4 (was 0.6)
    private static final double INTAKE_PASS2_SPEED = 0.35;   // case 8 (was 1.0)
    private static final double AUTO_TOTAL_S = 30.0;
    private static final double PARK_IF_REMAIN_S = 2.0;

    boolean spinIntake = false;
    final boolean blue = true;
    private static final double MOTOR_TICKS_PER_REV = 384.5;
    private static final double MOTOR_TO_TURRET_RATIO = 75.0 / 26.0;
    private static final double DEG_PER_TICK_TURETA =
            360.0 / (MOTOR_TICKS_PER_REV * MOTOR_TO_TURRET_RATIO);

    // Turret soft limits (degrees)
    private static final double LEFT_LIMIT  = -110;
    private static final double RIGHT_LIMIT = 110;

    // Control
    private static final double kP = 0.015;
    private static final double MAX_POWER_TURETA = 0.4;
    double targetX = 5;
    double targetY = 144;
    double turretX = 0.0;
    double turretY = 0.0;
    private static final double TURRET_TOL_DEG = 2.0;   // allowed error
    private boolean turretOnTarget = false;
    private static final double RPM_TOL = 50.0;
    private static final long RPM_STABLE_MS = 150;   // 40–80ms is fine in auto
    private long rpmInRangeSinceMs = 0;
    private Pose pose;
    boolean aimingEnabled = true;
    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }
    private void updateTurretAim() {

        double robotX = pose.getX();
        double robotY = pose.getY();
        double robotHeading = pose.getHeading();
        double robotHeadingDeg = Math.toDegrees(robotHeading);
        turretX = robotX;
        turretY = robotY;
        double dx = targetX - turretX;
        double dy = targetY - turretY;

        double fieldAngle = Math.toDegrees(Math.atan2(dy, dx));

        double startTurretAngle = -180.0;
        double currentTurretDeg = tureta.getCurrentPosition() * DEG_PER_TICK_TURETA + startTurretAngle;
        double targetTurretDeg;
        if(aimingEnabled) {
            targetTurretDeg = normalizeAngle(fieldAngle - robotHeadingDeg);
        } else {
            targetTurretDeg = startTurretAngle;
        }
        currentTurretDeg=normalizeAngle(currentTurretDeg);
        if(targetTurretDeg < 0 && targetTurretDeg > LEFT_LIMIT){
            targetTurretDeg = LEFT_LIMIT;
        } else if (targetTurretDeg >= 0 && targetTurretDeg < RIGHT_LIMIT) {
            targetTurretDeg = RIGHT_LIMIT;
        }

        double error = normalizeAngle(targetTurretDeg - currentTurretDeg);
        turretOnTarget = Math.abs(error) <= TURRET_TOL_DEG;

        double power = error * kP;
        power = Range.clip(power, -MAX_POWER_TURETA, MAX_POWER_TURETA);
        tureta.setPower(power);
    }


    private void abortOuttakeNow() {
        spinner.setReady(false);
        spinner.cancelOuttake();
        spinIntake = false;
        intake.setPower(0);
        rpmInRangeSinceMs = 0;
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
    private boolean resetTimer = true;

    /* ===================== INIT ===================== */
    @Override
    public void init() {

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


        trajectoryAngleModifier = hardwareMap.get(Servo.class, "unghituretaoy");
        trajectoryAngleModifier.setPosition(0);
        spinner = new DCSpindexer(hardwareMap,"Color1","Color2","Color3","spinner","ejector");

        autoDelay.reset();

        // Start state
        stage = 0;
        pathStarted = false;
        spinIntake = true;
        rpmInRangeSinceMs = 0;

        waiting = false;

    }


    /* ===================== LOOP ===================== */
    @Override
    public void loop() {
        if(resetTimer){
            autoDelay.reset();
            resetTimer = false;
        }
        follower.update();
        updateFlywheel();
        spinner.update();
        updateTurretAim();
        if(spinner.requestingOuttake){
            spinner.setReady(turretOnTarget && rpmInRangeStable());
        }
        pose = follower.getPose();

        double remain = AUTO_TOTAL_S - autoDelay.seconds();
        if (remain <= PARK_IF_REMAIN_S && stage < 11) {
            // force park ASAP
            abortOuttakeNow();
            stage = 11;
            pathStarted = false;     // so case 11 will start the park path immediately
        }

        switch (stage) {

            // ================== INITIAL POSITION ==================
            case 0:
                if (spinner.requestingOuttake) break;
                if (!pathStarted) {
                    follower.followPath(paths.Path0, 1, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    stage = 1;
                }
                break;

            case 1:
                aimingEnabled = true;
                if (!pathStarted) {
                    follower.followPath(paths.Path9, 1.0, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    stage = 2;   // SHOOT
                }
                break;

            // ================== SHOOT FIRST 3 ==================
            case 2:
                aimingEnabled = true;
                spinner.requestOuttake();
                stage = 3;
                break;

            // ================== GO TO STACK ==================
            case 3:
                aimingEnabled = false;
                if (spinner.requestingOuttake) break;
                spinner.cancelOuttake();
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
                if (spinner.requestingOuttake) break;
                aimingEnabled = false;
                spinIntake = true;
                intake.setPower(1);

                if (!pathStarted) {
                    follower.followPath(paths.Path2, INTAKE_SWEEP_SPEED, true);
                    pathStarted = true;
                }

                if (!follower.isBusy()) {
                    pathStarted = false;
                    stage = 5;
                }
                break;

            // ================== RETURN TO SHOOT ==================
            case 5:
                if (spinner.requestingOuttake) break;
                aimingEnabled = true;
                spinner.cancelOuttake();
                spinIntake = true;
                intake.setPower(-0.567);


                if (!pathStarted) {
                    follower.followPath(paths.Path4, 1.0, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    stage = 6;
                }
                break;

            // ================== SHOOT STACK ==================
            case 6:
                aimingEnabled = true;
                spinner.requestOuttake();
                stage = 7;
                break;

            // ================== SECOND STACK PASS ==================
            case 7:
                if (spinner.requestingOuttake) break;
                aimingEnabled = false;
                spinner.cancelOuttake();
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
                aimingEnabled = false;
                if (spinner.requestingOuttake) break;

                spinner.cancelOuttake();
                spinIntake = true;
                intake.setPower(1);

                if (!pathStarted) {
                    follower.followPath(paths.Path6, INTAKE_PASS2_SPEED, true);
                    pathStarted = true;
                }

                if ( !follower.isBusy()) {
                    pathStarted = false;
                    stage = 9;
                }
                break;

            // ================== RETURN AGAIN ==================
            case 9:
                if (spinner.requestingOuttake) break;
                aimingEnabled = true;
                spinner.cancelOuttake();
                spinIntake = true;
                intake.setPower(-1);

                if (!pathStarted) {
                    follower.followPath(paths.Path7, 1.0, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    waiting = false;
                    stage = 10;
                }
                break;

            // ================== FINAL SHOOT ==================
            case 10:
                aimingEnabled = true;
                spinner.requestOuttake();
                stage = 11;
                break;

            // ================== PARK ==================
            case 11:
                if (spinner.requestingOuttake) break;
                spinner.cancelOuttake();
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
    }
    @Override
    public void stop() {
        double x = pose.getX();
        double y = pose.getY();
        double heading = pose.getHeading();

        PoseStorage.savePose(hardwareMap.appContext, x, y, heading, blue);
    }
    private void updateFlywheel() {
        // Measure RPM
        rpm = flywheel.getVelocity() / FLYWHEEL_TICKS_PER_REV * 60.0;

        // Voltage compensation for feedforward
        double v = batteryVoltage.getVoltage();
        if (v < 1.0) v = 12.0; // safety
        double kF_comp = kF_v * (12.0 / v);

        double target = TARGET_RPM;

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
    // When full -> park at launch position and STAY until outtake finishes
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
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(293))
                    .build();

            Path9 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(57.000, 9.000),
                            new Pose(57.000, 12.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(293), Math.toRadians(293))
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
                            new Pose(47.000, 33.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(299), Math.toRadians(180))
                    .build();

            Path2 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(42.000, 33.000),
                            new Pose(9.000, 33.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path4 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(9.000, 33.000),
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