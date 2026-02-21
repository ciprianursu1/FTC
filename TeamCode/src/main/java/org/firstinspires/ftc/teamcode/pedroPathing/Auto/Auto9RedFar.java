package org.firstinspires.ftc.teamcode.pedroPathing.Auto;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.TeleOp.Main.DCSpindexer;
import org.firstinspires.ftc.teamcode.TeleOp.Main.PoseStorage;
import org.firstinspires.ftc.teamcode.TeleOp.Main.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name = "Auto9RedFar", group = "Test")
public class Auto9RedFar extends OpMode {


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
    Servo trajectoryAngleModifier;
    VoltageSensor batteryVoltage;
    DCSpindexer spinner;
    Limelight3A limelight;
    Shooter shooter;

    /* ===================== FLYWHEEL (PIDF VELOCITY) ===================== */
    static final double FLYWHEEL_TICKS_PER_REV = 28.0*1.4;

    // imported from TeleOp shooter
    public static double TARGET_RPM = 3050 ;
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
    final boolean blue = false;
    private static final double MOTOR_TICKS_PER_REV = 384.5;
    private static final double MOTOR_TO_TURRET_RATIO = 75.0 / 26.0;
    private static final double DEG_PER_TICK_TURETA =
            360.0 / (MOTOR_TICKS_PER_REV * MOTOR_TO_TURRET_RATIO);

    // Turret soft limits (degrees)
    private static final double LEFT_LIMIT  = -110;
    private static final double RIGHT_LIMIT = 110;
    int tagID = 0;
    // Control
    private static final double kP = 0.015;
    private static final double MAX_POWER_TURETA = 0.4;
    double targetX = 134;
    double targetY = 144;
    double turretX = 0.0;
    double turretY = 0.0;
    private static final double TURRET_TOL_DEG = 2.0;   // allowed error
    private boolean turretOnTarget = false;
    private static final double RPM_TOL = 75.0;
    private static final long RPM_STABLE_MS = 80;   // 40–80ms is fine in auto
    private long rpmInRangeSinceMs = 0;
    private Pose pose;
    boolean aimingEnabled = true;
    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
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
        follower.setStartingPose(new Pose(88, 8, Math.toRadians(90)));
        paths = new Paths(follower);
        intake = hardwareMap.get(DcMotor.class, "intake");
        batteryVoltage = hardwareMap.voltageSensor.iterator().next();
        // you had REVERSE in your code
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter = new Shooter(hardwareMap,"flywheel","tureta","unghituretaoy");
        shooter.init(telemetry,true);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        spinner = new DCSpindexer(hardwareMap,"Color1","Color2","Color3","spinner","ejector",telemetry);
        spinner.init(true);
        spinner.setInventory(new DCSpindexer.ArtifactColor[]{DCSpindexer.ArtifactColor.GREEN,DCSpindexer.ArtifactColor.PURPLE,DCSpindexer.ArtifactColor.PURPLE});
        autoDelay.reset();
        limelight.start();
        limelight.pipelineSwitch(4);

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
        shooter.enableLauncher();
        if(resetTimer){
            autoDelay.reset();
            resetTimer = false;
        }
        follower.update();
        pose = follower.getPose();
        spinner.update();
        Pose velocity = new Pose(0,0,pose.getHeading());
        shooter.update(pose,velocity,targetX, targetY,spinner.isReady() && spinner.requestingOuttake);
        if(spinner.requestingOuttake){
            spinner.setReady(shooter.isTurretOnTarget() && shooter.isRPMInRange());
        }


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
                    LLResult result = limelight.getLatestResult();
                    if (result.isValid() && result.getFiducialResults() != null && !result.getFiducialResults().isEmpty()) {
                        tagID = result.getFiducialResults().get(0).getFiducialId();
                        if ((tagID == 21 || tagID == 22 || tagID == 23)) {
                            spinner.setMotif(tagID);
                            spinner.enableSorting(false);
                            telemetry.addLine("Tag " + tagID + " detected");
                        }
                    } else {
                        telemetry.addLine("No fiducial detected or Limelight not connected");
                    }
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
                spinner.setInventory(new DCSpindexer.ArtifactColor[]{DCSpindexer.ArtifactColor.GREEN,DCSpindexer.ArtifactColor.PURPLE,DCSpindexer.ArtifactColor.PURPLE});
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
                spinner.setInventory(new DCSpindexer.ArtifactColor[]{DCSpindexer.ArtifactColor.GREEN,DCSpindexer.ArtifactColor.PURPLE,DCSpindexer.ArtifactColor.PURPLE});
                if (spinner.requestingOuttake) break;
                aimingEnabled = true;
                spinIntake = true;


                if (!pathStarted) {
                    follower.followPath(paths.Path4, 1.0, true);
                    pathStarted = true;
                    intake.setPower(-0.567);
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    stage = 6;
                }
                break;

            // ================== SHOOT STACK ==================
            case 6:
                spinner.setInventory(new DCSpindexer.ArtifactColor[]{DCSpindexer.ArtifactColor.GREEN,DCSpindexer.ArtifactColor.PURPLE,DCSpindexer.ArtifactColor.PURPLE});
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

                if (!pathStarted) {
                    follower.followPath(paths.Path7, 1.0, true);
                    pathStarted = true;
                    intake.setPower(-0.567);
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    waiting = false;
                    stage = 10;
                }
                break;

            // ================== FINAL SHOOT ==================
            case 10:
                spinner.setInventory(new DCSpindexer.ArtifactColor[]{DCSpindexer.ArtifactColor.GREEN,DCSpindexer.ArtifactColor.PURPLE,DCSpindexer.ArtifactColor.PURPLE});
                aimingEnabled = true;
                spinner.requestOuttake();
                stage = 11;
                break;

            // ================== PARK ==================
            case 11:
                if (spinner.requestingOuttake) break;
                spinner.cancelOuttake();
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

        PoseStorage.savePose(hardwareMap.appContext, x, y, heading, blue,tagID );
    }
    // When full -> park at launch position and STAY until outtake finishes
    /* ===================== PATHS ===================== */
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
                            new Pose(87.000, 20.000)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(90),
                            Math.toRadians(-119))
                    .build();

            Path1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(87.000, 20.000),
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
                            new Pose(87.000, 20.000)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(0),
                            Math.toRadians(-119))
                    .build();

            Path5 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(87.000, 20.000),
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
                            new Pose(87.000, 20.000)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(-20),
                            Math.toRadians(-119))
                    .build();

            Path8 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(87.000, 20.000),
                            new Pose(98.000, 21.000)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(-119),
                            Math.toRadians(-119))
                    .build();
        }
    }
}