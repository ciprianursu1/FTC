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
import org.firstinspires.ftc.teamcode.TeleOp.Main.Shooter;

@Autonomous(name = "Auto12BlueClose", group = "Test")
public class Auto12BlueClose extends OpMode {

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
    VoltageSensor batteryVoltage;
    DCSpindexer spinner;
    Limelight3A limelight;
    Shooter shooter;

    /* ===================== FLYWHEEL (PIDF VELOCITY) ===================== */
    static final double FLYWHEEL_TICKS_PER_REV = 28.0 * 1.4;

    // imported from TeleOp shooter
    public static double TARGET_RPM = 2350;
    public static double kP_v = 50.0;
    public static double kI_v = 0.0;
    public static double kD_v = 0.5;
    public static double kF_v = 14.0;
    public int targetX = 10;
    public int targetY = 144;
    public Pose velocity = new Pose(0,0,0);

    private double rpm = 0.0;

    // Slow only the actual intake/sweep segments
    private static final double INTAKE_SWEEP_SPEED    = 0.3;
    private static final double INTAKE_PASS2_SPEED    = 0.35;
    private static final double AUTO_TOTAL_S          = 30.0;
    private static final double PARK_IF_REMAIN_S      = 2.0;

    boolean spinIntake = false;
    final boolean blue = true;
    int tagID = 0;

    private Pose pose;
    boolean aimingEnabled = true;

    private boolean resetTimer = true;

    private void abortOuttakeNow() {
        spinner.setReady(false);
        spinner.cancelOuttake();
        spinIntake = false;
        intake.setPower(0);
    }

    /* ===================== INIT ===================== */
    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(22, 125, Math.toRadians(325)));

        paths = new Paths(follower);

        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        batteryVoltage = hardwareMap.voltageSensor.iterator().next();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();
        limelight.pipelineSwitch(4);

        shooter = new Shooter(hardwareMap, "flywheel", "tureta", "unghituretaoy");
        shooter.init(telemetry);

        spinner = new DCSpindexer(hardwareMap, "Color1", "Color2", "Color3", "spinner", "ejector", telemetry);
        spinner.init();

        // Start state
        stage = 0;
        pathStarted = false;
        spinIntake = true;
        aimingEnabled = true;
        waiting = false;
        autoDelay.reset();
    }

    /* ===================== LOOP ===================== */
    @Override
    public void loop() {
        shooter.enableLauncher();

        if (resetTimer) {
            autoDelay.reset();
            resetTimer = false;
        }
        shooter.update(pose,velocity,targetX, targetY,spinner.isReady() && spinner.requestingOuttake);
        follower.update();
        pose = follower.getPose();
        spinner.update();

        if (spinner.requestingOuttake) {
            spinner.setReady(shooter.isTurretOnTarget() && shooter.isRPMInRange());
        }

        double remain = AUTO_TOTAL_S - autoDelay.seconds();
        if (remain <= PARK_IF_REMAIN_S && stage < 15) {
            abortOuttakeNow();
            stage = 15;
            pathStarted = false;
        }

        switch (stage) {

            // ================== INITIAL POSITION + TAG SCAN ==================
            case 0:
                if (spinner.requestingOuttake) break;
                if (!pathStarted) {
                    follower.followPath(paths.Path1, 1.0, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    LLResult result = limelight.getLatestResult();
                    if (result.isValid() && result.getFiducialResults() != null && !result.getFiducialResults().isEmpty()) {
                        tagID = result.getFiducialResults().get(0).getFiducialId();
                        if (tagID == 21 || tagID == 22 || tagID == 23) {
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
                    follower.followPath(paths.Path2, 1.0, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    stage = 2;
                }
                break;

            // ================== SHOOT FIRST 3 ==================
            case 2:
                spinner.updateInventory();
                aimingEnabled = true;
                spinner.requestOuttake();
                stage = 3;
                break;

            // ================== GO TO STACK ==================
            case 3:
                if (spinner.requestingOuttake) break;
                spinner.cancelOuttake();
                aimingEnabled = false;
                spinIntake = true;
                intake.setPower(1.0);

                if (!pathStarted) {
                    follower.followPath(paths.Path3, 1.0, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    stage = 4;
                }
                break;

            // ================== SWEEP STACK 1 ==================
            case 4:
                if (spinner.requestingOuttake) break;
                aimingEnabled = false;
                spinIntake = true;
                intake.setPower(1.0);

                if (!pathStarted) {
                    follower.followPath(paths.Path4, INTAKE_SWEEP_SPEED, true);
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

                if (!pathStarted) {
                    follower.followPath(paths.Path5, 1.0, true);
                    pathStarted = true;
                    intake.setPower(-0.567);
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    stage = 6;
                }
                break;

            // ================== SHOOT STACK 1 ==================
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
                intake.setPower(1.0);

                if (!pathStarted) {
                    follower.followPath(paths.Path6, 1.0, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    stage = 8;
                }
                break;

            case 8:
                if (spinner.requestingOuttake) break;
                aimingEnabled = false;
                spinner.cancelOuttake();
                spinIntake = true;
                intake.setPower(1.0);

                if (!pathStarted) {
                    follower.followPath(paths.Path7, INTAKE_PASS2_SPEED, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
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
                    follower.followPath(paths.Path8, 1.0, true);
                    pathStarted = true;
                    intake.setPower(-0.567);
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    stage = 10;
                }
                break;

            // ================== FINAL SHOOT 1 ==================
            case 10:
                aimingEnabled = true;
                spinner.requestOuttake();
                stage = 11;
                break;

            // ================== THIRD STACK PASS ==================
            case 11:
                if (spinner.requestingOuttake) break;
                aimingEnabled = false;
                spinner.cancelOuttake();
                spinIntake = true;
                intake.setPower(1.0);

                if (!pathStarted) {
                    follower.followPath(paths.Path9, 1.0, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    stage = 12;
                }
                break;

            case 12:
                if (spinner.requestingOuttake) break;
                aimingEnabled = false;
                spinner.cancelOuttake();
                spinIntake = true;
                intake.setPower(1.0);

                if (!pathStarted) {
                    follower.followPath(paths.Path10, INTAKE_PASS2_SPEED, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    stage = 13;
                }
                break;

            // ================== RETURN AGAIN ==================
            case 13:
                if (spinner.requestingOuttake) break;
                aimingEnabled = true;
                spinner.cancelOuttake();
                spinIntake = true;

                if (!pathStarted) {
                    follower.followPath(paths.Path11, 1.0, true);
                    pathStarted = true;
                    intake.setPower(-0.567);
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    stage = 14;
                }
                break;

            // ================== FINAL SHOOT 2 ==================
            case 14:
                aimingEnabled = true;
                spinner.requestOuttake();
                stage = 15;
                break;

            // ================== PARK ==================
            case 15:
                if (spinner.requestingOuttake) break;
                spinner.cancelOuttake();
                aimingEnabled = false;

                if (!pathStarted) {
                    follower.followPath(paths.Path12, 1.0, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    stage = 16;
                }
                break;

            case 16:
                requestOpModeStop();
                break;
        }

        telemetry.addData("RPM", rpm);
        telemetry.update();
    }

    @Override
    public void stop() {
        double x = pose.getX();
        double y = pose.getY();
        double heading = pose.getHeading();
        PoseStorage.savePose(hardwareMap.appContext, x, y, heading, blue, tagID);
    }

    /* ===================== PATHS ===================== */
    public static class Paths {
        public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9, Path10, Path11, Path12;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(22.000, 125.000),
                            new Pose(72.897, 103.178)))
                    .setLinearHeadingInterpolation(Math.toRadians(325), Math.toRadians(270))
                    .build();

            Path2 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(72.897, 103.178),
                            new Pose(58.093, 85.234)))
                    .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(310))
                    .build();

            Path3 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(58.093, 85.234),
                            new Pose(48.423, 84.905)))
                    .setLinearHeadingInterpolation(Math.toRadians(310), Math.toRadians(180))
                    .build();

            Path4 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(48.423, 84.905),
                            new Pose(12.785, 83.888)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path5 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(12.785, 83.888),
                            new Pose(57.869, 85.234)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(310))
                    .build();

            Path6 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(57.869, 85.234),
                            new Pose(48.011, 61.264)))
                    .setLinearHeadingInterpolation(Math.toRadians(310), Math.toRadians(180))
                    .build();

            Path7 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(48.011, 61.264),
                            new Pose(5.316, 57.906)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path8 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(5.316, 57.906),
                            new Pose(57.869, 85.009)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(310))
                    .build();

            Path9 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(57.869, 85.009),
                            new Pose(18.168, 62.131)))
                    .setLinearHeadingInterpolation(Math.toRadians(310), Math.toRadians(150))
                    .build();

            Path10 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(18.168, 62.131),
                            new Pose(11.215, 65.047)))
                    .setLinearHeadingInterpolation(Math.toRadians(150), Math.toRadians(150))
                    .build();

            Path11 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(11.215, 65.047),
                            new Pose(58.093, 84.561)))
                    .setLinearHeadingInterpolation(Math.toRadians(150), Math.toRadians(310))
                    .build();

            Path12 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(58.093, 84.561),
                            new Pose(41.000, 64.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(310), Math.toRadians(0))
                    .build();
        }
    }
}