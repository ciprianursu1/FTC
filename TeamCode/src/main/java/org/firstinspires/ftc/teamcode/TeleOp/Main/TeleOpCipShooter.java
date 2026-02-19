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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp (name = "CipShooter", group = "Alpha")
public class TeleOpCipShooter extends OpMode {
    DCSpindexer spinner;
    DcMotorSimple intake;
    DcMotor front_left;
    DcMotor front_right;
    DcMotor back_left;
    DcMotor back_right;
    Limelight3A limelight;
    PinpointLocalizer pinpoint;
    Pose startPose;
    Pose pose;
    IMU imu;
    Shooter shooter;
    static final double INCH_PER_METER = 100/2.54;
    double targetX = 0;
    double targetY = 144;
    private static final double LL_OFFSET_X = -65.5/1000;
    private static final double LL_OFFSET_Y = 181/1000.0;
    boolean blue = true;
    public static double kP_v = 30.0;
    public static double kI_v = 0.0;
    public static double kD_v = 0.0;
    public static double kF_v = 14.0;
    enum IntakeState{
        ON,OFF,REVERSE
    }
//    int lastTag = 0;
    IntakeState intakeState = IntakeState.OFF;
    IntakeState prevIntakeState = IntakeState.OFF;
    Pose velocity = new Pose(0,0,0);
    boolean limelightCorrectionMode = false;


    public void init() {
        shooter = new Shooter(hardwareMap,"flywheel","tureta","unghituretaoy");
        shooter.init(telemetry);
        spinner = new DCSpindexer(hardwareMap,"Color1","Color2","Color3","spinner","ejector",telemetry);
        spinner.init();
        intake = hardwareMap.get(DcMotorSimple.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        front_left = hardwareMap.dcMotor.get("lf");
        front_right = hardwareMap.dcMotor.get("rf");
        back_left = hardwareMap.dcMotor.get("lr");
        back_right = hardwareMap.dcMotor.get("rr");
        pinpoint = new PinpointLocalizer(hardwareMap, Constants.localizerConstants);
        startPose = new Pose(64.3, 15.74/2.0, Math.toRadians(90));
        pinpoint.setPose(startPose);
//        double[] pose = PoseStorage.loadPose(hardwareMap.appContext);
//
//        double x = pose[0];
//        double y = pose[1];
//        double heading = pose[2];
//        blue = pose[3] == 1;
//        if(blue){
//            targetX = 0;
//            targetY = 144;
//        } else {
//            targetX = 144;
//            targetY = 144;
//        }
//        pinpoint.setPose(new Pose(x,y,heading));

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
        limelight.pipelineSwitch(blue ? 5 : 6);
        limelight.start();
    }
    //    public void init_loop() {
//        LLResult result = limelight.getLatestResult();
//        if (result.isValid() && result.getFiducialResults() != null && !result.getFiducialResults().isEmpty()) {
//            int tagID = result.getFiducialResults().get(0).getFiducialId();
//            if ((tagID == 21 || tagID == 22 || tagID == 23) && tagID != lastTag) {
//                spinner.setMotif(tagID);
//                lastTag = tagID;
//                telemetry.addLine("Tag " + tagID + " detected");
//            }
//        } else {
//            telemetry.addLine("No fiducial detected or Limelight not connected");
//        }
//    }
//    public void start(){
//        limelight.pipelineSwitch(5);
//    }
    public void loop(){
        pinpoint.update();
        pose = pinpoint.getPose();
        velocity = pinpoint.getVelocity();
        if(limelightCorrectionMode) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.getBotpose_MT2() != null) {
                Pose3D LLPose = result.getBotpose_MT2();
                Pose pedroPose = PoseConverter.pose2DToPose(new Pose2D(DistanceUnit.INCH, LLPose.getPosition().x * INCH_PER_METER, LLPose.getPosition().y * INCH_PER_METER, AngleUnit.DEGREES, LLPose.getOrientation().getYaw()), InvertedFTCCoordinates.INSTANCE);
                pedroPose = processPedroPose(pedroPose);
                pinpoint.setPose(pedroPose);
            }
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            limelight.updateRobotOrientation(normalizeAngle(orientation.getYaw() - Math.toDegrees(startPose.getHeading())));
            pinpoint.setHeading(normalizeAngle(orientation.getYaw(AngleUnit.RADIANS) - startPose.getHeading())); // magnetometru rev
        }
        Drive();
        if(spinner.spindexerFull()){
            shooter.enableLauncher();
        } else {
            shooter.disableLauncher();
        }
        shooter.update(pose,velocity,limelightCorrectionMode ? 72 : targetX, targetY);
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
//            spinner.setReady(rpmInRangeStable() && turretOnTarget);
            spinner.setReady(shooter.isRPMInRange() && shooter.isTurretOnTarget());
        }
        spinner.update();
        if(gamepad2.circleWasPressed()){
            spinner.cancelOuttake();
            if(intakeState == IntakeState.ON){
                intakeState = IntakeState.OFF;
            } else {
                intakeState = IntakeState.ON;
            }
        }
        if(gamepad2.right_trigger > 0.8){
            spinner.requestOuttake();
        } else {
            spinner.cancelOuttake(); // experimental
        }
        if (gamepad2.cross) {
            if (intakeState != IntakeState.REVERSE) {
                prevIntakeState = intakeState; // remember what we were doing
                intakeState = IntakeState.REVERSE;
            }
        } else {
            // When cross released, return to previous state if we were reversing
            if (intakeState == IntakeState.REVERSE) {
                intakeState = prevIntakeState;
            }
        }
        if(gamepad2.optionsWasPressed() && gamepad2.shareWasPressed()){
            pinpoint.setPose(startPose);
            gamepad2.rumble(100);
        }
        if(gamepad2.touchpadWasPressed()){
            spinner.enabledSorting = !spinner.enabledSorting;
            gamepad2.rumbleBlips(spinner.enabledSorting ? 2 : 1);
        }
        limelightCorrectionMode = gamepad2.left_trigger > 0.5;
    }

    private Pose processPedroPose(Pose pedroPose){
        double x = pedroPose.getX();
        double y = pedroPose.getY();
        double heading = pedroPose.getHeading();
        double currentTurretAngle = shooter.getCurrentTurretDeg();
        heading = normalizeAngle(heading - currentTurretAngle);
        x += (Math.cos(heading) * LL_OFFSET_X - Math.sin(heading) * LL_OFFSET_Y)*INCH_PER_METER;
        y += (Math.sin(heading) * LL_OFFSET_X + Math.cos(heading) * LL_OFFSET_Y)*INCH_PER_METER;
        x -= (Shooter.turretOffsetX * Math.cos(heading) - Shooter.turretOffsetY * Math.sin(heading))*INCH_PER_METER;
        y -= (Shooter.turretOffsetY * Math.sin(heading) + Shooter.turretOffsetX * Math.cos(heading))*INCH_PER_METER;
        pedroPose = new Pose(x,y,heading);
        return pedroPose;
    }
    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
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