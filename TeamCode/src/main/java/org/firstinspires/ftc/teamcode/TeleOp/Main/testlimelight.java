package org.firstinspires.ftc.teamcode.TeleOp.Main;

import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "OdometryIMUTestTeleOp")
public class testlimelight extends LinearOpMode {

    DcMotor frontLeft, frontRight, backLeft, backRight;

    PinpointLocalizer pinpoint;
    Pose pose;
    Pose startPose = new Pose(56, 8, Math.toRadians(90));

    Limelight3A limelight;

    BNO055IMU imu;
    Orientation imuAngles;

    // meters → inches conversion
    private static final double METERS_TO_INCHES = 39.3701;

    @Override
    public void runOpMode() {

        initDrive();
        initIMU();
        initLocalization();
        initLimelight();

        waitForStart();

        while (opModeIsActive()) {

            drive();

            // Update odometry
            pinpoint.update();

            // Correct heading using IMU
            double imuHeading = getIMUHeading();
            pose = pinpoint.getPose();
            pose.setHeading(imuHeading); // override heading with IMU

            // Reset odometry if options+share pressed
            if (gamepad1.options && gamepad1.share) {
                pinpoint.setPose(startPose);
            }

            // Get Limelight results
            LLResult result = limelight.getLatestResult();

            telemetry.addLine("========== ODOMETRY ==========");
            telemetry.addData("Odo X (in)", "%.2f", pose.getX());
            telemetry.addData("Odo Y (in)", "%.2f", pose.getY());
            telemetry.addData("Odo Heading (deg)", "%.1f", Math.toDegrees(pose.getHeading()));

            telemetry.addLine("========== LIMELIGHT ==========");
            if (result != null && result.isValid()) {
                Pose3D botPose = result.getBotpose();

                if (botPose != null) {
                    // Convert meters → inches
                    double visionX = botPose.getPosition().x * METERS_TO_INCHES;
                    double visionY = botPose.getPosition().y * METERS_TO_INCHES;
                    double visionHeading = botPose.getOrientation().getYaw();

                    telemetry.addData("Vision X (in)", "%.2f", visionX);
                    telemetry.addData("Vision Y (in)", "%.2f", visionY);
                    telemetry.addData("Vision Heading (deg)", "%.1f", Math.toDegrees(visionHeading));

                    Pose visionPedroPose = new Pose(visionX, visionY, visionHeading);

                    // Press A to sync odometry to vision
                    if (gamepad1.a) {
                        pinpoint.setPose(visionPedroPose);
                    }
                }
            } else {
                telemetry.addData("Has Target", false);
            }

            telemetry.update();
        }
    }

    private void initDrive() {
        frontLeft  = hardwareMap.dcMotor.get("lf");
        frontRight = hardwareMap.dcMotor.get("rf");
        backLeft   = hardwareMap.dcMotor.get("lr");
        backRight  = hardwareMap.dcMotor.get("rr");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    private void initIMU() {
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu.initialize(parameters);
    }

    private double getIMUHeading() {
        imuAngles = imu.getAngularOrientation();
        // Radians, heading around Z axis
        return imuAngles.firstAngle;
    }

    private void initLocalization() {
        pinpoint = new PinpointLocalizer(hardwareMap, Constants.localizerConstants);
        pinpoint.setStartPose(startPose);
    }

    private void initLimelight() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(4); // AprilTag pipeline
    }

    private void drive() {
        double y  = -gamepad1.left_stick_y;
        double x  =  gamepad1.left_stick_x;
        double rx =  gamepad1.right_stick_x;

        double fl = y + x + rx;
        double bl = y - x + rx;
        double fr = y - x - rx;
        double br = y + x - rx;

        double max = Math.max(
                Math.max(Math.abs(fl), Math.abs(bl)),
                Math.max(Math.abs(fr), Math.abs(br))
        );

        if (max > 1.0) {
            fl /= max;
            bl /= max;
            fr /= max;
            br /= max;
        }

        frontLeft.setPower(fl);
        backLeft.setPower(bl);
        frontRight.setPower(fr);
        backRight.setPower(br);
    }
}
