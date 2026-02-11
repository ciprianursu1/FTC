package org.firstinspires.ftc.teamcode.TeleOp.Main;

import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "OdometryTestTeleOp")
public class testodo extends LinearOpMode {

    // Drivetrain
    DcMotor frontLeft, frontRight, backLeft, backRight;

    // Localization
    PinpointLocalizer pinpoint;
    Pose pose;
    Pose startPose = new Pose(0, 0, Math.toRadians(90)); // change if needed

    @Override
    public void runOpMode() {

        initDrive();
        initLocalization();

        waitForStart();

        while (opModeIsActive()) {

            drive();
            pinpoint.update();
            pose = pinpoint.getPose();

            // Reset odometry if needed
            if (gamepad1.options && gamepad1.share) {
                pinpoint.setPose(startPose);
            }

            telemetry.addData("X", "%.2f", pose.getX());
            telemetry.addData("Y", "%.2f", pose.getY());
            telemetry.addData("Heading (deg)", "%.1f",
                    Math.toDegrees(pose.getHeading()));
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

    private void initLocalization() {
        pinpoint = new PinpointLocalizer(hardwareMap, Constants.localizerConstants);
        pinpoint.setStartPose(startPose);
    }

    private void drive() {
        double y  = -gamepad1.left_stick_y; // forward
        double x  =  gamepad1.left_stick_x; // strafe
        double rx =  gamepad1.right_stick_x; // rotate

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
