package org.firstinspires.ftc.teamcode.TeleOp.Main;

import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.localizerConstants;

import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "Limelight MT2 Accuracy Test", group = "TEST")
public class LimelightPosTest extends LinearOpMode {

    // ---------------- HARDWARE ----------------
    private Limelight3A limelight;
    private PinpointLocalizer pinpoint;

    // ---------------- POSES ----------------
    private Pose visionPose;
    private Pose pinpointPose;
    IMU imu;

    // ---------------- CONSTANTS ----------------
    private static final double INCH_PER_METER = 39.3701;

    // Limelight position relative to robot center (meters)
    private static final double LL_OFFSET_X = -65.5 / 1000.0; // forward +
    private static final double LL_OFFSET_Y = 181.0 / 1000.0; // left +

    @Override
    public void runOpMode() throws InterruptedException {

        // ---------- INIT ----------
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        pinpoint = new PinpointLocalizer(hardwareMap, localizerConstants);

        limelight.pipelineSwitch(5); // Blue AprilTags
        limelight.start();
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.UP)));
        imu.resetYaw();
        telemetry.addLine("Limelight MT2 Test Ready");
        telemetry.update();

        waitForStart();

        // ---------- LOOP ----------
        while (opModeIsActive()) {

            // Update odometry (for comparison only)
            pinpoint.update();
            pinpointPose = pinpoint.getPose();
            YawPitchRollAngles ypr = imu.getRobotYawPitchRollAngles();
            // Read Limelight
            LLResult result = limelight.getLatestResult();
            limelight.updateRobotOrientation(Math.toDegrees(pinpointPose.getHeading() + Math.PI/2));

            if (result != null && result.isValid() && !result.getFiducialResults().isEmpty()) {
                Pose3D llPose3D = result.getBotpose_MT2();

                if (llPose3D != null) {
                    visionPose = new Pose(
                            llPose3D.getPosition().x*INCH_PER_METER + 72,
                            llPose3D.getPosition().y*INCH_PER_METER + 72,
                            llPose3D.getOrientation().getYaw(AngleUnit.RADIANS) - Math.PI/2.0
                    );
                }
            }

            // ---------- TELEMETRY ----------
            telemetry.addLine("=== LIMELIGHT MT2 ===");
            if (visionPose != null) {
                telemetry.addData("Vision X (in)", visionPose.getX());
                telemetry.addData("Vision Y (in)", visionPose.getY());
                telemetry.addData("Vision Heading (deg)",
                        Math.toDegrees(visionPose.getHeading()));
            } else {
                telemetry.addLine("Vision Pose: INVALID");
            }

            telemetry.addLine("=== PINPOINT ===");
            telemetry.addData("Pinpoint X (in)", pinpointPose.getX());
            telemetry.addData("Pinpoint Y (in)", pinpointPose.getY());
            telemetry.addData("Pinpoint Heading (deg)",
                    Math.toDegrees(pinpointPose.getHeading()));
            telemetry.addData("IMU Heading", Math.toDegrees(ypr.getYaw(AngleUnit.RADIANS) + Math.PI/2.0));
            telemetry.update();

            sleep(50);
        }
    }
}
