package org.firstinspires.ftc.teamcode.TeleOp.Main;

import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.localizerConstants;

import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name="Pedro + Limelight Test", group="TEST")
public class LimelightPosTest extends LinearOpMode {

    Limelight3A limelight;
    PinpointLocalizer pinpoint;
    Pose startPose;
    Pose currentPose;
    boolean limelightCorrectionMode = true;

    static final double INCH_PER_METER = 39.3701;
    static final double LL_OFFSET_X = -65.5/1000.0; // meters
    static final double LL_OFFSET_Y = 181/1000.0;   // meters
    static final double turretOffsetX = 0.0;
    static final double turretOffsetY = 52/1000.0;

    @Override
    public void runOpMode() throws InterruptedException {

        limelight = hardwareMap.get(Limelight3A.class,"limelight");
        pinpoint = new PinpointLocalizer(hardwareMap, localizerConstants); // use real constants if available
        startPose = new Pose(64.3, 15.74/2.0, Math.toRadians(90));
        pinpoint.setStartPose(startPose);

        limelight.pipelineSwitch(5);
        limelight.start();

        waitForStart();

        while(opModeIsActive()) {

            // Update Pedro pose
            pinpoint.update();
            currentPose = pinpoint.getPose();
            limelight.updateRobotOrientation(Math.toDegrees(currentPose.getHeading()));
            // Limelight-based correction
            if(limelightCorrectionMode) {
                LLResult result = limelight.getLatestResult();
                if(result != null && result.getBotpose_MT2() != null) {
                    Pose3D LLPose = result.getBotpose_MT2();

                    Pose pedroPose = PoseConverter.pose2DToPose(
                            new Pose2D(
                                    DistanceUnit.INCH,
                                    LLPose.getPosition().x * INCH_PER_METER,
                                    LLPose.getPosition().y * INCH_PER_METER,
                                    AngleUnit.DEGREES,
                                    LLPose.getOrientation().getYaw()
                            ),
                            InvertedFTCCoordinates.INSTANCE
                    );

                    pedroPose = processPedroPose(pedroPose);
                    currentPose = pedroPose;
                }
            }

            // Telemetry
            telemetry.addLine("=== Pedro Limelight Test ===");
            if(currentPose != null) {
                telemetry.addData("Pedro X (in)", currentPose.getX());
                telemetry.addData("Pedro Y (in)", currentPose.getY());
                telemetry.addData("Pedro Heading (deg)", Math.toDegrees(currentPose.getHeading()));
            } else {
                telemetry.addLine("Pedro Pose: NULL");
            }
            telemetry.addData("pinpoint heading",pinpoint.getPose().getHeading());
            telemetry.update();

            sleep(50);
        }
    }

    // ---------------- Helpers ----------------
    private Pose processPedroPose(Pose pedroPose) {
        double x = pedroPose.getX();
        double y = pedroPose.getY();
        double heading = pedroPose.getHeading();

        // Apply Limelight offsets
        heading = normalizeAngle(heading); // No turret for this test
        x += (Math.cos(heading) * LL_OFFSET_X - Math.sin(heading) * LL_OFFSET_Y) * INCH_PER_METER;
        y += (Math.sin(heading) * LL_OFFSET_X + Math.cos(heading) * LL_OFFSET_Y) * INCH_PER_METER;
        x -= (turretOffsetX * Math.cos(heading) - turretOffsetY * Math.sin(heading))*INCH_PER_METER;
        y -= (turretOffsetX * Math.sin(heading) + turretOffsetY * Math.cos(heading))*INCH_PER_METER;
        pedroPose = new Pose(x, y, heading);
        return pedroPose;
    }

    private double normalizeAngle(double angle) {
        while(angle > 180) angle -= 360;
        while(angle < -180) angle += 360;
        return angle;
    }
}
