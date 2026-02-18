package org.firstinspires.ftc.teamcode.TeleOp.Main;

import android.content.Context;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "PoseStorage Save Test", group = "TEST")
public class PoseStorageSaveTest extends LinearOpMode {

    @Override
    public void runOpMode() {

        Context context = hardwareMap.appContext;

        double x = 72.0;
        double y = 36.0;
        double heading = Math.toRadians(90);
        boolean alliance = true; // red
        int motif = 5;

        waitForStart();

        while (opModeIsActive()) {

            // Press A to save
            if (gamepad1.a) {
                PoseStorage.savePose(
                        context,
                        x,
                        y,
                        heading,
                        alliance,
                        motif
                );
            }

            // Press B to change values (to prove persistence)
            if (gamepad1.b) {
                x += 5;
                y += 3;
                heading += Math.toRadians(10);
                motif++;
            }

            double[] loaded = PoseStorage.loadPose(context);

            telemetry.addLine("=== PoseStorage SAVE Test ===");
            telemetry.addLine("Press A = Save current pose");
            telemetry.addLine("Press B = Modify pose values");

            telemetry.addData("Saved X", x);
            telemetry.addData("Saved Y", y);
            telemetry.addData("Saved Heading (deg)", Math.toDegrees(heading));
            telemetry.addData("Saved Alliance (1=Red)", alliance ? 1 : 0);
            telemetry.addData("Saved Motif", motif);

            telemetry.addLine("--- Loaded From Storage ---");
            telemetry.addData("Loaded X", loaded[0]);
            telemetry.addData("Loaded Y", loaded[1]);
            telemetry.addData("Loaded Heading (deg)", Math.toDegrees(loaded[2]));
            telemetry.addData("Loaded Alliance", (int) loaded[3]);
            telemetry.addData("Loaded Motif", (int) loaded[4]);

            telemetry.update();
            sleep(100);
        }
    }
}
