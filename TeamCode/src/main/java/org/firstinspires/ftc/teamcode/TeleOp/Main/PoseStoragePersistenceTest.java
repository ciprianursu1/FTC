package org.firstinspires.ftc.teamcode.TeleOp.Main;

import android.content.Context;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "PoseStorage Persistence Test", group = "TEST")
public class PoseStoragePersistenceTest extends LinearOpMode {

    @Override
    public void runOpMode() {

        Context context = hardwareMap.appContext;

        waitForStart();

        while (opModeIsActive()) {

            double[] loaded = PoseStorage.loadPose(context);

            telemetry.addLine("=== PoseStorage PERSISTENCE Test ===");
            telemetry.addLine("Run AFTER Save Test or after Auto");

            telemetry.addData("Loaded X", loaded[0]);
            telemetry.addData("Loaded Y", loaded[1]);
            telemetry.addData("Loaded Heading (deg)", Math.toDegrees(loaded[2]));
            telemetry.addData("Loaded Alliance", (int) loaded[3]);
            telemetry.addData("Loaded Motif", (int) loaded[4]);

            telemetry.addLine("");
            telemetry.addLine("If values match last saved → SUCCESS");

            telemetry.update();
            sleep(200);
        }
    }
}
