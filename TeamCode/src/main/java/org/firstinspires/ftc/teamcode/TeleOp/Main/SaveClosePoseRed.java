package org.firstinspires.ftc.teamcode.TeleOp.Main;


import android.content.Context;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "SaveClosePoseRed", group = "TEST")
public class SaveClosePoseRed extends OpMode {

    double x = 88;
    double y = 8;
    double heading = Math.toRadians(90);
    boolean alliance = false;
    int motif = 21;
    public void init(){
        telemetry.addLine("Saving pose");
        PoseStorage.savePose(
                hardwareMap.appContext,
                x,
                y,
                heading,
                alliance,
                motif);

    }
    public void loop(){
        requestOpModeStop();
    }
}
