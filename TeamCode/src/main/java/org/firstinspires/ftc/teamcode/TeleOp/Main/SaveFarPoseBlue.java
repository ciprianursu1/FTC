package org.firstinspires.ftc.teamcode.TeleOp.Main;


import android.content.Context;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "SaveFarPoseBlue", group = "TEST")
public class SaveFarPoseBlue extends OpMode {
    Context context = hardwareMap.appContext;

    double x = 56;
    double y = 8;
    double heading = Math.toRadians(90);
    boolean alliance = true; // re
    int motif = 21;
    public void init(){
        PoseStorage.savePose(
                context,
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
