package org.firstinspires.ftc.teamcode.TeleOp.Main;

import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "TeleOPBlueClose")
public class TeleOpBC extends LinearOpMode {
    Spindexer spinner;
    Turret tureta;
    Intake intake;
    final private Pose startPose = new Pose(64.0,8.0,90);
    PinpointLocalizer pinpoint;
    public void runOpMode(){
        spinner = new Spindexer(hardwareMap,
                "spinnerClose","spinnerFar",
                "ejector",
                "Color1", "Color2","Color3"
        );
        pinpoint = new PinpointLocalizer(hardwareMap, Constants.localizerConstants);
        pinpoint.setStartPose(startPose);
        tureta = new Turret(hardwareMap,"flywheel","tureta","unghituretaoy",pinpoint);
        intake = new Intake(hardwareMap,"intake");
        intake.init();
        spinner.init();
        tureta.init(0.0,144.0);
        while (opModeIsActive()){
            spinner.update();
            tureta.update();
        }
    }
}
