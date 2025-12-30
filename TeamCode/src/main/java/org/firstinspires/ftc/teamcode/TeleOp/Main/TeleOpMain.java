package org.firstinspires.ftc.teamcode.TeleOp.Main;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@TeleOp(name="TeleOpServosAndPivoter")
public class TeleOpMain extends LinearOpMode {

    Robot robot;

    @Override
    public void runOpMode() {
        robot = new Robot(this);
        robot.init();

        waitForStart();

        new Thread(() -> {
            while (opModeIsActive()) robot.drive.loop();
        }).start();

        new Thread(() -> {
            while (opModeIsActive()) robot.poses.pozitii();
        }).start();

        new Thread(() -> {
            while (opModeIsActive()) robot.vision.aliniereTureta();
        }).start();

        while (opModeIsActive()) idle();
    }
}
