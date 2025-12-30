package org.firstinspires.ftc.teamcode.TeleOp.Main;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Poses {

    // Motoare unrelated de sasiu
    Servo ejector ;//servo ul care baga mingile in flywheel
    DcMotor matura;//intake ul activ, (ala pasiv unde e? ~tibichi)
    DcMotor flywheel;//cel care lanseaza mingea
    DcMotor spinner;//cel care roteste mingile in rezervor

    LinearOpMode opMode;

    public Poses(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    void servoInit() {
        ejector = opMode.hardwareMap.servo.get("ejector");
    }

    void DcInit() {
        matura = opMode.hardwareMap.dcMotor.get("matura");

        flywheel = opMode.hardwareMap.dcMotor.get("flywheel");
        flywheel.setPower(0.3);

        spinner = opMode.hardwareMap.dcMotor.get("spinner");
        spinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spinner.setTargetPosition(0);
        spinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spinner.setPower(0);
    }

    public void pozitii() {
        //intake
        if (opMode.gamepad2.circleWasPressed()) {
            matura.setPower(1);
            flywheel.setPower(0.3);
        }

        //outtake
        if (opMode.gamepad2.xWasPressed()) {
            flywheel.setPower(1);
            matura.setPower(0);
            ejector.setPosition(0.555);
            //sa se miste 120 de grade spinner ul
        }
    }
}
