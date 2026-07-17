package org.firstinspires.ftc.teamcode.TeleOp.Main;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="MotorTest")
public class MotorTest extends OpMode {
    DcMotorEx flywheel;
    DcMotorEx tureta;
    DcMotor wheel1;
    DcMotor wheel2;

    public void init(){
        flywheel = hardwareMap.get(DcMotorEx.class,"flywheel");
        tureta = hardwareMap.get(DcMotorEx.class,"tureta");
        wheel1 = hardwareMap.get(DcMotor.class,"lf");
        wheel2 = hardwareMap.get(DcMotor.class,"lr");
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tureta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void loop(){
        wheel1.setPower(-gamepad1.left_stick_y);
        wheel2.setPower(-gamepad1.left_stick_y);
        telemetry.addData("Bun Vel",flywheel.getVelocity());
        telemetry.addData("Rau Vel", tureta.getVelocity());
        telemetry.addData("Bun Pos", flywheel.getCurrentPosition());
        telemetry.addData("Rau Pos", tureta.getCurrentPosition());
        telemetry.update();
    }
}
