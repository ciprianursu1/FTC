package org.firstinspires.ftc.teamcode.TeleOp.Main;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "TurretLimitTest", group = "Alpha")
public class TurretLimitTest extends OpMode {
    DcMotorEx turret;
    final double startTurretAngle = -180;
    private static final double MOTOR_TICKS_PER_REV = 384.5;
    private static final double MOTOR_TO_TURRET_RATIO = 75.0 / 26.0;
    private static final double TURRET_DEG_PER_TICK =
            360.0 / (MOTOR_TICKS_PER_REV * MOTOR_TO_TURRET_RATIO);
    public void init(){
        turret = hardwareMap.get(DcMotorEx.class,"tureta");
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void loop(){
        telemetry.addData("TurretAngle",normalizeAngle(turret.getCurrentPosition()*TURRET_DEG_PER_TICK + startTurretAngle));
    }
    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }


}
