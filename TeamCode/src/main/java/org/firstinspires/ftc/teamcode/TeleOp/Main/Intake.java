package org.firstinspires.ftc.teamcode.TeleOp.Main;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    Motor intake;
    public Intake(HardwareMap hwMap, String intakeName) {
        intake = new Motor(hwMap,intakeName);
    }
    public void init(){
        intake.setRunMode(Motor.RunMode.RawPower);
        intake.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        intake.set(0);
    }
    public void setPower(double power){
        intake.set(power);
    }
}
