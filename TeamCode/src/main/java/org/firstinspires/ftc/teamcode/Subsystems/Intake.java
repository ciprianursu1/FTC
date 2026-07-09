package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Intake {
    DcMotor intake;
    double powerOn = 1.0;
    boolean enabled = true;
    boolean reverse = false;
    public Intake(DcMotor intake){
        this.intake = intake;
    }
    public void update(){
        if(!enabled) {
            intake.setPower(0);
            return;
        }
        intake.setPower(reverse ? -powerOn : powerOn);
    }
    public void setPowerOn(double powerOn){
        this.powerOn = powerOn;
    }
    public void enable(boolean enabled){
        this.enabled = enabled;
    }
    public void setReverse(boolean reverse){
        this.reverse = reverse;
    }
    public boolean isEnabled(){
        return enabled;
    }
}
