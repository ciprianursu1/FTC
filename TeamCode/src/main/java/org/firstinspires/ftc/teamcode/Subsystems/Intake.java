package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

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
    public void appendTelemetry(Telemetry telemetry) {
        telemetry.addLine("--- Intake ---");
        telemetry.addData("Intake Enabled", enabled);
        telemetry.addData("Intake Reverse", reverse);
        telemetry.addData("Intake Power On", "%.3f", powerOn);
        telemetry.addData("Intake Motor Power", "%.3f", intake.getPower());
        telemetry.addData("Intake Encoder", "%d ticks", intake.getCurrentPosition());
    }
}
