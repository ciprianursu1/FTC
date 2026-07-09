package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class Flywheel {
    DcMotorEx motor;
    private boolean enabled = true;
    private double targetRPM;
    private final double ticksPerRev;
    private double rpmDeadband = 50;
    private double radius;
    private double disableRPM;
    private double efficiency;
    public Flywheel(DcMotorEx motor,double ticksPerRev,double rpmDeadband,double radius,double efficiency,double disableRPM, PIDFCoefficients pid) {
        this.motor = motor;
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pid);
        this.ticksPerRev = ticksPerRev;
        this.rpmDeadband = rpmDeadband;
        this.radius = radius;
        this.efficiency = efficiency;
        this.disableRPM = disableRPM;
    }
    public void setDisableRPM(double disableRPM){
        this.disableRPM = disableRPM;
    }
    public void setRPMDeadband(double rpmDeadband){
        this.rpmDeadband = rpmDeadband;
    }
    public void setRadius(double radius){
        this.radius = radius;
    }
    public void setEfficiency(double efficiency){
        this.efficiency = efficiency;
    }
    public double getRadius(){
        return radius;
    }
    public double getEfficiency(){
        return efficiency;
    }
    public boolean isOnTarget(){
        double currentRPM = (motor.getVelocity() * 60.0) / ticksPerRev;
        return Math.abs(currentRPM - targetRPM) < rpmDeadband;
    }
    public void init(){
        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
    }
    public void enable(boolean enabled){
        this.enabled = enabled;
    }
    public void setTargetRPM(double targetRPM){
        this.targetRPM = targetRPM;
    }
    public void update(){
        if(!enabled) {
            motor.setVelocity(disableRPM*ticksPerRev/60.0);
            return;
        }
        motor.setVelocity(targetRPM*ticksPerRev/60.0);
    }

    public double getRPM() {
        return (motor.getVelocity() * 60.0) / ticksPerRev;
    }
}
