package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Flywheel {
    DcMotorEx motor;
    private boolean enabled = true;
    private double targetRPM;
    private final double ticksPerRev;
    private double rpmDeadband = 50;
    private double radius;
    private double disableRPM;
    private double efficiency;
    private final PIDFCoefficients pid;
    public Flywheel(DcMotorEx motor,double ticksPerRev,double rpmDeadband,double radius,double efficiency,double disableRPM, PIDFCoefficients pid) {
        this.motor = motor;
        this.pid = pid;
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
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pid);
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
    public void appendTelemetry(Telemetry telemetry) {
        double currentRPM = getRPM();
        telemetry.addLine("--- Flywheel ---");
        telemetry.addData("Flywheel Enabled", enabled);
        telemetry.addData("Flywheel RPM", "%.1f / %.1f", currentRPM, targetRPM);
        telemetry.addData("Flywheel Error", "%.1f RPM", targetRPM - currentRPM);
        telemetry.addData("Flywheel Deadband", "%.1f RPM", rpmDeadband);
        telemetry.addData("Flywheel On Target", isOnTarget());
        telemetry.addData("Flywheel Disable RPM", "%.1f", disableRPM);
        telemetry.addData("Flywheel Velocity", "%.1f ticks/s", motor.getVelocity());
        telemetry.addData("Flywheel Current", "%.2f A", motor.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Flywheel Over Current", motor.isOverCurrent());
        telemetry.addData("Flywheel Target Velocity", "%.1f ticks/s", targetRPM * ticksPerRev / 60.0);
        telemetry.addData("Flywheel Encoder", "%d ticks", motor.getCurrentPosition());
        telemetry.addData("Flywheel PIDF", "P %.3f I %.3f D %.3f F %.3f", pid.p, pid.i, pid.d, pid.f);
        telemetry.addData("Flywheel Radius/Efficiency", "%.4f m / %.3f", radius, efficiency);
    }
}
