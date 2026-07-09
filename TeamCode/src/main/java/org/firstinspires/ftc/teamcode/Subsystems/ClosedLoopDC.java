package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.TeleOp.Main.PIDController;

public class ClosedLoopDC {
    DcMotorEx motor;
    PIDController pid;
    private double maxPower;
    private boolean enabled = true;
    private boolean angleMode = false;
    private final double ticksPerRev;
    private double wrapAngle(double angle) {
        angle = (angle + 180) % 360;
        if (angle < 0) {
            angle += 360;
        }
        return angle - 180;
    }
    public ClosedLoopDC(DcMotorEx motor, PIDController pid, double maxPower, double ticksPerRev) {
        this.motor = motor;
        this.pid = pid;
        this.maxPower = maxPower;
        this.ticksPerRev = ticksPerRev;
    }
    public void init(boolean isAuto){
        if(isAuto) motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }
    public void setMaxPower(double maxPower) {
        this.maxPower = maxPower;
    }
    public double getCurrentPosition(){
        if(angleMode){
            return motor.getCurrentPosition() * 360.0 / ticksPerRev;
        }else{
            return motor.getCurrentPosition();
        }
    }
    public boolean isOnTarget(){
        return pid.isOnTarget();
    }
    public void setAngleMode(boolean angleMode) {
        if (this.angleMode != angleMode) {
            this.angleMode = angleMode;
            reset();
        }
    }
    public void update(double target){
        if(!enabled || pid == null){
            motor.setPower(0);
            reset();
            return;
        }

        double currentPos = motor.getCurrentPosition();
        double pidTarget;
        double pidCurrent;

        if (angleMode) {
            double currentAngle = currentPos * 360.0 / ticksPerRev;
            pidTarget = currentAngle + wrapAngle(target - currentAngle);
            pidCurrent = currentAngle;
        } else {
            pidTarget = target;
            pidCurrent = currentPos;
        }

        double power = pid.update(pidTarget, pidCurrent);
        power = Math.max(Math.min(power, maxPower), -maxPower);
        motor.setPower(power);
    }
    public void enable(boolean enabled){
        this.enabled = enabled;
    }
    public boolean isEnabled(){
        return enabled;
    }
    public void reset(){
        if(pid != null) pid.reset();
    }
    public void setPIDController(PIDController pid){
        this.pid = pid;
    }
}