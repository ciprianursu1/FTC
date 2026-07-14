package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.TeleOp.Main.PIDController;

public class ClosedLoopDC {
    DcMotorEx motor;
    PIDController pid;
    private double maxPower;
    private boolean enabled = true;
    private boolean angleMode = false;
    private final double ticksPerRev;
    private double lastTarget = 0;
    private double lastCurrent = 0;
    private double lastPidTarget = 0;
    private double lastPidCurrent = 0;
    private double lastPidOutput = 0;
    private double lastPower = 0;
    private double lastRawPosition = 0;
    private double lastVelocity = 0;
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
        return pid != null && Math.abs(getTargetError()) < pid.getDeadband();
    }
    public double getTargetError(){
        double current = getCurrentPosition();
        if(angleMode) return wrapAngle(lastTarget - current);
        return lastTarget - current;
    }
    public double getPositionTolerance(){
        if(pid == null) return 0;
        return pid.getDeadband();
    }
    public void setAngleMode(boolean angleMode) {
        if (this.angleMode != angleMode) {
            this.angleMode = angleMode;
            reset();
        }
    }
    public void update(double target){
        lastTarget = target;
        lastRawPosition = motor.getCurrentPosition();
        lastVelocity = motor.getVelocity();
        if(!enabled || pid == null){
            motor.setPower(0);
            lastCurrent = angleMode ? lastRawPosition * 360.0 / ticksPerRev : lastRawPosition;
            lastPidTarget = target;
            lastPidCurrent = lastCurrent;
            lastPidOutput = 0;
            lastPower = 0;
            reset();
            return;
        }

        double currentPos = lastRawPosition;
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
        lastPidOutput = power;
        power = Math.max(Math.min(power, maxPower), -maxPower);
        lastCurrent = pidCurrent;
        lastPidTarget = pidTarget;
        lastPidCurrent = pidCurrent;
        lastPower = power;
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
    public void appendTelemetry(Telemetry telemetry, String name) {
        telemetry.addLine("  " + name);
        telemetry.addData(name + " Enabled", enabled);
        telemetry.addData(name + " Angle Mode", angleMode);
        telemetry.addData(name + " Max Power", "%.3f", maxPower);
        telemetry.addData(name + " Raw Position", "%.0f ticks", lastRawPosition);
        telemetry.addData(name + " Position", "%.2f", getCurrentPosition());
        telemetry.addData(name + " Velocity", "%.2f ticks/s", lastVelocity);
        telemetry.addData(name + " Current", "%.2f A", motor.getCurrent(CurrentUnit.AMPS));
        telemetry.addData(name + " Over Current", motor.isOverCurrent());
        telemetry.addData(name + " Target", "%.2f", lastTarget);
        telemetry.addData(name + " PID Target", "%.2f", lastPidTarget);
        telemetry.addData(name + " PID Current", "%.2f", lastPidCurrent);
        telemetry.addData(name + " Error", "%.2f", lastPidTarget - lastPidCurrent);
        telemetry.addData(name + " PID Output", "%.3f", lastPidOutput);
        telemetry.addData(name + " Motor Power", "%.3f", lastPower);
        telemetry.addData(name + " On Target", isOnTarget());
        if(pid != null) {
            telemetry.addData(name + " PIDF", "P %.4f I %.4f D %.4f F %.4f", pid.getkP(), pid.getkI(), pid.getkD(), pid.getkF());
            telemetry.addData(name + " PID State", "err %.2f sum %.3f out %.3f", pid.getLastError(), pid.getErrorSum(), pid.getOutput());
            telemetry.addData(name + " PID Limits", "deadband %.2f integral %.2f..%.2f", pid.getDeadband(), pid.getIntegralMin(), pid.getIntegralMax());
        } else {
            telemetry.addData(name + " PID", "null");
        }
    }
}
