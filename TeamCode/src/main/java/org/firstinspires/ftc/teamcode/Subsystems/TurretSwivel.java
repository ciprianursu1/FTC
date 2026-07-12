package org.firstinspires.ftc.teamcode.Subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TurretSwivel {
    ClosedLoopDC turret;
    boolean enabled = true;
    double targetAngle;
    double currentAngle;
    double limitLeft; // positive
    double limitRight;// negative
    double startAngle;
    double lastCommandedAngle;
    double lastMotorTargetAngle;
    private double wrapAngle(double angle) {
        angle = (angle + 180) % 360;
        if (angle < 0) {
            angle += 360;
        }
        return angle - 180;
    }
    private double clampAngle(double angle){
        if(angle > limitLeft || angle < limitRight) {
            if(Math.abs(currentAngle - limitRight) > Math.abs(currentAngle - limitLeft)) angle = limitLeft;
            else angle = limitRight;
        }
        return angle;
    }
    public TurretSwivel(ClosedLoopDC turret, double limitLeft, double limitRight, double startAngle){
        this.turret = turret;
        this.limitLeft = limitLeft;
        this.limitRight = limitRight;
        this.startAngle = startAngle;
    }
    public void setLimitLeft(double limitLeft){
        this.limitLeft = limitLeft;
    }
    public void setLimitRight(double limitRight){
        this.limitRight = limitRight;
    }
    public void setStartAngle(double startAngle){
        this.startAngle = startAngle;
    }
    public double getLimitLeft(){
        return limitLeft;
    }
    public double getLimitRight(){
        return limitRight;
    }
    public double getStartAngle(){
        return startAngle;
    }
    public double getCurrentAngle(){
        currentAngle = wrapAngle(turret.getCurrentPosition() + startAngle);
        return currentAngle;
    }

    public void init(boolean isAuto){
        turret.init(isAuto);
        turret.setAngleMode(true);
    }
    public void enable(boolean enabled){
        this.enabled = enabled;
    }
    public void update(){
        currentAngle = getCurrentAngle();
        turret.enable(enabled);
        lastCommandedAngle = clampAngle(targetAngle);
        lastMotorTargetAngle = wrapAngle(lastCommandedAngle - startAngle);
        turret.update(lastMotorTargetAngle);
    }
    public void setTargetAngle(double angle){
        targetAngle = wrapAngle(angle);
    }
    public boolean isOnTarget(){
        return turret.isOnTarget();
    }
    public void appendTelemetry(Telemetry telemetry) {
        telemetry.addLine("--- Turret Swivel ---");
        telemetry.addData("Turret Enabled", enabled);
        telemetry.addData("Turret Current", "%.2f deg", currentAngle);
        telemetry.addData("Turret Target", "%.2f deg", targetAngle);
        telemetry.addData("Turret Commanded", "%.2f deg", lastCommandedAngle);
        telemetry.addData("Turret Motor Target", "%.2f deg", lastMotorTargetAngle);
        telemetry.addData("Turret Error", "%.2f deg", wrapAngle(lastCommandedAngle - currentAngle));
        telemetry.addData("Turret On Target", isOnTarget());
        telemetry.addData("Turret Limits L/R", "%.2f / %.2f deg", limitLeft, limitRight);
        telemetry.addData("Turret Start Angle", "%.2f deg", startAngle);
        turret.appendTelemetry(telemetry, "Turret Motor");
    }

}
