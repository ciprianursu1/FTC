package org.firstinspires.ftc.teamcode.Subsystems;

public class TurretSwivel {
    ClosedLoopDC turret;
    boolean enabled = true;
    double targetAngle;
    double currentAngle;
    double limitLeft; // positive
    double limitRight;// negative
    double startAngle;
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
        currentAngle = turret.getCurrentPosition() + startAngle;
        if(!enabled) {
            turret.enable(false);
            return;
        }
        turret.update(clampAngle(targetAngle));
    }
    public void setTargetAngle(double angle){
        targetAngle = wrapAngle(angle);
    }
    public boolean isOnTarget(){
        return turret.isOnTarget();
    }

}
