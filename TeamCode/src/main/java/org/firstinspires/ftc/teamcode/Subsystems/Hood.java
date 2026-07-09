package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.Servo;


public class Hood {
    Servo hood;
    private double angle = 0;
    private double maxAngle;
    private double minAngle;
    private boolean enabled = true;
    final double gearRatio = 127/15.0;
    final double maxTravel = 300.0;
    final double posPerDeg = gearRatio/maxTravel;
    private double angleToPosition(double targetMechanismAngle) {
        double mechanismDegreesFromMin = targetMechanismAngle - minAngle;

        double servoDegreesFromMin = mechanismDegreesFromMin * gearRatio;

        return servoDegreesFromMin / maxTravel;
    }
    public double getMaxAngle() {
        return maxAngle;
    }
    public double getMinAngle() {
        return minAngle;
    }
    public void setMaxAngle(double maxAngle) {
        this.maxAngle = maxAngle;
    }
    public void setMinAngle(double minAngle) {
        this.minAngle = minAngle;
    }
    public Hood(Servo hood, double maxAngle, double minAngle) {
        this.hood = hood;
        this.maxAngle = maxAngle;
        this.minAngle = minAngle;
    }
    public void enable(boolean enabled){
        this.enabled = enabled;
    }
    public boolean isWithinLimits(double angle){
        return angle >= minAngle && angle <= maxAngle;
    }
    public void setAngle(double angle){
        this.angle = angle;
    }
    public void update(){
            if(!enabled) {
                hood.setPosition(0);
                return;
            }
            hood.setPosition(angleToPosition(Math.max(Math.min(angle,maxAngle),minAngle)));
        }
}
