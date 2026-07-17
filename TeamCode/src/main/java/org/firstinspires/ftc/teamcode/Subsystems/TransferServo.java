package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TransferServo {
    Servo transferServo;
    private final double posUp;
    private final double posDown;
    private boolean goUp; //true = up, false = down
    private boolean enabled = true;
    private double posDisable;
    private double downAdjustment = 0;
    public TransferServo(Servo transferServo, double posUp, double posDown, double posDisable) {
        this.transferServo = transferServo;
        this.posUp = posUp;
        this.posDown = posDown;
        this.posDisable = posDisable;
    }
    public void setPosDisable(double posDisable){
        this.posDisable = posDisable;
    }
    public boolean addToDownPosition(double amount){
        double lastAdjustment = downAdjustment;
        downAdjustment = Math.max(0, Math.min(1 - posDown, downAdjustment + amount));
        return downAdjustment != lastAdjustment;
    }
    public void resetDownPosition(){
        downAdjustment = 0;
    }
    public double getDownPosition(){
        return posDown + downAdjustment;
    }
    public void goUp(){
        goUp = true;
    }
    public void goDown(){
        goUp = false;
    }
    public boolean isGoingUp(){
        return goUp;
    }
    public void enable(boolean enabled){
        this.enabled = enabled;
    }
    public void update(){
        if(enabled) {
            if (goUp) transferServo.setPosition(posUp);
            else transferServo.setPosition(getDownPosition());
        } else {
            transferServo.setPosition(posDisable);
        }
    }
    public void appendTelemetry(Telemetry telemetry) {
        telemetry.addLine("--- Transfer Servo ---");
        telemetry.addData("Transfer Enabled", enabled);
        telemetry.addData("Transfer State", goUp ? "UP" : "DOWN");
        telemetry.addData("Transfer Position", "%.3f", transferServo.getPosition());
        telemetry.addData("Transfer Up/Down/Disabled", "%.3f / %.3f / %.3f", posUp, getDownPosition(), posDisable);
        telemetry.addData("Transfer Down Adjustment", "%.3f", downAdjustment);
    }
}
