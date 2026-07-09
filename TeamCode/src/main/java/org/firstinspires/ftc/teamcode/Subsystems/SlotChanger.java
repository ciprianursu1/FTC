package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;

public class SlotChanger {
    ClosedLoopDC spindexerMotor;
    boolean enabled = true;
    public int slots;
    private int currentSlot = 0;
    private int targetSlot = 0;
    private boolean outtake = false; // if true, add 186 deg offset to all slots
    private boolean forceIntakeAlignment = false;
    double degPerSlot;
    double offset; //186 deg
    ElapsedTime stallTimer = new ElapsedTime();
    double lastPosition = 0;
    boolean stalled = false;
    double stallTimeoutMs;
    double stallMinMovementDeg;
    public SlotChanger(ClosedLoopDC spindexerMotor, int slots, double offset, double stallTimeoutMs, double stallMinMovementDeg){
        this.spindexerMotor = spindexerMotor;
        this.slots = slots;
        this.offset = offset;
        this.stallTimeoutMs = stallTimeoutMs;
        this.stallMinMovementDeg = stallMinMovementDeg;
    }
    public void init(boolean isAuto){
        spindexerMotor.init(isAuto);
        spindexerMotor.setAngleMode(true);
        degPerSlot = 360.0 / slots;
        lastPosition = spindexerMotor.getCurrentPosition();
    }
    public void setSlots(int slots){
        this.slots = slots;
        degPerSlot = 360.0/slots;
    }
    public void enable(boolean enabled){
        this.enabled = enabled;
        if(!enabled) stalled = false;
    }
    public void setOuttake(boolean outtake){
        this.outtake = outtake;
        forceIntakeAlignment = false;
        stalled = false;
        lastPosition = spindexerMotor.getCurrentPosition();
        stallTimer.reset();
    }
    public boolean isBusy(){
        return !spindexerMotor.isOnTarget();
    }
    public void setSlot(int slot){
        currentSlot = slot;
    }
    public int getSlot(){
        return currentSlot + 1;
    }
    public boolean isOuttake() {
        return  outtake;
    }
    public void setOffset(double offset){
        this.offset = offset;
    }
    public void setTargetSlot(int targetSlot){
        this.targetSlot = targetSlot - 1;
        stalled = false;
        lastPosition = spindexerMotor.getCurrentPosition();
        stallTimer.reset();
    }
    public void setForceIntakeAlignment(boolean forceIntakeAlignment){
        this.forceIntakeAlignment = forceIntakeAlignment;
        stalled = false;
        lastPosition = spindexerMotor.getCurrentPosition();
        stallTimer.reset();
    }
    public boolean isStalled(){
        return stalled;
    }
    public void update(){
        if(!enabled) {
            spindexerMotor.enable(false);
            spindexerMotor.update(0);
            return;
        }
        if(stalled) {
            spindexerMotor.enable(false);
            spindexerMotor.update(0);
            return;
        }
        spindexerMotor.enable(true);
        spindexerMotor.update(targetSlot * degPerSlot + (outtake && !forceIntakeAlignment ? offset : 0));
        double position = spindexerMotor.getCurrentPosition();
        if(spindexerMotor.isOnTarget()) {
            currentSlot = targetSlot;
            lastPosition = position;
            stallTimer.reset();
        } else if(Math.abs(position - lastPosition) > stallMinMovementDeg) {
            lastPosition = position;
            stallTimer.reset();
        } else if(stallTimer.milliseconds() > stallTimeoutMs) {
            stalled = true;
            spindexerMotor.enable(false);
            spindexerMotor.update(0);
        }


    }
}
