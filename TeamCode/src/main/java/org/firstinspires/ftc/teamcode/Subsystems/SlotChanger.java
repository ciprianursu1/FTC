package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SlotChanger {
    ClosedLoopDC spindexerMotor;
    boolean enabled = true;
    public int slots;
    private int currentSlot = 0;
    private int targetSlot = 0;
    private boolean outtake = false; // if true, add 186 deg offset to all slots
    private boolean forceIntakeAlignment = false;
    private boolean customTargetAngleEnabled = false;
    private double customTargetAngle = 0;
    double degPerSlot;
    double slotZeroAngle = 0;
    double offset; //186 deg
    ElapsedTime stallTimer = new ElapsedTime();
    double lastPosition = 0;
    boolean stalled = false;
    double stallTimeoutMs;
    double stallMinMovementDeg;
    private double wrapAngle(double angle) {
        angle = angle % 360;
        if(angle < 0) angle += 360;
        return angle;
    }
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
        currentSlot = (int) Math.round(wrapAngle(spindexerMotor.getCurrentPosition()) / degPerSlot) % slots;
        targetSlot = currentSlot;
        slotZeroAngle = spindexerMotor.getCurrentPosition() - currentSlot * degPerSlot;
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
        targetSlot = slot;
        slotZeroAngle = spindexerMotor.getCurrentPosition() - currentSlot * degPerSlot;
    }
    public int getSlot(){
        return currentSlot + 1;
    }
    public boolean isOuttake() {
        return  outtake;
    }
    public boolean isIntakeAligned(){
        return !outtake && !isBusy();
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
    public void clearStall(){
        stalled = false;
        lastPosition = spindexerMotor.getCurrentPosition();
        stallTimer.reset();
    }
    public void setCustomTargetAngle(boolean enabled, double targetAngle){
        if(enabled != customTargetAngleEnabled || targetAngle != customTargetAngle) {
            stalled = false;
            lastPosition = spindexerMotor.getCurrentPosition();
            stallTimer.reset();
        }
        customTargetAngleEnabled = enabled;
        customTargetAngle = targetAngle;
    }
    public boolean isCustomTargetAngleEnabled(){
        return customTargetAngleEnabled;
    }
    private double getTargetAngle(){
        if(customTargetAngleEnabled) return customTargetAngle;
        return slotZeroAngle + targetSlot * degPerSlot + (outtake && !forceIntakeAlignment ? offset : 0);
    }
    public double getTargetAngleForTelemetry(){
        return getTargetAngle();
    }
    public void update(){
        if(!enabled || stalled) {
            spindexerMotor.enable(false);
            spindexerMotor.update(getTargetAngle());
            return;
        }
        spindexerMotor.enable(true);
        spindexerMotor.update(getTargetAngle());
        double position = spindexerMotor.getCurrentPosition();
        if(spindexerMotor.isOnTarget()) {
            stalled = false;
            if(!customTargetAngleEnabled) currentSlot = targetSlot;
            lastPosition = position;
            stallTimer.reset();
        } else if(Math.abs(position - lastPosition) > stallMinMovementDeg) {
            lastPosition = position;
            stallTimer.reset();
        } else if(stallTimer.milliseconds() > stallTimeoutMs) {
            stalled = true;
            spindexerMotor.enable(false);
            spindexerMotor.update(getTargetAngle());
        }


    }
    public void appendTelemetry(Telemetry telemetry) {
        telemetry.addLine("--- Slot Changer ---");
        telemetry.addData("Slot Enabled", enabled);
        telemetry.addData("Slot Current/Target", "%d / %d", currentSlot + 1, targetSlot + 1);
        telemetry.addData("Slot Count", slots);
        telemetry.addData("Slot Busy", isBusy());
        telemetry.addData("Slot Outtake", outtake);
        telemetry.addData("Slot Force Intake Align", forceIntakeAlignment);
        telemetry.addData("Slot Custom Target Enabled", customTargetAngleEnabled);
        telemetry.addData("Slot Custom Target Angle", "%.2f deg", customTargetAngle);
        telemetry.addData("Slot Stalled", stalled);
        telemetry.addData("Slot Target Angle", "%.2f deg", getTargetAngle());
        telemetry.addData("Slot Target Error", "%.2f deg", spindexerMotor.getTargetError());
        telemetry.addData("Slot Zero Angle", "%.2f deg", slotZeroAngle);
        telemetry.addData("Slot Deg Per Slot", "%.2f deg", degPerSlot);
        telemetry.addData("Slot Outtake Offset", "%.2f deg", offset);
        telemetry.addData("Slot Last Movement Position", "%.2f deg", lastPosition);
        telemetry.addData("Slot Stall Timer", "%.0f / %.0f ms", stallTimer.milliseconds(), stallTimeoutMs);
        telemetry.addData("Slot Stall Min Movement", "%.2f deg", stallMinMovementDeg);
        spindexerMotor.appendTelemetry(telemetry, "Spindexer Motor");
    }
}
