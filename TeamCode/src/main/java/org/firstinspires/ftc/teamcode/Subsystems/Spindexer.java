package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Arrays;

public class Spindexer {
    SlotChanger slotChanger;
    ColorTuning tuneV3;
    final ColorSensor colorSensor;
    final ColorSensor[] verificationColorSensors;
    final int[] verificationSensorToSlot;
    int[] sensedSlotColor;
    int sampleIndex = 0;
    int sampleSize = 5;
    int[] samples = new int[sampleSize];
    int[] slotColor;
    boolean enabled = true;
    ElapsedTime sampleTimer = new ElapsedTime();
    int samplingDelay = 80;
    public Spindexer(SlotChanger slotChanger, ColorSensor colorSensor, ColorSensor[] verificationColorSensors, int[] verificationSensorToSlot, ColorTuning tuneV3){
        this.slotChanger = slotChanger;
        this.colorSensor = colorSensor;
        this.verificationColorSensors = verificationColorSensors;
        this.verificationSensorToSlot = verificationSensorToSlot;
        slotColor = new int[slotChanger.slots];
        sensedSlotColor = new int[slotChanger.slots];
        this.tuneV3 = tuneV3;
    }
    public void init(boolean isAuto){
        slotChanger.init(isAuto);
        for (ColorSensor verificationColorSensor : verificationColorSensors) {
            verificationColorSensor.setGain(RobotConfig.SECONDARY_SENSOR_GAIN);
        }
        colorSensor.setGain(RobotConfig.MAIN_SENSOR_GAIN);
    }
    public boolean isFull() {
        for (int color : slotColor) {
            if (color == 0) {
                return false;
            }
        }
        return true;
    }
    public void requestOuttake(boolean outtake) {
        slotChanger.setOuttake(outtake);
    }
    private int classifySlot(ColorSensor sensor){
        float[] hsv = sensor.getHSV();
        if(sensor.getAlpha() < tuneV3.minAlpha) return 0;
        if(hsv[1] < tuneV3.minSaturation) return 0;
        if(hsv[0] > tuneV3.greenHueMin && hsv[0] < tuneV3.greenHueMax){
            return 1;
        }
        if(hsv[0] > tuneV3.purpleHueMin && hsv[0] < tuneV3.purpleHueMax){
            return 2;
        }
        return 0;
    }
    public void enable(boolean enabled){
        this.enabled = enabled;
    }
    public void setSamplingDelay(int delay){
        samplingDelay = delay;
    }
    public void setSampleSize(int size){
        sampleSize = size;
        samples = new int[sampleSize];
    }
    public boolean getOuttakeState() {
        return slotChanger.isOuttake();
    }
    public int getCurrentSlotColor() {
        return slotColor[slotChanger.getSlot() - 1];
    }
    public void setInventory(int[] inventory){
        Arrays.fill(slotColor, 0);
        if(inventory == null) return;
        for(int i = 0; i < slotColor.length && i < inventory.length; i++){
            if(inventory[i] == 1 || inventory[i] == 2) slotColor[i] = inventory[i];
        }
    }
    public void scanSlotsForVerification(){
        Arrays.fill(sensedSlotColor, 0);
        for(int i = 0; i < verificationColorSensors.length && i < sensedSlotColor.length; i++){
            verificationColorSensors[i].enable(true);
            verificationColorSensors[i].update();
            if(i < verificationSensorToSlot.length && verificationSensorToSlot[i] >= 0 && verificationSensorToSlot[i] < sensedSlotColor.length) {
                sensedSlotColor[verificationSensorToSlot[i]] = classifySlot(verificationColorSensors[i]);
            }
            verificationColorSensors[i].enable(false);
        }
    }
    public boolean scanAllSlotsWhenIntakeAligned(){
        if(!slotChanger.isIntakeAligned()) return false;
        scanSlotsForVerification();
        for(int i = 0; i < slotColor.length; i++){
            slotColor[i] = sensedSlotColor[i];
        }
        return true;
    }
    public boolean isCurrentSlotDetectedByVerification(){
        scanSlotsForVerification();
        return sensedSlotColor[slotChanger.getSlot() - 1] != 0;
    }
    public void setVerificationAlignment(boolean enabled){
        slotChanger.setForceIntakeAlignment(enabled);
    }
    public boolean isStalled(){
        return slotChanger.isStalled();
    }
    public void clearStall(){
        slotChanger.clearStall();
    }
    public void moveToColor(int color){
        if(slotChanger.isOuttake()) {
            for (int i = 0; i < slotColor.length; i++) {
                if (slotColor[i] == color) {
                    slotChanger.setTargetSlot(i + 1);
                    return;
                }
            }
            for (int i = 0; i < slotColor.length; i++) {
                if (slotColor[i] != 0) {
                    slotChanger.setTargetSlot(i + 1); // FALLBACK
                    return;
                }
            }
            slotChanger.setOuttake(false); // RESET IF EMPTY
        }
    }
    public void clearCurrentSlot(){
            slotColor[slotChanger.getSlot() - 1] = 0;
    }
    public void update(){
        if(slotChanger.isCustomTargetAngleEnabled()) {
            slotChanger.enable(true);
            colorSensor.enable(false);
            colorSensor.update();
            for (ColorSensor verificationColorSensor : verificationColorSensors) {
                verificationColorSensor.enable(false);
            }
            sampleIndex = 0;
            sampleTimer.reset();
            return;
        }
        if(!enabled) {
            slotChanger.enable(false);
            slotChanger.update();
            colorSensor.enable(false);
            colorSensor.update();
            for (ColorSensor verificationColorSensor : verificationColorSensors) {
                verificationColorSensor.enable(false);
            }
            sampleIndex = 0;
            sampleTimer.reset();
            return;
        }
        slotChanger.enable(enabled);
        int currentSlot = slotChanger.getSlot() - 1;
        if(!slotChanger.isBusy() && !slotChanger.isOuttake() && !isFull()) {
            colorSensor.enable(true);
            colorSensor.update();
            if(sampleTimer.milliseconds() > samplingDelay) {
                samples[sampleIndex++] = classifySlot(colorSensor);
                if(sampleIndex == sampleSize){
                    int count1 = 0;
                    int count2 = 0;
                    for(int i = 0; i < sampleSize; i++){
                        if(samples[i] == 1) count1++;
                        else if(samples[i] == 2) count2++;
                    }
                    if(count1 > sampleSize/2) slotColor[currentSlot] = 1;
                    else if(count2 > sampleSize/2) slotColor[currentSlot] = 2;
                    else slotColor[currentSlot] = 0;
                    sampleIndex = 0;
                    if(slotColor[currentSlot] != 0) {
                        if (!isFull()) {
                            for (int i = 0; i < slotColor.length; i++)
                                if (slotColor[i] == 0) {
                                    slotChanger.setTargetSlot(i + 1);
                                    break;
                                }
                        } else {
                            slotChanger.setOuttake(true);
                        }
                    }
                }
                sampleTimer.reset();
            }

        } else {
            colorSensor.enable(false);
            sampleIndex = 0;
            sampleTimer.reset();
        }
        slotChanger.update();
    }
    public void appendTelemetry(Telemetry telemetry) {
        telemetry.addLine("--- Spindexer ---");
        telemetry.addData("Spindexer Enabled", enabled);
        telemetry.addData("Spindexer Full", isFull());
        telemetry.addData("Spindexer Outtake", getOuttakeState());
        telemetry.addData("Spindexer Current Slot", slotChanger.getSlot());
        telemetry.addData("Spindexer Current Color", getCurrentSlotColor());
        telemetry.addData("Spindexer Slot Colors", Arrays.toString(slotColor));
        telemetry.addData("Spindexer Verification Colors", Arrays.toString(sensedSlotColor));
        telemetry.addData("Spindexer Samples", Arrays.toString(samples));
        telemetry.addData("Spindexer Sample Index/Size", "%d / %d", sampleIndex, sampleSize);
        telemetry.addData("Spindexer Sample Timer", "%.0f / %d ms", sampleTimer.milliseconds(), samplingDelay);
        colorSensor.appendTelemetry(telemetry, "Main Color");
        for(int i = 0; i < verificationColorSensors.length; i++) {
            verificationColorSensors[i].appendTelemetry(telemetry, "Verify Color " + (i + 1));
        }
        slotChanger.appendTelemetry(telemetry);
    }
}
// WORK IN PROGRESS
