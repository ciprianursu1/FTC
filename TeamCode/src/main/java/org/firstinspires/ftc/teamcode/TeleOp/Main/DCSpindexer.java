package org.firstinspires.ftc.teamcode.TeleOp.Main;
import android.graphics.Color;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Arrays;

public class DCSpindexer {
    final double TICKS_PER_REV = 384.5;
    final double TICKS_PER_120 = TICKS_PER_REV/3.0;
    final double SPINDEXER_SPEED = 0.3;
    final double TRANSFER_UP = 0.02;
    final double TRANSFER_DOWN = 0.255;
    final double GREEN_MIN = 130;
    final double GREEN_MAX = 160;
    final double PURPLE_MIN = 200;
    final double PURPLE_MAX = 230;
    final double DISTANCE_THRESHOLD = 40.0;
    final int CONFIDENCE_THRESHOLD = 3;
    private enum ArtifactColor{
        PURPLE,GREEN,EMPTY
    }
    private enum SpindexerState{
        IDLE,
        ROTATING_TO_EMPTY,
        WAITING_FOR_OBJECT,
        CLASSIFYING,
        ROTATING_TO_OUTTAKE,
        TRANSFERRING
    }
    ArtifactColor[] slotColors = new ArtifactColor[3];
    SpindexerState state = SpindexerState.IDLE;
    int targetPosition = 0;
    double delay = 0;
    boolean transferUp = false;
    boolean readyToShoot = false;
    boolean rotating = false;
    boolean enabledSorting = false;
    int currentSlot = 0;
    int targetSlot = 0;
    int motifIndex = 0;
    public boolean requestingOuttake = false;
    DcMotorEx spindexer;
    Servo transfer;
    RevColorSensorV3[] colorSensors = new RevColorSensorV3[3];
    int[] purpleConfidence = new int[3];
    int[] greenConfidence = new int[3];
    int[] emptyConfidence = new int[3];
    ArtifactColor[] motif = new ArtifactColor[3];
    ElapsedTime time = new ElapsedTime();
    public DCSpindexer(HardwareMap hwMap, String sensor1Name , String sensor2Name, String sensor3Name, String motorName, String servoName){
        spindexer = hwMap.get(DcMotorEx.class,motorName);
        transfer = hwMap.get(Servo.class,servoName);
        colorSensors[0] = hwMap.get(RevColorSensorV3.class,sensor1Name);
        colorSensors[1] = hwMap.get(RevColorSensorV3.class,sensor2Name);
        colorSensors[2] = hwMap.get(RevColorSensorV3.class,sensor3Name);
    }
    public void init(){
        spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spindexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transfer.setDirection(Servo.Direction.REVERSE);
        transfer.setPosition(TRANSFER_DOWN);
        Arrays.fill(slotColors,ArtifactColor.EMPTY);
        Arrays.fill(purpleConfidence,0);
        Arrays.fill(greenConfidence,0);
        Arrays.fill(emptyConfidence,0);
    }
    public void setMotif (int tagID){
        switch (tagID){
            case 21:
                motif = new ArtifactColor[]{ArtifactColor.GREEN,ArtifactColor.PURPLE,ArtifactColor.PURPLE};
                break;
            case 22:
                motif = new ArtifactColor[]{ArtifactColor.PURPLE,ArtifactColor.GREEN,ArtifactColor.PURPLE};
                break;
            case 23:
                motif = new ArtifactColor[]{ArtifactColor.PURPLE,ArtifactColor.PURPLE,ArtifactColor.GREEN};
                break;
        }
    }
    public void requestOuttake(){
        requestingOuttake = true;
    }
    public void cancelOuttake(){
        requestingOuttake = false;
    }
    private void startDelay(int ms){
        delay = ms;
        time.reset();
    }
    private boolean delayActive(){
        return time.milliseconds() < delay;
    }
    public void setReady(boolean ready){
        readyToShoot = ready;
    }
    public boolean spindexerFull(){
        for (int i = 0; i < 3; i++) {
            if (slotColors[i] == ArtifactColor.EMPTY) return false;
        }
        return true;
    }
    public boolean spindexerEmpty(){
        for (int i = 0; i < 3; i++) {
            if (slotColors[i] != ArtifactColor.EMPTY) return false;
        }
        return true;
    }
    public void enableSorting(boolean enable){
        enabledSorting = enable;
    }
    private int findSlotForColor(ArtifactColor color) {
        if(enabledSorting) {
            for (int i = 0; i < 3; i++) {
                if (slotColors[i] == color)
                    return i;
            }
        }
        for (int i = 0; i < 3; i++) {
            if (slotColors[i] != ArtifactColor.EMPTY)
                return i;
        }
        return -1;
    }
    private ArtifactColor getColor(RevColorSensorV3 sensor) {
        NormalizedRGBA color = sensor.getNormalizedColors();
        float[] hsvValues = new float[3];
        Color.colorToHSV(color.toColor(),hsvValues);
        double h = hsvValues[0];
        double s = hsvValues[1];
        double v = hsvValues[2];
        if (s < 0.1) return ArtifactColor.EMPTY;
        else if (v < 0.1) return ArtifactColor.EMPTY;
        if (h > GREEN_MIN && h < GREEN_MAX) return ArtifactColor.GREEN;
        else if (h > PURPLE_MIN && h < PURPLE_MAX) return ArtifactColor.PURPLE;
        else return ArtifactColor.EMPTY;
    }
    private void rotateToSlot(int slot) {

        targetSlot = slot;

        int delta = (targetSlot - currentSlot + 3) % 3;

        int moveTicks = 0;

        if (delta == 1) moveTicks = (int) TICKS_PER_120;
        else if (delta == 2) moveTicks = -(int) TICKS_PER_120;

        targetPosition += moveTicks;

        spindexer.setTargetPosition(targetPosition);
        spindexer.setPower(SPINDEXER_SPEED);
    }

    public void update() {
        if (delayActive()) return;
        switch (state) {
            case IDLE:
                spindexer.setPower(0);
                break;
            case ROTATING_TO_EMPTY:
                for (RevColorSensorV3 sensor : colorSensors) {
                    if (sensor instanceof SwitchableLight) {
                        ((SwitchableLight) sensor).enableLight(false);
                    }
                }
                if(!spindexerFull() && !requestingOuttake) {
                    if (!rotating) {
                        for (int i = 0; i < 3; i++) {
                            if (slotColors[i] == ArtifactColor.EMPTY) {
                                rotateToSlot(i);
                                rotating = true;
                                break;
                            }
                        }
                    }

                    if (!spindexer.isBusy()) {
                        spindexer.setPower(0);
                        rotating = false;
                        currentSlot = targetSlot;
                        state = SpindexerState.WAITING_FOR_OBJECT;
                    }
                } else {
                    targetPosition += (int)(TICKS_PER_REV / 2.0);
                    int nextSlot = findSlotForColor(motif[0]);
                    if(nextSlot != -1) {
                        rotateToSlot(nextSlot);
                        state = SpindexerState.ROTATING_TO_OUTTAKE;
                    }

                }
                break;
            case CLASSIFYING:
                for (RevColorSensorV3 sensor : colorSensors) {
                    if (sensor instanceof SwitchableLight) {
                        ((SwitchableLight) sensor).enableLight(true);
                    }
                }

                for (int i = 0; i < 3; i++) {
                    ArtifactColor currentReading = getColor(colorSensors[i]);

                    // Update confidence counters
                    switch (currentReading) {
                        case PURPLE:
                            purpleConfidence[i]++;
                            greenConfidence[i] = 0;
                            emptyConfidence[i] = 0;
                            break;
                        case GREEN:
                            purpleConfidence[i] = 0;
                            greenConfidence[i]++;
                            emptyConfidence[i] = 0;
                            break;
                        case EMPTY:
                            purpleConfidence[i] = 0;
                            greenConfidence[i] = 0;
                            emptyConfidence[i]++;
                            break;
                    }

                    // Update slot color if confidence threshold reached
                    if (purpleConfidence[i] >= CONFIDENCE_THRESHOLD) {
                        slotColors[i] = ArtifactColor.PURPLE;
                        purpleConfidence[i] = 0; // Reset after confirming
                    } else if (greenConfidence[i] >= CONFIDENCE_THRESHOLD) {
                        slotColors[i] = ArtifactColor.GREEN;
                        greenConfidence[i] = 0;
                    } else if (emptyConfidence[i] >= CONFIDENCE_THRESHOLD) {
                        slotColors[i] = ArtifactColor.EMPTY;
                        emptyConfidence[i] = 0;
                    }
                }
                if(slotColors[0] != ArtifactColor.EMPTY) {
                    state = SpindexerState.ROTATING_TO_EMPTY;
                }
                break;
            case WAITING_FOR_OBJECT:
                if (colorSensors[0] instanceof SwitchableLight) {
                    ((SwitchableLight) colorSensors[0]).enableLight(true);
                }
                if (purpleConfidence[0] != 0 || greenConfidence[0] != 0 || emptyConfidence[0] != 0) {
                    Arrays.fill(purpleConfidence, 0);
                    Arrays.fill(greenConfidence, 0);
                    Arrays.fill(emptyConfidence, 0);
                }
                if (getColor(colorSensors[0]) != ArtifactColor.EMPTY  && colorSensors[0].getDistance(DistanceUnit.MM) < DISTANCE_THRESHOLD){
                    startDelay(150);
                    state = SpindexerState.CLASSIFYING;
                }
                break;
            case TRANSFERRING:
                if(!requestingOuttake) {
                    state = SpindexerState.ROTATING_TO_EMPTY;
                    if(transferUp){
                        transfer.setPosition(TRANSFER_DOWN);
                        startDelay(300);
                        transferUp = false;
                    }
                    break;
                }
                if(transferUp && readyToShoot) {
                    transfer.setPosition(TRANSFER_UP);
                    startDelay(300);
                    transferUp = false;
                    slotColors[currentSlot] = ArtifactColor.EMPTY;
                } else {
                    transfer.setPosition(TRANSFER_DOWN);
                    startDelay(250);
                    if (spindexerEmpty()) {
                        state = SpindexerState.ROTATING_TO_EMPTY;
                    } else {
                        int nextSlot = findSlotForColor(motif[motifIndex%2 + 1]);
                        motifIndex++;
                        if(nextSlot != -1) {
                            rotateToSlot(nextSlot);
                            state = SpindexerState.ROTATING_TO_OUTTAKE;
                        } else {
                            requestingOuttake = false;
                            state = SpindexerState.ROTATING_TO_EMPTY;
                        }
                    }
                }
                break;
            case ROTATING_TO_OUTTAKE:
                if (!spindexer.isBusy()) {
                    spindexer.setPower(0);
                    currentSlot = targetSlot;
                    if(requestingOuttake) {
                        state = SpindexerState.TRANSFERRING;
                    }
                    transferUp = true;
                }
                break;
        }
    }

}
