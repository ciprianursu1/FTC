package org.firstinspires.ftc.teamcode.TeleOp.Main;
import android.graphics.Color;
import android.graphics.pdf.PdfRenderer;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Arrays;

public class DCSpindexer {
    final double TICKS_PER_REV = 384.5;
    final double TICKS_PER_120 = TICKS_PER_REV/3.0;
    final double SPINDEXER_SPEED = 0.8;
    final double TRANSFER_UP = 0.02;
    final double TRANSFER_DOWN = 0.26;
    final double GREEN_MIN = 140;
    final double GREEN_MAX = 175;
    final double PURPLE_MIN = 190;
    final double PURPLE_MAX = 250;
    final int CONFIDENCE_THRESHOLD = 1;
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
    ArtifactColor[] slotColors = {ArtifactColor.EMPTY,ArtifactColor.EMPTY,ArtifactColor.EMPTY};
    SpindexerState state = SpindexerState.ROTATING_TO_EMPTY;
    int targetPosition = 0;
    int currentPosition = 0;
    double delay = 0;
    boolean transferUp = false;
    boolean readyToShoot = false;
    boolean rotating = false;
    boolean enabledSorting = false;
    int currentSlot = 0;
    int targetSlot = 0;
    int motifIndex = 0;
    int lastError = 0;
    final double kP = 0.01;
    final double kD = 0.0002;
    public boolean requestingOuttake = false;
    boolean lastDelayActive = false;
    Telemetry telemetry;
    DcMotorEx spindexer;
    Servo transfer;
    RevColorSensorV3[] colorSensors = new RevColorSensorV3[3];
    int[] purpleConfidence = new int[3];
    int[] greenConfidence = new int[3];
    int[] emptyConfidence = new int[3];
    ArtifactColor[] motif = {ArtifactColor.PURPLE,ArtifactColor.PURPLE,ArtifactColor.GREEN};
    ElapsedTime time = new ElapsedTime();
    boolean preOuttakeOffsetDone = false;

    public DCSpindexer(HardwareMap hwMap, String sensor1Name , String sensor2Name, String sensor3Name, String motorName, String servoName, Telemetry telemetry){
        spindexer = hwMap.get(DcMotorEx.class,motorName);
        transfer = hwMap.get(Servo.class,servoName);
        colorSensors[0] = hwMap.get(RevColorSensorV3.class,sensor1Name);
        colorSensors[1] = hwMap.get(RevColorSensorV3.class,sensor2Name);
        colorSensors[2] = hwMap.get(RevColorSensorV3.class,sensor3Name);
        this.telemetry = telemetry;
    }
    public void init(){
        for(int i = 0; i < 3; i++) {
            colorSensors[i].setGain(5);
        }
        spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        spindexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transfer.setDirection(Servo.Direction.REVERSE);
        transfer.setPosition(TRANSFER_DOWN);
        spindexer.setTargetPositionTolerance(1);
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
        if (s < 0.1) return ArtifactColor.EMPTY;
        if (h > GREEN_MIN && h < GREEN_MAX) return ArtifactColor.GREEN;
        else if (h > PURPLE_MIN && h < PURPLE_MAX) return ArtifactColor.PURPLE;
        else return ArtifactColor.EMPTY;
    }
    private void shiftSlotsRight(int times) {
        for (int t = 0; t < times; t++) {
            ArtifactColor temp = slotColors[2];
            slotColors[2] = slotColors[1];
            slotColors[1] = slotColors[0];
            slotColors[0] = temp;
        }
    }

    private void shiftSlotsLeft(int times) {
        for (int t = 0; t < times; t++) {
            ArtifactColor temp = slotColors[0];
            slotColors[0] = slotColors[1];
            slotColors[1] = slotColors[2];
            slotColors[2] = temp;
        }
    }

    private void rotateToSlot(int slot) {
        targetSlot = slot;

        int delta = (targetSlot - currentSlot + 3) % 3;
        int moveTicks = 0;

        if (delta == 1) {
            moveTicks = -(int) TICKS_PER_120;
//            if(state == SpindexerState.ROTATING_TO_EMPTY || state == SpindexerState.CLASSIFYING) {
//                shiftSlotsRight(1);
//            }
        }
        else if (delta == 2) {
            moveTicks = (int) TICKS_PER_120;
//            if(state == SpindexerState.ROTATING_TO_EMPTY || state == SpindexerState.CLASSIFYING) {
//                shiftSlotsLeft(1);
//            }
        }

        targetPosition += moveTicks;

        rotating = true;
    }
    private void updateSpindexerPID(int targetPosition) {
        currentPosition = spindexer.getCurrentPosition();

        int error = targetPosition - currentPosition;

        int derivative = error - lastError;
        lastError = error;

        double output =
                (error * kP) +
                        (derivative * kD);

        output = Math.max(-SPINDEXER_SPEED, Math.min(SPINDEXER_SPEED, output));

        spindexer.setPower(output);
    }
    private static final double POSITION_TOLERANCE = 1;
    int notBusyConfidence = 0;
    final int BUSY_THRESHOLD = 3;
    private boolean spindexerAtTarget() {
        if (delayActive()){
            notBusyConfidence = 0;
            return false;
        }
        if (Math.abs(targetPosition - currentPosition) > POSITION_TOLERANCE){
            notBusyConfidence = 0;
            return false;
        }
        notBusyConfidence++;
        return notBusyConfidence > BUSY_THRESHOLD;
    }


    public void update() {
        currentPosition = spindexer.getCurrentPosition();
        boolean delayNow = delayActive();

        if (lastDelayActive && !delayNow) {
            rotating = false; // allow new movement after delay
        }

        lastDelayActive = delayNow;
        if (delayNow){
            updateSpindexerPID(currentPosition);
            return;
        }
        updateSpindexerPID(targetPosition);
        sendTelemetry();
        switch (state) {
            case IDLE:
                break;
            case ROTATING_TO_EMPTY:
                for (RevColorSensorV3 sensor : colorSensors) {
                    if (sensor instanceof SwitchableLight) {
                        ((SwitchableLight) sensor).enableLight(false);
                    }
                }
                if(!requestingOuttake) {
                    if (!rotating) {
                        for (int i = 0; i < 3; i++) {
                            if (slotColors[i] == ArtifactColor.EMPTY) {
                                rotateToSlot(i);
                                break;
                            }
                        }
                    }

                    if (spindexerAtTarget()) {
                        rotating = false;
                        currentSlot = targetSlot;
                        state = SpindexerState.WAITING_FOR_OBJECT;
                        preOuttakeOffsetDone = false;
                    }
                } else {
                    if (spindexerEmpty()) break;
                    if(!preOuttakeOffsetDone) {
                        targetPosition += (int) (TICKS_PER_REV / 2.0) - 9;
                        preOuttakeOffsetDone = true;
                    }
                    int nextSlot = findSlotForColor(motif[0]);
                    if(nextSlot != -1) {
                        rotateToSlot(nextSlot);
                        state = SpindexerState.ROTATING_TO_OUTTAKE;
                        preOuttakeOffsetDone = false;
                    }

                }
                break;
            case CLASSIFYING:
                // Turn LEDs on
                for (RevColorSensorV3 sensor : colorSensors) {
                    if (sensor instanceof SwitchableLight) {
                        ((SwitchableLight) sensor).enableLight(true);
                    }
                }

                boolean classified = false;

                for (int i = 0; i < 3; i++) {
                    ArtifactColor reading = getColor(colorSensors[i]);

                    // Update confidence counters
                    if (reading == ArtifactColor.PURPLE) {
                        purpleConfidence[i]++;
                        greenConfidence[i] = 0;
                        emptyConfidence[i] = 0;
                    } else if (reading == ArtifactColor.GREEN) {
                        purpleConfidence[i] = 0;
                        greenConfidence[i]++;
                        emptyConfidence[i] = 0;
                    } else {
                        purpleConfidence[i] = 0;
                        greenConfidence[i] = 0;
                        emptyConfidence[i]++;
                    }

                    // Check confidence threshold
                    if (purpleConfidence[i] >= CONFIDENCE_THRESHOLD) {
                        slotColors[i] = ArtifactColor.PURPLE;
                        classified = true;
                    } else if (greenConfidence[i] >= CONFIDENCE_THRESHOLD) {
                        slotColors[i] = ArtifactColor.GREEN;
                        classified = true;
                    } else if (emptyConfidence[i] >= CONFIDENCE_THRESHOLD) {
                        slotColors[i] = ArtifactColor.EMPTY;
                        classified = true;
                    }
                }

                // Stay in CLASSIFYING until a color is confirmed
                if (classified) {
                    Arrays.fill(purpleConfidence, 0);
                    Arrays.fill(greenConfidence, 0);
                    Arrays.fill(emptyConfidence, 0);
                    if(slotColors[0] != ArtifactColor.EMPTY) {
                        state = SpindexerState.ROTATING_TO_EMPTY;
                    }
                }
                if(requestingOuttake){
                    if (spindexerEmpty()) break;
                    if(!preOuttakeOffsetDone) {
                        targetPosition += (int) (TICKS_PER_REV / 2.0) - 9;
                        preOuttakeOffsetDone = true;
                    }
                    motifIndex = 0;
                    int nextSlot = findSlotForColor(motif[0]);
                    if(nextSlot != -1) {
                        rotateToSlot(nextSlot);
                        state = SpindexerState.ROTATING_TO_OUTTAKE;
                        preOuttakeOffsetDone = false;
                    }
                }

                break;
            case WAITING_FOR_OBJECT:
                state = SpindexerState.CLASSIFYING;
                break;
            case TRANSFERRING:
                if(!requestingOuttake) {
                    currentSlot = 0;
                    targetPosition = 0;
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
                        int nextSlot = findSlotForColor(motif[motifIndex%2 + 1]);
                        motifIndex++;
                        if(nextSlot != -1) {
                            rotateToSlot(nextSlot);
                            state = SpindexerState.ROTATING_TO_OUTTAKE;
                        } else {
                            requestingOuttake = false;
                            targetPosition = 0;
                            state = SpindexerState.ROTATING_TO_EMPTY;
                        }
                    }
                break;
            case ROTATING_TO_OUTTAKE:
                if (spindexerAtTarget()) {
                    rotating = false;
                    currentSlot = targetSlot;
                    if(requestingOuttake) {
                        state = SpindexerState.TRANSFERRING;
                    }
                    transferUp = true;
                }
                break;
        }
    }
    private void sendTelemetry() {
        if (telemetry == null) return;

        telemetry.addLine("=== DC Spindexer ===");

        telemetry.addData("State", state);
        telemetry.addData("Current Slot", currentSlot);
        telemetry.addData("Target Slot", targetSlot);
        telemetry.addData("Motor Target", targetPosition);
        telemetry.addData("Motor Pos", spindexer.getCurrentPosition());
        telemetry.addData("Motor Busy", spindexer.isBusy());

        telemetry.addData("Requesting Outtake", requestingOuttake);
        telemetry.addData("Ready To Shoot", readyToShoot);
        telemetry.addData("Sorting Enabled", enabledSorting);

        telemetry.addLine("--- Slots ---");
        for (int i = 0; i < 3; i++) {
            telemetry.addData(
                    "Slot " + i,
                    (slotColors[i] == ArtifactColor.PURPLE ? "PURPLE" : slotColors[i] == ArtifactColor.GREEN ? "GREEN" : "EMPTY")
            );
        }

        telemetry.addData("Transfer Servo", transfer.getPosition());
        telemetry.addData("Delay Active", delayActive());

        telemetry.update();
    }


}
