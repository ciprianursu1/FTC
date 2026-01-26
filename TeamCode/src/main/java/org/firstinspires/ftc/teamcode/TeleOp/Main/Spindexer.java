package org.firstinspires.ftc.teamcode.TeleOp.Main;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.TeleOp.SensorV2;
import org.firstinspires.ftc.teamcode.TeleOp.SensorV3;

import java.util.Arrays;

public class Spindexer {
    Servo SlotChanger1;
    Servo SlotChanger2;
    private final double SlotChangerRangeOfMotion = 355.0;
    private final double SlotChangerGearRatio = 2;
    private final double SlotChangerPosPerDegree = 1.0/(SlotChangerRangeOfMotion*SlotChangerGearRatio);
    private final double SlotChangerPosPer60Deg = 60*SlotChangerPosPerDegree;
    private final double SlotChangerPosPer120Deg = 120*SlotChangerPosPerDegree;
    private final double[] IntakeSlotPositions = {0,SlotChangerPosPer120Deg,2*SlotChangerPosPer120Deg};
    private final double[] OuttakeSlotPositions = {SlotChangerPosPer60Deg, SlotChangerPosPer120Deg + SlotChangerPosPer60Deg, 2*SlotChangerPosPer120Deg +  SlotChangerPosPer60Deg};
    private final Sample.Colors[] IntakeSlotColors = new Sample.Colors[3];
    private final Sample.Colors[] OuttakeSlotColors = new Sample.Colors[3];
    private final Sample.Colors[] Motif = {Sample.Colors.PURPLE, Sample.Colors.GREEN, Sample.Colors.PURPLE};
    SpindexerState stateBeforeSlotChange = SpindexerState.INTAKE;
    Servo OuttakeTransfer;
    SensorV2 IntakeSensor;
    SensorV3 AuxSensor1;
    SensorV3 AuxSensor2;
    final double OuttakeTransferDownPos = 0.285;
    final double OuttakeTransferUpPos = 0.000;
    private double SlotChangerPos = 0.000;
    public enum SpindexerState {
        INTAKE,OUTTAKE
    }
    private ElapsedTime timer = new ElapsedTime();
    private final int TransferUpDelay = 300;
    private final int TransferDownDelay = 300;
    private final int SlotChangeDelay = 300;
    private boolean waiting = false;
    private int waitDuration = 0;
    public SpindexerState spindexerState = SpindexerState.INTAKE;
    public boolean SlotChangerFull = false;
    private int currentOuttakeStep = 0;
    private int CurrentSlot = 0;
    private int ballsLoaded = 0;
    private int motifIndex = 0;
    private int nextOuttakeSlot = -1;
    private boolean enabledSorting = true;
    private boolean intakeDetectedPrev = false;
    private boolean aux1DetectedPrev = false;
    private boolean aux2DetectedPrev = false;
    private final int DEBOUNCE_MS = 20; // milliseconds to confirm a reading
    private ElapsedTime intakeTimer = new ElapsedTime();
    private ElapsedTime aux1Timer = new ElapsedTime();
    private ElapsedTime aux2Timer = new ElapsedTime();
    Spindexer(HardwareMap hwMap,String SlotChanger1Name,String SlotChanger2Name, String OuttakeTransferName,String IntakeSensorName,String AuxSensor1Name,String AuxSensor2Name){
        SlotChanger1 = hwMap.get(Servo.class,SlotChanger1Name);
        SlotChanger2 = hwMap.get(Servo.class,SlotChanger2Name);
        OuttakeTransfer = hwMap.get(Servo.class,OuttakeTransferName);
        IntakeSensor = new SensorV2();
        AuxSensor1 = new SensorV3();
        AuxSensor2 = new SensorV3();
        IntakeSensor.setSensor(hwMap,IntakeSensorName);
        AuxSensor1.setSensor(hwMap,AuxSensor1Name);
        AuxSensor2.setSensor(hwMap,AuxSensor2Name);
    }
    public void init(){
        setSlotChangerPos(IntakeSlotPositions[0]);
        OuttakeTransfer.setPosition(OuttakeTransferDownPos);
    }
    public void update(){
        if(waiting){
            if(timer.milliseconds() >= waitDuration){
                waiting = false;
            } else {
                return;
            }
        }
        switch (spindexerState){
            case INTAKE:
                activateColorSensors();
                updateColorSensors();
                boolean intakeDetected = IntakeSensor.getDetectedColor() != Sample.Colors.NONE;
                boolean aux1Detected = AuxSensor1.getDetectedColor() != Sample.Colors.NONE;
                boolean aux2Detected = AuxSensor2.getDetectedColor() != Sample.Colors.NONE;
                if (intakeDetected && !intakeDetectedPrev) intakeTimer.reset();
                if (intakeDetected && intakeTimer.milliseconds() >= DEBOUNCE_MS) {
                    IntakeSlotColors[0] = IntakeSensor.getDetectedColor();
                    intakeDetectedPrev = true;
                } else if (!intakeDetected) {
                    intakeDetectedPrev = false;
                }
                if (aux1Detected && !aux1DetectedPrev) aux1Timer.reset();
                if (aux1Detected && aux1Timer.milliseconds() >= DEBOUNCE_MS) {
                    IntakeSlotColors[1] = AuxSensor1.getDetectedColor();
                    aux1DetectedPrev = true;
                } else if (!aux1Detected) {
                    aux1DetectedPrev = false;
                }

                if (aux2Detected && !aux2DetectedPrev) aux2Timer.reset();
                if (aux2Detected && aux2Timer.milliseconds() >= DEBOUNCE_MS) {
                    IntakeSlotColors[2] = AuxSensor2.getDetectedColor();
                    aux2DetectedPrev = true;
                } else if (!aux2Detected) {
                    aux2DetectedPrev = false;
                }
                ballsLoaded = (IntakeSlotColors[0] != Sample.Colors.NONE ? 1 : 0)+ (IntakeSlotColors[1] != Sample.Colors.NONE ? 1 : 0) + (IntakeSlotColors[2] != Sample.Colors.NONE ? 1 : 0);

                switch (ballsLoaded){
                    case 0:
                        SlotChangerPos = IntakeSlotPositions[0];
                        break;
                    case 1:
                        SlotChangerPos = IntakeSlotPositions[1];
                        break;
                    case 2:
                        SlotChangerPos = IntakeSlotPositions[2];
                        break;
                    case 3:
                        SlotChangerPos = OuttakeSlotPositions[0];
                        SlotChangerFull = true;
                        break;
                }
                stateBeforeSlotChange = SpindexerState.INTAKE;
                break;
            case OUTTAKE:
                deactivateColorSensors();
                if (stateBeforeSlotChange == SpindexerState.INTAKE) {
                    mapIntakeToOuttakeSlots();
                    Arrays.fill(IntakeSlotColors, Sample.Colors.NONE);
                }
                stateBeforeSlotChange = spindexerState;
                switch (currentOuttakeStep++){
                    case 0:
                    case 3:
                    case 6:
                    case 9:
                        selectNextOuttakeSlot();
                        if (nextOuttakeSlot != -1) {
                            SlotChangerPos = OuttakeSlotPositions[nextOuttakeSlot];
                        } else {
                            SlotChangerPos = IntakeSlotPositions[0];
                            currentOuttakeStep = 0;
                            ballsLoaded = 0;
                            spindexerState = SpindexerState.INTAKE;
                        }
                        break;
                    case 1:
                    case 4:
                    case 7:
                        OuttakeTransfer.setPosition(OuttakeTransferUpPos);
                        OuttakeSlotColors[nextOuttakeSlot] = Sample.Colors.NONE;
                        startDelay(TransferUpDelay);
                        break;
                    case 2:
                    case 5:
                    case 8:
                        OuttakeTransfer.setPosition(OuttakeTransferDownPos);
                        startDelay(TransferDownDelay);
                        break;
                }
                break;
        }
        setSlotChangerPos(SlotChangerPos);
    }
    private void activateColorSensors(){
        IntakeSensor.enableSensor();
        AuxSensor1.enableSensor();
        AuxSensor2.enableSensor();
    }
    private void deactivateColorSensors(){
        IntakeSensor.disableSensor();
        AuxSensor1.disableSensor();
        AuxSensor2.disableSensor();
    }
    private void updateColorSensors(){
        IntakeSensor.updateColor();
        AuxSensor1.updateColor();
        AuxSensor2.updateColor();
    }
    public void enableSorting(){
        enabledSorting = true;
    }
    public void disableSorting(){
        enabledSorting = false;
    }

    private void setSlotChangerPos(double pos){
        if(Math.abs(SlotChanger1.getPosition() - pos) < SlotChangerPosPerDegree) return;
        if(pos > 1.0) pos = 1.0;
        else if (pos < 0) pos = 0;
        SlotChanger1.setPosition(pos);
        SlotChanger2.setPosition(pos);
        startDelay(SlotChangeDelay);
    }
    private void startDelay(int milliseconds){
        timer.reset();
        waitDuration = milliseconds;
        waiting = true;
    }
    private void mapIntakeToOuttakeSlots() {
        for (int i = 0; i < 3; i++) {
            OuttakeSlotColors[2 - i] = IntakeSlotColors[i];
        }
    }
    private void selectNextOuttakeSlot() {
        nextOuttakeSlot = -1;
        if(enabledSorting) {
            for (int i = 0; i < 3; i++) {
                if (OuttakeSlotColors[i] == Motif[motifIndex]) {
                    nextOuttakeSlot = i;
                    break;
                }
            }
            motifIndex = (motifIndex + 1) % Motif.length;
        }

        // If no match, just take the next available ball
        if (nextOuttakeSlot == -1) {
            for (int i = 0; i < 3; i++) {
                if (OuttakeSlotColors[i] != Sample.Colors.NONE) {
                    nextOuttakeSlot = i;
                    break;
                }
            }
        }
    }

}
