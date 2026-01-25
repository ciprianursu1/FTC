package org.firstinspires.ftc.teamcode.TeleOp.Main;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.TeleOp.SensorV2;
import org.firstinspires.ftc.teamcode.TeleOp.SensorV3;

import java.io.BufferedInputStream;

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
    private int currentOuttakeStep = 0;
    private int CurrentSlot = 0;
    private int ballsLoaded = 0;
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
                ballsLoaded = 0;
                if(IntakeSensor.getDetectedColor() != Sample.Colors.NONE && ballsLoaded < 3){
                    IntakeSlotColors[ballsLoaded++] = IntakeSensor.getDetectedColor();
                }
                if(AuxSensor1.getDetectedColor() != Sample.Colors.NONE && ballsLoaded < 3){
                    IntakeSlotColors[ballsLoaded++] = AuxSensor1.getDetectedColor();
                }
                if(AuxSensor2.getDetectedColor() != Sample.Colors.NONE && ballsLoaded < 3){
                    IntakeSlotColors[ballsLoaded++] = AuxSensor2.getDetectedColor();
                }

                switch (ballsLoaded){
                    case 1:
                        SlotChangerPos = IntakeSlotPositions[1];
                        break;
                    case 2:
                        SlotChangerPos = IntakeSlotPositions[2];
                        break;
                    case 3:
                        SlotChangerPos = OuttakeSlotPositions[0];
                        break;
                }
                break;
            case OUTTAKE:
                deactivateColorSensors();
                switch (currentOuttakeStep++){
                    case 1:
                    case 4:
                    case 7:
                        OuttakeTransfer.setPosition(OuttakeTransferUpPos);
                        startDelay(TransferUpDelay);
                        break;
                    case 2:
                    case 5:
                    case 8:
                        OuttakeTransfer.setPosition(OuttakeTransferDownPos);
                        startDelay(TransferDownDelay);
                        break;
                    case 3:
                        SlotChangerPos = OuttakeSlotPositions[1];
                        break;
                    case 6:
                        SlotChangerPos = OuttakeSlotPositions[2];
                        break;
                    case 9:
                        SlotChangerPos = IntakeSlotPositions[0];
                        currentOuttakeStep = 0;
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
    private void rotateSlotChangerCCW() {
        switch (spindexerState){
            case INTAKE:
                setSlotChangerPos(IntakeSlotPositions[CurrentSlot%3]);
                break;
            case OUTTAKE:
                setSlotChangerPos(OuttakeSlotPositions[CurrentSlot%3]);
                break;
        }
    }
    private void rotateSlotChangerCW() {
        switch (spindexerState){
            case OUTTAKE:
                setSlotChangerPos(OuttakeSlotPositions[++CurrentSlot%3]);
                break;
            case INTAKE:
                setSlotChangerPos(IntakeSlotPositions[++CurrentSlot%3]);
                break;

        }

    }

    private void setSlotChangerPos(double pos){
        if(Math.abs(SlotChanger1.getPosition() - pos) > SlotChangerPosPerDegree) return;
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
}
