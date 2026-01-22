package org.firstinspires.ftc.teamcode.TeleOp.Main;

import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp
public class CipTest extends OpMode {
    private Thread highRateThread;
    boolean highRateThreadOn = false;
    ColorSensor ColorSensorSlot1;
    ColorSensor ColorSensorSlot2;
    ColorSensor ColorSensorSlot3;
    final int GREEN = 1, PURPLE = 2, BLACK = 0;
    int Color1 = 0, Color2 = 0, Color3 = 0;
    int indexSensor1 = 0, indexSensor2 = 0, indexSensor3 = 0;
    int[] last5Sensor1 = new int[5], last5Sensor2 = new int[5], last5Sensor3 = new int[5];
    Servo spinnerClose;
    Servo spinnerFar;
    final double SPINNER_SERVO_MAX_POS = 0.71;
    final double SPINNER_SERVO_MAX_POS_ACC = 0.708;
    final double SPINNER_SERVO_120_DEG = SPINNER_SERVO_MAX_POS / 3.0;
    final double SPINNER_SERVO_60_DEG = SPINNER_SERVO_MAX_POS / 6.0;
    final double SPINNER_SERVO_30_DEG = SPINNER_SERVO_MAX_POS / 12.0;
    final double SPINNER_SERVO_15_DEG = SPINNER_SERVO_MAX_POS / 24.0;
    private enum spinnerStates {
        STANDBY, INTAKE, BUSY_INTAKE, FULL, OUTTAKE, BUSY_OUTTAKE
    }
    spinnerStates spinnerState = spinnerStates.STANDBY;
    final int HIGH_RATE_LOOP_HZ = 500;
    final long HIGH_RATE_LOOP_MS = 1000/HIGH_RATE_LOOP_HZ;
    ElapsedTime autoRotateSpinnerTimer;
    ElapsedTime timer;
    int busyIntakeTime = 0;
    int busyOuttakeTime = 0;
    static final int INTAKE_DELAY =  1000;
    static final int OUTTAKE_DELAY = 1000;
    final int autoRotateSpinnerDelay = 500; //ms
    double spinnerPos = 0;
    int delaySpinner = 0;
    int[] colorsRelativeToOuttake = new int[3]; // 0 LEFT / 1 MIDDLE / 2 RIGHT
    DcMotor intake;
    private enum intakeStates {
        ON,OFF,REVERSE
    }
    intakeStates intakeState = intakeStates.OFF;
    DcMotorEx tureta;
    Servo unghiTureta;
    Servo ejector;
    private enum ejectorStates{
        UP,DOWN,FINISHED
    }
    ejectorStates ejectorState = ejectorStates.DOWN;
    final double EJECTOR_UP_POS = 0.05;
    final double EJECTOR_DOWN_POS = 0.285;
    DcMotor front_left;
    DcMotor front_right;
    DcMotor back_left;
    DcMotor back_right;
    DcMotorEx flywheel;
    final double RPM_PER_TICK_PER_SEC = 28*60;
    private enum flywheelStates{
        OFF,SPINUP,READY
    }
    flywheelStates flywheelState = flywheelStates.OFF;
    IMU imu;
    PinpointLocalizer pinpoint;
    Pose pose;
    double CoordX, CoordY, header;
    final double THROTTLE_EXPO = 1.6;
    private void InitWheels() {
        front_left = hardwareMap.dcMotor.get("lf");
        front_right = hardwareMap.dcMotor.get("rf");
        back_left = hardwareMap.dcMotor.get("lr");
        back_right = hardwareMap.dcMotor.get("rr");

        front_right.setDirection(DcMotorSimple.Direction.FORWARD);
        back_right.setDirection(DcMotorSimple.Direction.FORWARD);
        front_left.setDirection(DcMotorSimple.Direction.REVERSE);
        back_left.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    private void InitDc() {
        intake = hardwareMap.get(DcMotor.class, "intake");
        tureta = hardwareMap.get(DcMotorEx.class, "tureta");
        flywheel = (DcMotorEx) hardwareMap.get(DcMotor.class, "flywheel");

        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        tureta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tureta.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        tureta.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }
    private void InitServo() {
        ejector = hardwareMap.get(Servo.class, "ejector");
        ejector.setPosition(EJECTOR_DOWN_POS);
        unghiTureta = hardwareMap.get(Servo.class,"unghituretaoy"); // future use
        spinnerClose = hardwareMap.get(Servo.class, "SpinnerClose");
        spinnerFar = hardwareMap.get(Servo.class, "SpinnerFar");
        spinnerFar.setPosition(0);
        spinnerClose.setPosition(0);
    }
    private void InitAux() {
        ColorSensorSlot1 = hardwareMap.colorSensor.get("Color1");
        ColorSensorSlot2 = hardwareMap.colorSensor.get("Color2");
        ColorSensorSlot3 = hardwareMap.colorSensor.get("Color3");
        pinpoint=new PinpointLocalizer(hardwareMap, Constants.localizerConstants);
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
        );
        imu.initialize(new IMU.Parameters(orientation));
        Pose startPos=new Pose(0, 0, 0);
        pinpoint.setStartPose(startPos);
    }
    private int colorDetection(ColorSensor colorSensor) {
        int r = colorSensor.red();
        int g = colorSensor.green();
        int b = colorSensor.blue();
        int alpha = colorSensor.alpha();
        double h = getHue(r, g, b);
        int detected;
        if (alpha < 100 && (h == 150 || h == 144)) detected = 0;
        else if ((h > 215) || (alpha < 100 && (h == 160 || h == 180))) detected = PURPLE;
        else if (h > 135 && h < 160 && alpha > 100) detected = GREEN;
        else if ((h == 140 || h == 145) && alpha == 43) detected = BLACK;
        else if (h > 135 && h < 160 && alpha > 60) detected = GREEN;
        else if ((h == 210 || h == 220 || h == 225 || h == 200) && alpha < 100) detected = PURPLE;
        else detected = 0;
        return detected;
    }
    private int colorErrorCorrection(ColorSensor sensor, int[] last5, int index) {
        last5[index] = colorDetection(sensor);

        int count1 = 0, count2 = 0;
        for (int v : last5) {
            if (v == 1) count1++;
            else if (v == 2) count2++;
        }

        if (count1 >= 3) return 1;
        if (count2 >= 3) return 2;
        return 0;
    }
    private void updateColors() {
        Color1 = colorErrorCorrection(ColorSensorSlot1, last5Sensor1, indexSensor1);
        indexSensor1 = (indexSensor1 + 1) % 5;
        Color2 = colorErrorCorrection(ColorSensorSlot2, last5Sensor2, indexSensor2);
        indexSensor2 = (indexSensor2 + 1) % 5;
        Color3 = colorErrorCorrection(ColorSensorSlot3, last5Sensor3, indexSensor3);
        indexSensor3 = (indexSensor3 + 1) % 5;
    }
    private boolean spinnerFull() {
        return (Color1 == PURPLE || Color1 == GREEN) && (Color2 == PURPLE || Color2 == GREEN) && (Color3 == PURPLE || Color3 == GREEN);
    }
    private void autoRotateSpinner() {
        if(spinnerFull()){
            spinnerState = spinnerStates.FULL;
        }
        else if (Color1 != 0) {
            if (autoRotateSpinnerTimer.milliseconds() >= autoRotateSpinnerDelay) {
                spinnerPos += SPINNER_SERVO_120_DEG;
                spinnerState = spinnerStates.BUSY_INTAKE;
                autoRotateSpinnerTimer.reset();
            }
        }
    }
    private void updateSpinner(){
        spinnerPos -= (int)(spinnerPos/SPINNER_SERVO_MAX_POS)*SPINNER_SERVO_MAX_POS_ACC;
        spinnerClose.setPosition(spinnerPos);
        spinnerFar.setPosition(spinnerPos);

    }
    private void updateTelemetry() {
        telemetry.addData("INFO","Welcome to CIP Airlines");
        telemetry.addLine();
        telemetry.update();
    }
    private void rotateSlotsRight() {
        int temp = colorsRelativeToOuttake[0];
        colorsRelativeToOuttake[0] = colorsRelativeToOuttake[2];
        colorsRelativeToOuttake[2] = colorsRelativeToOuttake[1];
        colorsRelativeToOuttake[1] = temp;
        spinnerPos += SPINNER_SERVO_120_DEG;

    }

    private void rotateSlotsLeft() {
        int temp = colorsRelativeToOuttake[1];
        colorsRelativeToOuttake[0] = colorsRelativeToOuttake[1];
        colorsRelativeToOuttake[1] = colorsRelativeToOuttake[2];
        colorsRelativeToOuttake[2] = temp;
        spinnerPos -= SPINNER_SERVO_120_DEG;
    }
    private double getHue(int r, int g, int b) {
        int max = Math.max(r, Math.max(g, b));
        int min = Math.min(r, Math.min(g, b));
        if (max == min) return 0.0;

        double chroma = max - min;
        double h;

        if (max == r) h = (double) (g - b) / chroma;
        else if (max == g) h = (double) (b - r) / chroma + 2.0;
        else h = (double) (r - g) / chroma + 4.0;

        h *= 60.0;
        if (h < 0) h += 360.0;

        return h;
    }
    private void SetWheelsPower() {
        double left_x = Math.pow(gamepad1.left_stick_x,THROTTLE_EXPO);
        double left_y = -Math.pow(gamepad1.left_stick_y,THROTTLE_EXPO); // forward is negative
        double right_x = Math.pow(gamepad1.right_stick_x,THROTTLE_EXPO);

        double front_left_pw = left_y + left_x + right_x;
        double back_left_pw = left_y - left_x + right_x;
        double front_right_pw = left_y - left_x - right_x;
        double back_right_pw = left_y + left_x - right_x;

        // Normalize so no motor power exceeds 1.0
        double max = Math.max(Math.abs(front_left_pw),
                Math.max(Math.abs(back_left_pw),
                        Math.max(Math.abs(front_right_pw), Math.abs(back_right_pw))));
        if (max > 1.0) {
            front_left_pw /= max;
            back_left_pw /= max;
            front_right_pw /= max;
            back_right_pw /= max;
        }

        front_left.setPower(front_left_pw);
        back_left.setPower(back_left_pw);
        front_right.setPower(front_right_pw);
        back_right.setPower(back_right_pw);
    }

    private void readGamepads(){
        if(gamepad1.circleWasPressed() && spinnerState == spinnerStates.STANDBY){
            spinnerState = spinnerStates.INTAKE;
        }
        if(gamepad1.crossWasPressed() && spinnerState == spinnerStates.FULL){
            spinnerState = spinnerStates.OUTTAKE;
        }
        if(gamepad1.dpadUpWasPressed()){
            if(flywheelState == flywheelStates.OFF) {
                spinUpFlywheel();
            } else {
                flywheelState = flywheelStates.OFF;
            }
        }
        if(gamepad1.dpadDownWasPressed()){
            intakeState = intakeStates.REVERSE;
            intake.setPower(-0.8);
        }
        SetWheelsPower();
        if(gamepad1.shareWasPressed() && gamepad1.optionsWasPressed()){
            resetRobot();
        }
        //TODO
    }
    private void spinnerInOuttakePos() {
        spinnerPos += (spinnerState == spinnerStates.OUTTAKE ? 0 : SPINNER_SERVO_60_DEG);
        spinnerState = spinnerStates.BUSY_OUTTAKE;
    }
    private void spinnerInIntakePos() {
        spinnerPos += (spinnerState == spinnerStates.OUTTAKE ? SPINNER_SERVO_60_DEG : 0);
        spinnerState = spinnerStates.BUSY_INTAKE;
    }
    private boolean spinnerEmpty() {
        return (colorsRelativeToOuttake[0] == BLACK && colorsRelativeToOuttake[1] == BLACK && colorsRelativeToOuttake[2] == BLACK);
    }
    private void spinnerLogic(){
        switch (spinnerState){
            case INTAKE:
                spinnerInIntakePos();
                if(intakeState != intakeStates.REVERSE) {
                    intakeState = intakeStates.ON;
                }
                updateColors();
                autoRotateSpinner();
            case FULL:
                intakeState = intakeStates.OFF;
                spinnerInOuttakePos();
            case OUTTAKE:
                if(spinnerState == spinnerStates.OUTTAKE && ejectorState == ejectorStates.FINISHED){
                    ejector.setPosition(EJECTOR_UP_POS);
                    spinnerState = spinnerStates.BUSY_OUTTAKE;
                    ejectorState = ejectorStates.UP;
                } else if(spinnerState == spinnerStates.OUTTAKE && ejectorState == ejectorStates.UP) {
                    ejector.setPosition(EJECTOR_DOWN_POS);
                    spinnerState = spinnerStates.BUSY_OUTTAKE;
                    ejectorState = ejectorStates.DOWN;
                }
                //TODO: SORT HERE (future)
                if(spinnerState == spinnerStates.OUTTAKE) {
                    rotateSlotsRight();
                    spinnerState = spinnerStates.BUSY_OUTTAKE;
                }
                colorsRelativeToOuttake[1] = BLACK;
                ejectorState = ejectorStates.FINISHED;
                if(spinnerEmpty()) spinnerState = spinnerStates.STANDBY;
            case BUSY_INTAKE:
                if(busyIntakeTime == 0) {
                    busyIntakeTime = (int)timer.milliseconds();
                    return;
                }
                if(timer.milliseconds() - busyIntakeTime >= INTAKE_DELAY){
                    spinnerState = spinnerStates.INTAKE;
                    busyIntakeTime = 0;
                }
            case BUSY_OUTTAKE:
                if(busyOuttakeTime == 0) {
                    busyOuttakeTime = (int)timer.milliseconds();
                    return;
                }
                if(timer.milliseconds() - busyOuttakeTime >= OUTTAKE_DELAY){
                    spinnerState = spinnerStates.OUTTAKE;
                    busyOuttakeTime = 0;
                }

        }
    }
    private void spinUpFlywheel(){
        flywheelState = flywheelStates.SPINUP;
        flywheel.setVelocity(3000*RPM_PER_TICK_PER_SEC);
    }
    private void resetRobot(){
        flywheelState = flywheelStates.OFF;
        if(spinnerState == spinnerStates.OUTTAKE) {
            ejector.setPosition(EJECTOR_DOWN_POS);
        }
        spinnerState = spinnerStates.STANDBY;
    }
    public void init(){
        InitDc();
        InitWheels();
        InitAux();
        InitServo();
        highRateThread = new Thread(this::highRateLoop);
    }
    public void start() {
        highRateThreadOn = true;
        highRateThread.start();
        timer.reset();
    }
    public void loop(){
        readGamepads();
        spinnerLogic();
        //TODO
    }
    public void stop(){
        highRateThreadOn = false;
    }
    public void highRateLoop() {
        // WARNING: no hardware interactions (read / write)
        while (highRateThreadOn) {
            try {
                Thread.sleep(HIGH_RATE_LOOP_MS);
            } catch (InterruptedException ignored) {

            }
        }
    }
}
