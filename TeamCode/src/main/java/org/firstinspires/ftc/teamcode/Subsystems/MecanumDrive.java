package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MecanumDrive {
    //Motor sasiu
    DcMotor front_left;
    DcMotor front_right;
    DcMotor back_left;
    DcMotor back_right;
    final double DEADZONE = 0.08;
    double powerMultiplier = 1.0;
    private final Gamepad gamepad;
    boolean isFieldCentric = false;
    boolean isAuto = false;
    double lastX = 0;
    double lastY = 0;
    double lastTurn = 0;
    double lastFrontLeftPower = 0;
    double lastFrontRightPower = 0;
    double lastBackLeftPower = 0;
    double lastBackRightPower = 0;
    public MecanumDrive(Gamepad gamepad, DcMotor front_left, DcMotor front_right, DcMotor back_left, DcMotor back_right, boolean isAuto) {
        this.gamepad = gamepad;

        this.front_left = front_left;
        this.front_right = front_right;
        this.back_left = back_left;
        this.back_right = back_right;
        this.isAuto = isAuto;

    }
    public void init() {
        front_right.setDirection(DcMotorSimple.Direction.FORWARD);
        back_right.setDirection(DcMotorSimple.Direction.FORWARD);
        front_left.setDirection(DcMotorSimple.Direction.REVERSE);
        back_left.setDirection(DcMotorSimple.Direction.REVERSE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void setFieldCentric(boolean isFieldCentric){
        this.isFieldCentric = isFieldCentric;
    }
    public void update(double heading) {
        if(isAuto) return;
        double x = gamepad.left_stick_x;
        double y = -gamepad.left_stick_y;
        double turn = gamepad.right_stick_x;
        lastX = x;
        lastY = y;
        lastTurn = turn;

        if (isFieldCentric) {
            double cos = Math.cos(heading);
            double sin = Math.sin(heading);

            double rotatedX = x * cos - y * sin;
            double rotatedY = x * sin + y * cos;

            x = rotatedX;
            y = rotatedY;
        }

        double front_left_pw  = y + x + turn;
        double back_left_pw   = y - x + turn;
        double front_right_pw = y - x - turn;
        double back_right_pw  = y + x - turn;

        double max = Math.max(Math.abs(front_left_pw),
                Math.max(Math.abs(back_left_pw),
                        Math.max(Math.abs(front_right_pw), Math.abs(back_right_pw))));

        if (max > 1.0) {
            front_left_pw  /= max;
            back_left_pw   /= max;
            front_right_pw /= max;
            back_right_pw  /= max;
        }

        front_left.setPower(front_left_pw * powerMultiplier);
        back_left.setPower(back_left_pw * powerMultiplier);
        front_right.setPower(front_right_pw * powerMultiplier);
        back_right.setPower(back_right_pw * powerMultiplier);
        lastFrontLeftPower = front_left_pw * powerMultiplier;
        lastBackLeftPower = back_left_pw * powerMultiplier;
        lastFrontRightPower = front_right_pw * powerMultiplier;
        lastBackRightPower = back_right_pw * powerMultiplier;
    }
    private double deadzone(double value) {
        return Math.abs(value) < DEADZONE ? 0.0 : (value - Math.copySign(DEADZONE, value)) / (1.0 - DEADZONE);
    }
    public void setPowerMultiplier(double powerMultiplier){
        this.powerMultiplier = powerMultiplier;
    }
    private double squareInput(double v) {
        return Math.copySign(v * v, v);
    }
    public void appendTelemetry(Telemetry telemetry) {
        telemetry.addLine("--- Drive ---");
        telemetry.addData("Drive Auto", isAuto);
        telemetry.addData("Drive Mode", isFieldCentric ? "FIELD" : "ROBOT");
        telemetry.addData("Drive Multiplier", "%.2f", powerMultiplier);
        telemetry.addData("Drive Input X/Y/Turn", "%.3f / %.3f / %.3f", lastX, lastY, lastTurn);
        telemetry.addData("Drive Power FL/FR", "%.3f / %.3f", lastFrontLeftPower, lastFrontRightPower);
        telemetry.addData("Drive Power BL/BR", "%.3f / %.3f", lastBackLeftPower, lastBackRightPower);
        telemetry.addData("Drive Enc FL/FR", "%d / %d", front_left.getCurrentPosition(), front_right.getCurrentPosition());
        telemetry.addData("Drive Enc BL/BR", "%d / %d", back_left.getCurrentPosition(), back_right.getCurrentPosition());
    }
    }

