package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

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
    public MecanumDrive(Gamepad gamepad, DcMotor front_left, DcMotor front_right, DcMotor back_left, DcMotor back_right, boolean isAuto) {
        this.gamepad = gamepad;

        this.front_left = front_left;
        this.front_right = front_right;
        this.back_left = back_left;
        this.back_right = back_right;
        this.isAuto = isAuto;

    }
    public void init() {
        this.front_right.setDirection(DcMotorSimple.Direction.REVERSE);
        this.back_right.setDirection(DcMotorSimple.Direction.REVERSE);
        this.front_left.setDirection(DcMotorSimple.Direction.FORWARD);
        this.back_left.setDirection(DcMotorSimple.Direction.FORWARD);
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
        double x = squareInput(deadzone(gamepad.left_stick_x));
        double y = squareInput(deadzone(-gamepad.left_stick_y));
        double turn = squareInput(deadzone(gamepad.right_stick_x));

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
    }

