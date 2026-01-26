package org.firstinspires.ftc.teamcode.TeleOp.Main;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveTrain {
    //Motor sasiu
    DcMotor front_left;
    DcMotor front_right;
    DcMotor back_left;
    DcMotor back_right;

    private final Gamepad gamepad;

    public DriveTrain(HardwareMap hardwareMap, Gamepad gamepad, String front_left, String front_right, String back_left, String back_right) {
        this.gamepad = gamepad;

        this.front_left = hardwareMap.get(DcMotor.class,front_left);
        this.front_right = hardwareMap.get(DcMotor.class,front_right);
        this.back_left = hardwareMap.get(DcMotor.class,back_left);
        this.back_right = hardwareMap.get(DcMotor.class,back_right);


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
    public void update() {
            double left_x  = gamepad.left_stick_x;
            double left_y  = -gamepad.left_stick_y;
            double right_x = gamepad.right_stick_x;

            double front_left_pw  = left_y + left_x + right_x;
            double back_left_pw   = left_y - left_x + right_x;
            double front_right_pw = left_y - left_x - right_x;
            double back_right_pw  = left_y + left_x - right_x;

            double max = Math.max(Math.abs(front_left_pw),
                    Math.max(Math.abs(back_left_pw),
                            Math.max(Math.abs(front_right_pw), Math.abs(back_right_pw))));

            if (max > 1.0) {
                front_left_pw  /= max;
                back_left_pw   /= max;
                front_right_pw /= max;
                back_right_pw  /= max;
            }

            front_left.setPower(front_left_pw);
            back_left.setPower(back_left_pw);
            front_right.setPower(front_right_pw);
            back_right.setPower(back_right_pw);
        }
    }
