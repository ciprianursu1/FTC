package org.firstinspires.ftc.teamcode.TeleOp.Main;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.arcrobotics.ftclib.command.*;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class AutoDriveOnSetPath extends LinearOpMode {
    Motor fl = new Motor(hardwareMap,"fl", Motor.GoBILDA.RPM_435),fr = new Motor(hardwareMap,"fr",Motor.GoBILDA.RPM_435),rl = new Motor(hardwareMap,"rl",Motor.GoBILDA.RPM_435),rr = new Motor(hardwareMap,"rr",Motor.GoBILDA.RPM_435);
    MecanumDrive robotDrive = new MecanumDrive(fl,fr,rl,rr);
    PinpointLocalizer pinpoint = new PinpointLocalizer(hardwareMap, Constants.localizerConstants);
    public void runOpMode() {
        fl.setInverted(true);
        fr.setInverted(true);
        rl.setInverted(false);
        rr.setInverted(false);
        fl.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rl.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rr.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        while(opModeIsActive()){

        }

    }
}
