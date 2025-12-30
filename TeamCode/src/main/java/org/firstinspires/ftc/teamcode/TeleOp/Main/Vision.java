package org.firstinspires.ftc.teamcode.TeleOp.Main;

import com.qualcomm.hardware.limelightvision.*;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.robotcore.util.Range;

public class Vision {

    Limelight3A limelight;
    IMU imu;
    Turret turret;
    LinearOpMode opMode;

    double scanSpeed = 0.05;
    boolean scanDir = true;
    double lastTx = 0;
    double txDeadzone = 3.5;

    long UltimaDataVazut = 0;
    long TimpDeLaPierdereaTargetului = 0;
    double TimpPauza = 0.3;
    double TimpCautareLocala = 6;

    double PutereScanareLocala = 0.12;
    double PerioadaSchimbariiSensului = 0.4;

    boolean ConditieScanarePlanetara = false;

    public Vision(LinearOpMode opMode, Turret turret) {
        this.opMode = opMode;
        this.turret = turret;
    }

    public void limeInit() {
        limelight = opMode.hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(5);
        limelight.start();

        imu = opMode.hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
        );
        imu.initialize(new IMU.Parameters(orientation));
    }

    public void aliniereTureta() {
        turret.lastTime = System.nanoTime();
        while (opMode.opModeIsActive()) {

            YawPitchRollAngles ypr = imu.getRobotYawPitchRollAngles();
            limelight.updateRobotOrientation(ypr.getYaw());

            LLResult result = limelight.getLatestResult();
            double tx;
            long OraActuala = System.nanoTime();

            if (result != null && result.isValid()) {
                tx = -result.getTx();
                lastTx = tx;
                UltimaDataVazut = OraActuala;
                TimpDeLaPierdereaTargetului = 0;
            } else {
                continue;
            }

            if (Math.abs(tx) <= txDeadzone) {
                turret.MotorTureta.setPower(0);
                turret.integral = 0;
                turret.lastError = 0;
                continue;
            }

            long now = System.nanoTime();
            double dt = (now - turret.lastTime) / 1e9;
            turret.lastTime = now;

            double error = -tx;
            turret.integral += error * dt;
            double derivative = (error - turret.lastError) / dt;
            turret.lastError = error;

            double output = turret.kP * error + turret.kI * turret.integral + turret.kD * derivative;
            output = Range.clip(output, -0.5, 0.5);
            output = turret.Limitare(output);

            turret.MotorTureta.setPower(output);
        }
    }
}
