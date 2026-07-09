package org.firstinspires.ftc.teamcode.TeleOp.Main;

import com.bylazar.panels.Panels;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(name = "PID Velocity Tuner (Panels)", group = "TEST")
public class PanelsPIDTunerVelocity extends OpMode {
    private DcMotorEx motor;
    private TelemetryManager telemetryM;
    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorEx.class, "flywheel");
        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void loop() {
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(PIDTunerConfig.kP, PIDTunerConfig.kI, PIDTunerConfig.kD, PIDTunerConfig.kF));
        double currentTicks = motor.getVelocity();
        motor.setVelocity(PIDTunerConfig.targetTicks);
        telemetryM.addData("Velocity", currentTicks);
        telemetryM.addData("Target", PIDTunerConfig.targetTicks);
        telemetryM.update(telemetry);
    }
}