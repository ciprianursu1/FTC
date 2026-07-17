package org.firstinspires.ftc.teamcode.TeleOp.Main;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "PID Tuner (Panels)", group = "TEST")
public class PanelsPIDTuner extends OpMode {
    private DcMotorEx motor;
    private PIDController pid;
    private TelemetryManager telemetryM;
    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorEx.class, "spinner");
        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        pid = new PIDController(
                PIDTunerConfig.kP,
                PIDTunerConfig.kI,
                PIDTunerConfig.kD,
                PIDTunerConfig.kF
        );
        pid.setFixedDt(PIDTunerConfig.fixedDtSeconds);
        pid.setIntegralLimits(PIDTunerConfig.integralMax, PIDTunerConfig.integralMin);
        pid.setResetIntegralOnSignChange(PIDTunerConfig.resetIntegralOnSignChange);
    }

    @Override
    public void loop() {
        pid.setCoefficients(
                PIDTunerConfig.kP,
                PIDTunerConfig.kI,
                PIDTunerConfig.kD,
                PIDTunerConfig.kF
        );
        pid.setFixedDt(PIDTunerConfig.fixedDtSeconds);
        pid.setIntegralLimits(PIDTunerConfig.integralMax, PIDTunerConfig.integralMin);
        pid.setResetIntegralOnSignChange(PIDTunerConfig.resetIntegralOnSignChange);
        int currentTicks = motor.getCurrentPosition();
        double power = pid.update(
                PIDTunerConfig.targetTicks,
                currentTicks
        );
        motor.setPower(power);
        telemetryM.addData("Position", currentTicks);
        telemetryM.addData("Power", power);
        telemetryM.addData("Target", PIDTunerConfig.targetTicks);
        telemetryM.addData("Fixed Dt", pid.getFixedDt());
        telemetryM.addData("Real Dt", pid.getLastMeasuredDt());
        telemetryM.addData("Integral", pid.getErrorSum());
        telemetryM.addData("Integral Delta", pid.getLastIntegralDelta());
        telemetryM.addData("Integral State", pid.getLastIntegralState());
        telemetryM.addData("Reset Count", pid.getResetCount());
        telemetryM.addData("Reset Reason", pid.getLastResetReason());
        telemetryM.update(telemetry);
    }
}
