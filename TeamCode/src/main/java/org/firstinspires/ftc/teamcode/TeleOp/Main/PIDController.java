package org.firstinspires.ftc.teamcode.TeleOp.Main;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {
    private double kP, kI, kD, kF;
    private double errorSum;
    private double lastMeasurement;
    private double lastError;
    private double output = 0;
    private double lastDt = 0;
    private double lastMeasuredDt = 0;
    private double fixedDt = 0;
    private double integralMax = 1.0, integralMin = -1.0;
    private double deadband = 1.0;
    private double lastIntegralDelta = 0;
    private int resetCount = 0;
    private String lastResetReason = "none";
    private boolean resetIntegralOnSignChange = false;
    private String lastIntegralState = "reset";
    private boolean isFirstRun = true;
    private final ElapsedTime timer = new ElapsedTime();

    public PIDController(double p, double i, double d, double f) {
        kP = p;
        kI = i;
        kD = d;
        kF = f;
        reset();
    }

    public double update(double target, double current) {
        double measuredDt = timer.seconds();

        if (measuredDt < 1e-6) {
            return output;
        }

        timer.reset();
        lastMeasuredDt = measuredDt;
        double dt = fixedDt > 0 ? fixedDt : measuredDt;
        lastDt = dt;
        double error = target - current;

        if (Math.abs(error) < deadband) {
            lastIntegralDelta = -errorSum;
            errorSum = 0;
            lastIntegralState = "deadband reset";
            lastError = error;
            lastMeasurement = current;
            output = 0;
            return 0;
        }

        if (resetIntegralOnSignChange && Math.signum(error) != Math.signum(lastError) && lastError != 0) {
            errorSum = 0;
            lastIntegralState = "sign reset";
        }

        double derivative = 0;
        if (!isFirstRun) {
            derivative = (current - lastMeasurement) / dt;
        } else {
            isFirstRun = false;
        }

        lastError = error;
        lastMeasurement = current;

        double unclampedErrorSum = errorSum + error * dt;
        errorSum = Math.max(Math.min(unclampedErrorSum, integralMax), integralMin);
        lastIntegralDelta = errorSum - (unclampedErrorSum - error * dt);
        if(errorSum != unclampedErrorSum) {
            lastIntegralState = "clamped";
        } else if(!"sign reset".equals(lastIntegralState)) {
            lastIntegralState = "accumulating";
        }

        output = (kP * error) + (kI * errorSum) - (kD * derivative) + (kF * Math.signum(error));
        output = Math.max(Math.min(output, 1.0), -1.0);

        return output;
    }

    public void setPositionTolerance(double tol) {
        deadband = tol;
    }

    public void setCoefficients(double p, double i, double d, double f) {
        kP = p;
        kI = i;
        kD = d;
        kF = f;
    }

    public void setIntegralLimits(double max, double min) {
        integralMin = Math.min(min, max);
        integralMax = Math.max(min, max);
    }
    public void setFixedDt(double fixedDt) {
        this.fixedDt = Math.max(fixedDt, 0);
    }
    public void setResetIntegralOnSignChange(boolean resetIntegralOnSignChange) {
        this.resetIntegralOnSignChange = resetIntegralOnSignChange;
    }
    public boolean isOnTarget() {
        return Math.abs(lastError) < deadband;
    }
    public void setkP(double kP) { this.kP = kP; }
    public void setkI(double kI) { this.kI = kI; }
    public void setkD(double kD) { this.kD = kD; }
    public void setkF(double kF) { this.kF = kF; }

    public double getkP() { return kP; }
    public double getkI() { return kI; }
    public double getkD() { return kD; }
    public double getkF() { return kF; }
    public double getErrorSum() { return errorSum; }
    public double getLastMeasurement() { return lastMeasurement; }
    public double getLastError() { return lastError; }
    public double getOutput() { return output; }
    public double getLastDt() { return lastDt; }
    public double getLastMeasuredDt() { return lastMeasuredDt; }
    public double getFixedDt() { return fixedDt; }
    public double getLastIntegralDelta() { return lastIntegralDelta; }
    public int getResetCount() { return resetCount; }
    public String getLastResetReason() { return lastResetReason; }
    public String getLastIntegralState() { return lastIntegralState; }
    public boolean getResetIntegralOnSignChange() { return resetIntegralOnSignChange; }
    public double getIntegralMax() { return integralMax; }
    public double getIntegralMin() { return integralMin; }
    public double getDeadband() { return deadband; }

    public void reset() {
        reset("manual");
    }
    public void reset(String reason) {
        errorSum = 0;
        lastError = 0;
        lastMeasurement = 0;
        output = 0;
        lastDt = 0;
        lastMeasuredDt = 0;
        lastIntegralDelta = 0;
        resetCount++;
        lastResetReason = reason;
        lastIntegralState = "reset";
        isFirstRun = true;
        timer.reset();
    }
}
