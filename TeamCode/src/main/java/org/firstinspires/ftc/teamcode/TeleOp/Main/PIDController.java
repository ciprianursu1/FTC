package org.firstinspires.ftc.teamcode.TeleOp.Main;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {
    private double kP, kI, kD, kF;
    private double errorSum;
    private double lastMeasurement;
    private double lastError;
    private double output = 0;
    private double integralMax = 1.0, integralMin = -1.0;
    private double deadband = 1.0;
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
        double dt = timer.seconds();

        if (dt < 1e-6) {
            return output;
        }

        timer.reset();
        double error = target - current;

        if (Math.abs(error) < deadband) {
            errorSum = 0;
            lastError = error;
            lastMeasurement = current;
            output = 0;
            return 0;
        }

        if (Math.signum(error) != Math.signum(lastError) && lastError != 0) {
            errorSum = 0;
        }

        double derivative = 0;
        if (!isFirstRun) {
            derivative = (current - lastMeasurement) / dt;
        } else {
            isFirstRun = false;
        }

        lastError = error;
        lastMeasurement = current;

        errorSum += error * dt;
        errorSum = Math.max(Math.min(errorSum, integralMax), integralMin);

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
    public double getIntegralMax() { return integralMax; }
    public double getIntegralMin() { return integralMin; }
    public double getDeadband() { return deadband; }

    public void reset() {
        errorSum = 0;
        lastError = 0;
        lastMeasurement = 0;
        isFirstRun = true;
        timer.reset();
    }
}
