package org.firstinspires.ftc.teamcode.TeleOp.Main;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {
    private double kP, kI, kD;
    private double errorSum = 0;
    private double lastMeasurement = 0;
    private double lastError = 0;
    private double output = 0;
    private double integralMax = 1.0 , integralMin = -1.0;
    private double deadband = 1;
    ElapsedTime timer = new ElapsedTime();
    public PIDController(double p, double i, double d) {
        kP = p;
        kI = i;
        kD = d;
        errorSum = 0;
        lastMeasurement = 0;
        lastError = 0;
        timer.reset();
    }
    public double update(double target, double current){
        double dt = timer.seconds();
        if (dt > 0) {
            double error = target - current;
            if (Math.abs(error) < deadband) {
                errorSum = 0;
                timer.reset();
                lastError = error;
                return 0; // brake to hold at position
            }

            if (Math.signum(error) != Math.signum(lastError)) {
                errorSum = 0;
            }

            lastError = error;
            errorSum += error * dt;
            errorSum = Math.max(Math.min(errorSum, integralMax), integralMin);
            double derivative = (current - lastMeasurement) / dt;
            lastMeasurement = current;
            timer.reset();
            output = kP * error + kI * errorSum - kD * derivative;
            output = Math.max(Math.min(output, 1), -1);
        }
        return output;
    }
    public void setPositionTolerance(double tol){
        deadband = tol;
    }
    public void setCoefficients(double p, double i, double d){
        kP = p;
        kI = i;
        kD = d;
    }
    public void setIntegralLimits(double max, double min){
        integralMin = Math.min(min, max);
        integralMax = Math.max(min, max);
    }
    public void setkP(double kP) {
        this.kP = kP;
    }
    public void setkI(double kI) {
        this.kI = kI;
    }
    public void setkD(double kD) {
        this.kD = kD;
    }
    public double getkP() {
        return kP;
    }
    public double getkI() {
        return kI;
    }
    public double getkD() {
        return kD;
    }
    public void reset(){
        errorSum = 0;
        lastMeasurement = 0;
        lastError = 0;
        timer.reset();
    }


}
