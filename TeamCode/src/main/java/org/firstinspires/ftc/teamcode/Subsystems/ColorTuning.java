package org.firstinspires.ftc.teamcode.Subsystems;

public class ColorTuning {
    public double minSaturation;
    public int minAlpha;
    public int greenHueMin, greenHueMax;
    public int purpleHueMin, purpleHueMax;
    public ColorTuning(int greenHueMin, int greenHueMax,
                       int purpleHueMin, int purpleHueMax,
                       double minSaturation, int minAlpha) {
        // Set the class variables
        this.minAlpha = minAlpha;
        this.greenHueMin = greenHueMin;
        this.greenHueMax = greenHueMax;
        this.purpleHueMin = purpleHueMin;
        this.purpleHueMax = purpleHueMax;
        this.minSaturation = minSaturation;
    }
}
