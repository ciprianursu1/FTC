package org.firstinspires.ftc.teamcode.TeleOp;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.teamcode.TeleOp.Main.Sample;

import java.util.Arrays;

class ColorTuning {
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
public class SensorV2 {
    ColorSensor sensor;
    ColorTuning tuneV2 = new ColorTuning(100,145,230,260,0,20);
    private static final int HUE_SAMPLE_SIZE = 5;
    private float[] hueSamples = new float[HUE_SAMPLE_SIZE];
    private float[] satSamples = new float[HUE_SAMPLE_SIZE];
    private int hueIndex = 0;
    private int hueCount = 0;
    public enum State {
        ACTIVE,INACTIVE
    }
    public int red;
    public int green;
    public int blue;
    public int alpha;
    public double hue = 0, sat = 0;
    private State sensorState = State.INACTIVE;
    private Sample.Colors detectedColor = Sample.Colors.NONE;
    public float[] hsv = new float[3];
    public void updateColor() {
        if(sensorState == State.INACTIVE) return;
        int argb = sensor.argb();
        alpha = (argb >> 24) & 0xFF;
        red   = ((argb >> 16) & 0xFF);
        green = ((argb >> 8)  & 0xFF);
        blue  = (argb & 0xFF);
        int max = Math.max(red, Math.max(green, blue));
        if (max == 0) max = 1;
        Color.RGBToHSV(red,green,blue,hsv);
        int min = Math.min(red, Math.min(green, blue));
        int chroma = max - min;
//        if (max < 20) {
//            detectedColor = Sample.Colors.NONE;
//            return;
//        }
//        if (chroma < 15) {
//            detectedColor = Sample.Colors.NONE;
//            return;
//        }
//        if (alpha < tuneV2.minAlpha) {
//            detectedColor = Sample.Colors.NONE;
//            return;
//        }

        hueSamples[hueIndex] = hsv[0];
        satSamples[hueIndex] = hsv[1];
        hueIndex = (hueIndex + 1) % HUE_SAMPLE_SIZE;
        if (hueCount < HUE_SAMPLE_SIZE) hueCount++;
        double hueSum = 0;
        for (int i = 0; i < hueCount; i++) {
            hueSum += hueSamples[i];
        }
        hue = hueSum / hueCount;
//        if (hue < 0) {
//            detectedColor = Sample.Colors.NONE;
//            return;
//        }


        double satSum = 0;
        for (int i = 0; i < hueCount; i++) satSum += satSamples[i];
        sat = satSum / hueCount;
        if (sat < tuneV2.minSaturation) {
            detectedColor = Sample.Colors.NONE;
            return;
        }

        if (hue > tuneV2.greenHueMin && hue < tuneV2.greenHueMax){
            detectedColor = Sample.Colors.GREEN;
            return;
        }
        if (hue > tuneV2.purpleHueMin && hue < tuneV2.purpleHueMax) {
            detectedColor = Sample.Colors.PURPLE;
            return;
        }
        detectedColor = Sample.Colors.NONE;
    }
    public void disableSensor() {
        sensorState = SensorV2.State.INACTIVE;
        if (sensor instanceof SwitchableLight) {
            ((SwitchableLight) sensor).enableLight(false);
        }
        detectedColor = Sample.Colors.NONE;
        Arrays.fill(hueSamples,0);
        Arrays.fill(satSamples,0);
        hueCount = 0;
        hueIndex = 0;
    }

    public void enableSensor() {
        sensorState = SensorV2.State.ACTIVE;
        if (sensor instanceof SwitchableLight) {
            ((SwitchableLight) sensor).enableLight(true);
        }
    }

    public Sample.Colors getDetectedColor(){
        return detectedColor;
    }
    public void setSensor(HardwareMap hwMap, String sensorName){
        sensor = hwMap.get(ColorSensor.class,sensorName);
    }
}
