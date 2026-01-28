package org.firstinspires.ftc.teamcode.TeleOp;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.teamcode.TeleOp.Main.Sample;

import java.util.Arrays;

public class SensorV3 {
    RevColorSensorV3 sensor;
    ColorTuning tuneV3 = new ColorTuning(140,160,200,250,0.3,40); // tuned for V3
    private static final int HUE_SAMPLE_SIZE = 5;
    private float[] hueSamples = new float[HUE_SAMPLE_SIZE];
    private float[] satSamples = new float[HUE_SAMPLE_SIZE];
    private int hueIndex = 0;
    private int hueCount = 0;
    public double hue = 0,sat = 0;

    public enum State {
        ACTIVE, INACTIVE
    }
    

    public int red;
    public int green;
    public int blue;
    public int alpha;
    private State sensorState = State.INACTIVE;
    private Sample.Colors detectedColor = Sample.Colors.NONE;
    public float[] hsv = new float[3];

    public void updateColor() {
        if (sensorState == State.INACTIVE) return;

        NormalizedRGBA colors = sensor.getNormalizedColors();
        alpha = (int)(colors.alpha * 255);
        red   = (int)(colors.red   * 255);
        green = (int)(colors.green * 255);
        blue  = (int)(colors.blue  * 255);

        Color.RGBToHSV(red, green, blue, hsv);

        // Chroma guard
        int max = Math.max(red, Math.max(green, blue));
        int min = Math.min(red, Math.min(green, blue));
        int chroma = max - min;
//        if (chroma < 15 || alpha < tuneV3.minAlpha) {
//            detectedColor = Sample.Colors.NONE;
//            return;
//        }

        // Hue + saturation averaging
        hueSamples[hueIndex] = hsv[0];
        satSamples[hueIndex] = hsv[1];
        hueIndex = (hueIndex + 1) % HUE_SAMPLE_SIZE;
        if (hueCount < HUE_SAMPLE_SIZE) hueCount++;

        double hueSum = 0;
        double satSum = 0;
        for (int i = 0; i < hueCount; i++) {
            hueSum += hueSamples[i];
            satSum += satSamples[i];
        }

         hue = hueSum / hueCount;
         sat = satSum / hueCount;

        if (sat < tuneV3.minSaturation) {
            detectedColor = Sample.Colors.NONE;
            return;
        }

        if (hue >= tuneV3.greenHueMin && hue <= tuneV3.greenHueMax) {
            detectedColor = Sample.Colors.GREEN;
            return;
        }
        if (hue >= tuneV3.purpleHueMin && hue <= tuneV3.purpleHueMax) {
            detectedColor = Sample.Colors.PURPLE;
            return;
        }

        detectedColor = Sample.Colors.NONE;
    }

    public void disableSensor() {
        sensorState = State.INACTIVE;
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
        sensorState = State.ACTIVE;
        if (sensor instanceof SwitchableLight) {
            ((SwitchableLight) sensor).enableLight(true);
        }
    }

    public State getSensorState() {
        return sensorState;
    }

    public Sample.Colors getDetectedColor() {
        return detectedColor;
    }
    public void setSensor(HardwareMap hwMap, String sensorName){
        sensor = hwMap.get(RevColorSensorV3.class,sensorName);
    }
}
