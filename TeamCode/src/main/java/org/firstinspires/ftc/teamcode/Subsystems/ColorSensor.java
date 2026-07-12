package org.firstinspires.ftc.teamcode.Subsystems;

import android.graphics.Color;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ColorSensor {
    private boolean enabled = true;
    private final float[] hsv = new float[3];
    private float alpha = 0;
    private final NormalizedColorSensor colorSensor;

    public ColorSensor(NormalizedColorSensor colorSensor) {
        this.colorSensor = colorSensor;
    }
    public void setGain(int gain){
        colorSensor.setGain(gain);
    }

    public void enable(boolean enable) {
        enabled = enable;
    }

    public boolean isEnabled() {
        return enabled;
    }

    public float[] getHSV() {
        return hsv;
    }

    public float getAlpha() {
        return alpha;
    }

    public void update() {
        if (!enabled) {
            hsv[0] = 0;
            hsv[1] = 0;
            hsv[2] = 0;
            alpha = 0;
            return;
        }

        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        alpha = colors.alpha * 255F;
        Color.RGBToHSV(
                (int) (colors.red * 255F),
                (int) (colors.green * 255F),
                (int) (colors.blue * 255F),
                hsv
        );
    }
    public void appendTelemetry(Telemetry telemetry, String name) {
        telemetry.addData(name + " Enabled", enabled);
        telemetry.addData(name + " HSV/A", "%.1f / %.3f / %.3f / %.1f", hsv[0], hsv[1], hsv[2], alpha);
    }
}
