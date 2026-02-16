package org.firstinspires.ftc.teamcode.TeleOp.Main;
import android.content.Context;
import android.content.SharedPreferences;

public class PoseStorage {

    private static final String PREF_NAME = "OdometryPose";

    private static final String KEY_X = "x";
    private static final String KEY_Y = "y";
    private static final String KEY_HEADING = "heading";
    private static final String KEY_ALLIANCE = "alliance";

    public static void savePose(Context context, double x, double y, double heading, boolean alliance) {
        SharedPreferences prefs =
                context.getSharedPreferences(PREF_NAME, Context.MODE_PRIVATE);

        prefs.edit()
                .putFloat(KEY_X, (float) x)
                .putFloat(KEY_Y, (float) y)
                .putFloat(KEY_HEADING, (float) heading)
                .putBoolean(KEY_ALLIANCE, alliance) // red / blue
                .apply();
    }

    public static double[] loadPose(Context context) {
        SharedPreferences prefs =
                context.getSharedPreferences(PREF_NAME, Context.MODE_PRIVATE);

        double x = prefs.getFloat(KEY_X, 0);
        double y = prefs.getFloat(KEY_Y, 0);
        double heading = prefs.getFloat(KEY_HEADING, 0);
        boolean alliance = prefs.getBoolean(KEY_ALLIANCE, false); // red / blue

        return new double[]{x, y, heading,alliance ? 1 : 0};
    }
}
