package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.teamcode.TeleOp.Main.PIDController;

public final class RobotConfig {
    private RobotConfig() {
    }
    public static final int MAIN_SENSOR_GAIN = 5;
    public static final int SECONDARY_SENSOR_GAIN = 5;
    public static final ColorTuning tuneV3  = new ColorTuning(140,160,200,250,0.3,40);
    public static final double[][][] autoAimZones = {
//        {{48,144},{99,144},{72,172}},
//        {{48,144},{99,144},{72,116}},
        {{32,76},{112,76},{72,0}},
        {{-4,290},{148,290},{72,212}},
    };
    public static final double[][][] autoAimZoneTargets = {
//            {{72,9},{72,9}},
//            {{135.5,288},{8.5,288}},
            {{72,9},{72,9}},
            {{135.5,288},{8.5,288}}
    };
    public static final int LIMELIGHT_POSITION_PIPELINE = 5;
    public static final int LIMELIGHT_MOTIF_PIPELINE = 4;
    public static final double TELEMETRY_UPDATE_INTERVAL_MS = 100;
    public static final double MOTIF_X = 72;
    public static final double MOTIF_Y = 288;
    public static final int[] DEFAULT_MOTIF = {1,2,2};
    public static final double SECONDARY_TARGET_X = 72;
    public static final double SECONDARY_TARGET_Y = 9;
    public static final PIDController spindexerPID = new PIDController(0.01,0,0.0002,0);
    public static final double SPINDEXER_POWER = 0.8;
    public static final double SPINDEXER_TICKS_PER_REV = 384.5;
    public static final double SPINDEXER_OUTTAKE_OFFSET_DEG = 186;
    public static final int SPINDEXER_SLOTS = 3;
    public static final int TRANSFER_VERIFY_DELAY = 150;
    public static final int TRANSFER_MAX_RETRIES = 2;
    public static final double SLOT_STALL_TIMEOUT_MS = 500;
    public static final double SLOT_STALL_MIN_MOVEMENT_DEG = 2.0;
    public static final int[] VERIFICATION_SENSOR_TO_SLOT = {0,1,2};
    public static final double TRANSFER_UP = 0.02;
    public static final double TRANSFER_DOWN = 0.29;
    public static final double HOOD_MAX_ANGLE = 70;
    public static final double HOOD_MIN_ANGLE = 50;
    public static final double TURRET_POWER = 0.5;
    public static final double TURRET_TICKS_PER_REV = 384.5 * (75.0 / 26.0);
    public static final double TURRET_LIMIT_LEFT = 110;
    public static final double TURRET_LIMIT_RIGHT = -110;
    public static final double TURRET_START_ANGLE = -180;
    public static final PIDController turretPID = new PIDController(0.01,0,0.0002,0);
    public static final double TARGET_Z = 0.75;
    public static final double FLYWHEEL_TICKS_PER_REV = 28;
    public static final double FLYWHEEL_RPM_DEADBAND = 10.0;
    public static final double FLYWHEEL_RADIUS_METERS = 0.048;
    public static final double FLYWHEEL_EFFICIENCY = 0.43;
    public static final double FLYWHEEL_DISABLE_RPM = 2000.0;
    public static final PIDFCoefficients FLYWHEEL_PID = new PIDFCoefficients(28,0,0,0);







    public static final String LEFT_FRONT = "lf";
    public static final String RIGHT_FRONT = "rf";
    public static final String LEFT_REAR = "lr";
    public static final String RIGHT_REAR = "rr";

    public static final String INTAKE = "intake";
    public static final String FLYWHEEL = "flywheel";
    public static final String TURRET = "tureta";
    public static final String HOOD_SERVO = "unghituretaoy";
    public static final String SPINDEXER = "spinner";
    public static final String EJECTOR = "ejector";
    public static final String COLOR_1 = "Color1";
    public static final String COLOR_2 = "Color2";
    public static final String COLOR_3 = "Color3";

    public static final String LIMELIGHT = "limelight";
    public static final String IMU = "imu";

    public static final Pose DEFAULT_START_POSE = new Pose(64.3, 15.74 / 2.0, Math.toRadians(90.0));
    public static final Pose BLUE_RESET_POSE = new Pose(27.0, 132.5, Math.toRadians(324.0));
    public static final Pose RED_RESET_POSE = new Pose(117.0, 132.5, Math.toRadians(216.0));

    public static final double BLUE_PRIMARY_TARGET_X = 0.0;
    public static final double RED_PRIMARY_TARGET_X = 144.0;
    public static final double PRIMARY_TARGET_Y = 288.0;
    public static final double MOTIF_FALLBACK_TIME = 2.0;
    public static final double AUTO_MAX_TIME = 29.0;
    public static final double AUTO_PATH_POWER = 1.0;
    public static final double AUTO_INTAKE_PATH_POWER = 0.5;

    public static final int DEFAULT_MOTIF_TAG = 21;

    public static final double INCHES_PER_METER = 100.0 / 2.54;
    public static final double LIMELIGHT_OFFSET_X_METERS = -65.5 / 1000.0;
    public static final double LIMELIGHT_OFFSET_Y_METERS = -115.0 / 1000.0;
    public static final double TURRET_OFFSET_X_METERS = 0.0;
    public static final double TURRET_OFFSET_Y_METERS = 52.0 / 1000.0;

    public static final double VISION_BLEND = 0.35;
    public static final double VISION_MAX_SPEED_INCHES = 10.0;
    public static final double VISION_MAX_JUMP_INCHES = 20.0;
    public static final double VISION_MAX_HEADING_JUMP_DEG = 20.0;

    public static final long SHOOTER_READY_HOLD_MS = 100L;

    public static final RevHubOrientationOnRobot.LogoFacingDirection IMU_LOGO =
            RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
    public static final RevHubOrientationOnRobot.UsbFacingDirection IMU_USB =
            RevHubOrientationOnRobot.UsbFacingDirection.UP;
}
