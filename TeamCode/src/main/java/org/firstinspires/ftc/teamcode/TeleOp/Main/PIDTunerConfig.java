package org.firstinspires.ftc.teamcode.TeleOp.Main;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.Sorter;

import org.firstinspires.ftc.teamcode.Subsystems.RobotConfig;

@Configurable
public class PIDTunerConfig {
    @Sorter(sort = 0)
    public static double kP = RobotConfig.SPINDEXER_KP;
    @Sorter(sort = 1)
    public static double kI = RobotConfig.SPINDEXER_KI;
    @Sorter(sort = 2)
    public static double kD = RobotConfig.SPINDEXER_KD;
    @Sorter(sort = 3)
    public static double kF = RobotConfig.SPINDEXER_KF;
    @Sorter(sort = 4)
    public static int targetTicks = 0;
    @Sorter(sort = 5)
    public static String motorName = "spinner";
    @Sorter(sort = 6)
    public static double targetAngleDeg = 0.0;
    @Sorter(sort = 7)
    public static double ticksPerRev = 384.5;
    @Sorter(sort = 8)
    public static double maxPower = RobotConfig.SPINDEXER_POWER;
    @Sorter(sort = 9)
    public static double positionTolerance = RobotConfig.SPINDEXER_POSITION_TOLERANCE;
    @Sorter(sort = 10)
    public static boolean resetEncoderOnInit = true;
    @Sorter(sort = 11)
    public static double fixedDtSeconds = RobotConfig.CLOSED_LOOP_PID_DT_SECONDS;
    @Sorter(sort = 12)
    public static boolean freestyleCustomTargetAngle = false;
    @Sorter(sort = 13)
    public static double integralMax = 1.0;
    @Sorter(sort = 14)
    public static double integralMin = -1.0;
    @Sorter(sort = 15)
    public static boolean resetIntegralOnSignChange = false;
}
