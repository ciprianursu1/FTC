package org.firstinspires.ftc.teamcode.TeleOp.Main;

import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.Arrays;

@TeleOp(name = "&TeleOpMainBlueClose")
public class TeleOpStefan extends LinearOpMode {

    static final long DETECT_DELAY_MS = 50;       // immediate once stable
    static final long SERVO_MOVE_LOCK_MS = 45;   // shorter lock for faster cycling
    long colorStartTimeMs = 0;
    long servoMoveStartMs = 0;
    int[] last5Sensor1 = new int[5];
    int[] last5Sensor2 = new int[5];
    int[] last5Sensor3 = new int[5];

    int indexSensor1 = 0;
    int indexSensor2 = 0;
    int indexSensor3 = 0;


    int Color1 = 0;
    int Color2 = 0;
    int Color3 = 0;
    int slotIntakeIndex = 0;
    double prev_t_outtake = 0;

    ColorSensor colorSensorSlot1;
    ColorSensor colorSensorSlot2;
    ColorSensor colorSensorSlot3;
    DcMotor intake;
    DcMotorEx turret;
    Servo ejector;
    Servo trajectoryAngleModifier;
    DcMotor front_left;
    DcMotor front_right;
    DcMotor back_left;
    DcMotor back_right;
    DcMotorEx flywheel;
    Servo spinnerClose;
    Servo spinnerFar;

    
    boolean spinIntake = false;

    Limelight3A limelight;
    IMU imu;
    private enum SpinnerState {
        INTAKE,OUTTAKE
    }
    SpinnerState spinnerState = SpinnerState.INTAKE;
    private final ElapsedTime outtakeTimeout = new ElapsedTime();
    final double ejectorDown = 0.19;
    final double ejectorUp = 0.02;
    final double[] slotPositionsIntake = {0,0.19,0.38,0.095};
    final double[] slotPositionsOuttake = {0.095, 0.285, 0.475};
    PinpointLocalizer pinpoint;
    Pose pose;
    Pose velocity;
    double PosSpinner = 0;
    double PosSpinnerMin = 0;
    double PosSpinnerMax = 0.95;
    double v0 = 0.0;
    final int minFlywheelRPM = 1000;

    static final double FLYWHEEL_TICKS_PER_REV = 28;
    private static final double MOTOR_TICKS_PER_REV = 384.5;
    private static final double MOTOR_TO_TURRET_RATIO = 75 / 26.0;

    private static final double DEG_PER_TICK_TURRET =
            360.0 / (MOTOR_TICKS_PER_REV * MOTOR_TO_TURRET_RATIO);

    // Turret soft limits (degrees)
    private static final double LEFT_LIMIT  = -110;
    private static final double RIGHT_LIMIT = 110;

    // Control
    private static final double MAX_TURRET_POWER = 0.2;
    double targetX = 12;
    double targetY = 136;
    double turretX = 0.0;
    double turretY = 0.0;
    double power = 0;
    double trajectoryAngle = 73.2;
    int flywheelTargetRPM = 0;

    /* ================= LOCALIZATION ================= */

    private Pose startPose;


    /* ================= TARGET ================= */

    private boolean aimingEnabled = false;
    private boolean launcherEnabled = false;
    final double[][] launchZoneBig = {{-18,144},{162,144},{72,55}};
    final double[][] launchZoneSmall = {{40,0},{102,0},{72,32}};
    final double absoluteTurretHeight = 0.25; //meters
    final double absoluteTargetHeight = 1.0; //meters
    final double relativeHeight = absoluteTargetHeight - absoluteTurretHeight;
    final double absoluteObstacleHeight = 1.1; //meters
    final double relativeObstacleTargetDistance = 0.465; //meters
    final double launcherEfficiency = 0.425; // needs experimenting (0.46 calculated)
    final double flywheelRadius = 0.048;
    final double g = 9.81;
    final double trajectoryAngleModifierGearRatio = 127/15.0;
    final double trajectoryAnglerMaxTravel = 300.0;
    final double trajectoryAnglePosPerDegree = trajectoryAngleModifierGearRatio/trajectoryAnglerMaxTravel;
    final double minTrajectoryAngle = 50.2;
    final double maxTrajectoryAngle = 70.0;
    final double startTurretAngle = -180.0;
    final double turretOffsetX = 0.0;
    final double turretOffsetY =  -52/1000.0;
    final int launcherStartRPM = 2500;
    final double TICKS_PER_REV_FLYWHEEL = 28;
    final int maxFlywheelRPM = 6000;
    final int telemetryDelay = 200;
    private final ElapsedTime telemetryTimer = new ElapsedTime();
    private final ElapsedTime loopTime = new ElapsedTime();
    int outtakeStep = 0;
    int[] lastNIntake = new int[5];
    int idxIntake = 0;
    boolean spinnerMoving = false;
    boolean detectionLocked = false;
    boolean colorPending = false;
    boolean waitingForClear = false;
    int lastStableIntakeColor = 0;
    int turretPos = 0;
    double error = 0;
    double targetTurretDeg = 0;
    double currentTurretDeg = 0;
    boolean enabledSorting = false;
    int motifIndex = 0;
    int nextOuttakeSlot = -1;
    int lastOuttakeSlot = -1;
    int[] OuttakeSlotColors = {0,0,0};
    int[] Motif = {1,2,1};
    int[] IntakeSlotColors = {0,0,0};
    static final int RPM_BUF = 8;
    static final double RPM_STD_MAX = 35;
    static final double RPM_ERR_MAX = 60;

    double[] rpmBuf = new double[RPM_BUF];
    int rpmIdx = 0;



    private void updateLauncher(){
        double ticksPerSecond =
                flywheelTargetRPM * TICKS_PER_REV_FLYWHEEL / 60.0;
        flywheel.setVelocity(ticksPerSecond);
    }
    private void updateTrajectoryAngle(){
        setTrajectoryAngle(trajectoryAngle);
    }
    private void updateShooter() {
        // --- 1. Robot pose and velocity in field frame ---
        double robotX = pose.getX();
        double robotY = pose.getY();
        double robotHeadingRad = pose.getHeading(); // radians
        double robotHeadingDeg = Math.toDegrees(robotHeadingRad);

        double vRobotX = velocity.getX() / 1000.0; // robot frame m/s
        double vRobotY = velocity.getY() / 1000.0;

        // Rotate robot velocity into field frame
        double fieldVelX = vRobotX * Math.cos(robotHeadingRad) - vRobotY * Math.sin(robotHeadingRad);
        double fieldVelY = vRobotX * Math.sin(robotHeadingRad) + vRobotY * Math.cos(robotHeadingRad);

        // --- 2. Turret world position ---
        turretX = robotX + turretOffsetX * Math.cos(robotHeadingRad) - turretOffsetY * Math.sin(robotHeadingRad);
        turretY = robotY + turretOffsetX * Math.sin(robotHeadingRad) + turretOffsetY * Math.cos(robotHeadingRad);

        // --- 3. Vector to target ---
        double dx = targetX - turretX;
        double dy = targetY - turretY;
        double distance = Math.hypot(dx, dy);

        if (distance < 1e-3) return; // target too close

        // --- 4. Compute trajectory angle with obstacle avoidance ---
        double d = distance * 0.0254; // meters
        double d1 = d - relativeObstacleTargetDistance;

        double k = (d*d*(absoluteObstacleHeight - absoluteTurretHeight) - d1*d1*relativeHeight)
                / (d*d1*relativeObstacleTargetDistance);
        double idealAngle = Math.toDegrees(Math.atan(k));
        boolean constrained = (idealAngle > maxTrajectoryAngle || idealAngle < minTrajectoryAngle);
        trajectoryAngle = constrained
                ? (idealAngle > maxTrajectoryAngle ? maxTrajectoryAngle : minTrajectoryAngle)
                : idealAngle;

        double thetaRad = Math.toRadians(trajectoryAngle);
        double cosTheta = Math.cos(thetaRad);
        double tanTheta = Math.tan(thetaRad);

        double denominator;
        if (!constrained) {
            denominator = (d1*tanTheta - (absoluteObstacleHeight - absoluteTurretHeight)) / (d1*d1);
        } else {
            double h1 = absoluteTurretHeight + d1*tanTheta - (d1*d1)/(d*d)*(d*tanTheta - relativeHeight);
            denominator = (d1*tanTheta - (h1 - absoluteTurretHeight)) / (d1*d1);
        }

        if (Math.abs(cosTheta) < 1e-6 || denominator <= 0) {
            flywheelTargetRPM = minFlywheelRPM;
            return;
        }

        // --- 5. Compute required world-frame launch speed ---
        double vWorld = Math.sqrt(g / (2 * cosTheta * cosTheta * denominator));

        // --- 6. Compute shot direction unit vector ---
        double shotDirX = dx / distance;
        double shotDirY = dy / distance;

        // --- 7. Estimate time-of-flight for lead ---
        double vHorizontal = vWorld * cosTheta;
        double tFlight = distance / Math.max(vHorizontal, 0.1); // seconds, prevent div0

        // --- 8. Project robot velocity along and lateral to shot ---
        double robotVelAlongShot = fieldVelX * shotDirX + fieldVelY * shotDirY;
        double robotVelLateral = fieldVelX * (-shotDirY) + fieldVelY * shotDirX;

        // --- 9. Apply lead to target vector ---
        dx -= robotVelAlongShot * tFlight;
        dy -= robotVelLateral * tFlight;

        // --- 10. Compute final turret field angle ---
        double fieldAngle = Math.toDegrees(Math.atan2(dy, dx));
        targetTurretDeg = aimingEnabled ? normalizeAngle(fieldAngle - robotHeadingDeg) : startTurretAngle;
        targetTurretDeg = Range.clip(targetTurretDeg, LEFT_LIMIT, RIGHT_LIMIT);

        // --- 11. Send turret target ---
        int targetTicks = (int) (targetTurretDeg * DEG_PER_TICK_TURRET);
        turret.setTargetPosition(targetTicks);
        turret.setPower(MAX_TURRET_POWER);

        // --- 12. Update current turret angle from encoder ---
        turretPos = turret.getCurrentPosition();
        currentTurretDeg = turretPos * DEG_PER_TICK_TURRET + startTurretAngle;

        // --- 13. Compensate flywheel speed for robot motion along shot ---
        double turretYawFieldRad = robotHeadingRad + Math.toRadians(targetTurretDeg);
        double shotDirCompX = Math.cos(turretYawFieldRad);
        double shotDirCompY = Math.sin(turretYawFieldRad);
        double robotVelAlongShotComp = fieldVelX * shotDirCompX + fieldVelY * shotDirCompY;

        v0 = vWorld - robotVelAlongShotComp;
        if (v0 <= 0) {
            flywheelTargetRPM = minFlywheelRPM;
        } else {
            flywheelTargetRPM = (int) (60 * v0 / (2 * Math.PI * flywheelRadius * launcherEfficiency));
            flywheelTargetRPM = Math.max(minFlywheelRPM, Math.min(flywheelTargetRPM, maxFlywheelRPM));
        }

        // --- 14. Update trajectory angle servo ---
        setTrajectoryAngle(trajectoryAngle);
        // --- 15. Update flywheel ---
        updateLauncher();
    }

    private void setTrajectoryAngle(double angle){
        angle = Range.clip(angle,minTrajectoryAngle,maxTrajectoryAngle);
        double position = (angle - minTrajectoryAngle)*trajectoryAnglePosPerDegree;
        trajectoryAngleModifier.setPosition(position);
    }

    public void disableIfNotInLaunchZone(){
        double robotX = pose.getX();
        double robotY = pose.getY();
        double d1,d2,d3;
        boolean has_neg, has_pos;
        if(robotY > 48) {
            d1 = sign(robotX, robotY, launchZoneBig[0][0], launchZoneBig[0][1], launchZoneBig[1][0], launchZoneBig[1][1]);
            d2 = sign(robotX, robotY, launchZoneBig[1][0], launchZoneBig[1][1], launchZoneBig[2][0], launchZoneBig[2][1]);
            d3 = sign(robotX, robotY, launchZoneBig[2][0], launchZoneBig[2][1], launchZoneBig[0][0], launchZoneBig[0][1]);
        }else{
            d1 = sign(robotX, robotY, launchZoneSmall[0][0], launchZoneSmall[0][1], launchZoneSmall[1][0], launchZoneSmall[1][1]);
            d2 = sign(robotX, robotY, launchZoneSmall[1][0], launchZoneSmall[1][1], launchZoneSmall[2][0], launchZoneSmall[2][1]);
            d3 = sign(robotX, robotY, launchZoneSmall[2][0], launchZoneSmall[2][1], launchZoneSmall[0][0], launchZoneSmall[0][1]);
        }
        has_neg = d1 < 0 || d2 < 0 || d3 < 0;
        has_pos = d1 > 0 || d2 > 0 || d3 > 0;
        if (has_neg && has_pos){
            disableAiming();
        } else {
            if(!aimingEnabled) enableAiming();
        }
    }
    private double sign(double x1,double y1,double x2,double y2,double x3,double y3){
        return (x1-x3)*(y2-y3)-(x2-x3)*(y1-y3);
    }

    public void enableLauncher(){
        if(launcherEnabled) return;
        launcherEnabled = true;
        flywheelTargetRPM = launcherStartRPM;
        updateLauncher();
    }
    public void disableLauncher(){
        if(!launcherEnabled) return;
        launcherEnabled = false;
        flywheelTargetRPM = 0;
        updateLauncher();
    }
    public void enableAiming(){
        aimingEnabled = true;
    }
    public void disableAiming(){
        aimingEnabled = false;
    }
    private void initLocalization() {
        pinpoint = new PinpointLocalizer(hardwareMap, Constants.localizerConstants);
        startPose = new Pose(22, 127, Math.toRadians(-36));
        pinpoint.setStartPose(startPose);
    }
    private void mapIntakeToOuttakeSlots() {
        for (int i = 0; i < 3; i++) {
            OuttakeSlotColors[2 - i] = IntakeSlotColors[i];
        }
    }
    private void selectNextOuttakeSlot() {
        nextOuttakeSlot = -1;
        if(enabledSorting) {
            for (int i = 0; i < 3; i++) {
                if (OuttakeSlotColors[i] == Motif[motifIndex]) {
                    nextOuttakeSlot = i;
                    break;
                }
            }
            motifIndex = (motifIndex + 1) % Motif.length;
        }

        // If no match, just take the next available ball
        if (nextOuttakeSlot == -1) {
            for (int i = 0; i < 3; i++) {
                if (OuttakeSlotColors[i] != 0) {
                    nextOuttakeSlot = i;
                    break;
                }
            }
        }
    }


    private void InitWheels() {
        front_left = hardwareMap.dcMotor.get("lf");
        front_right = hardwareMap.dcMotor.get("rf");
        back_left = hardwareMap.dcMotor.get("lr");
        back_right = hardwareMap.dcMotor.get("rr");

        front_right.setDirection(DcMotorSimple.Direction.FORWARD);
        back_right.setDirection(DcMotorSimple.Direction.FORWARD);
        front_left.setDirection(DcMotorSimple.Direction.REVERSE);
        back_left.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    private void InitDc() {

        intake = hardwareMap.get(DcMotor.class, "intake");
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        turret = hardwareMap.get(DcMotorEx.class, "tureta");
        PIDFCoefficients pidfCoefficientsTurret = new PIDFCoefficients(5,2,2,4);
        turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        turret.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficientsTurret);
        PIDFCoefficients pidfCoefficientsFlywheel = new PIDFCoefficients(20, 5, 12, 12);
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficientsFlywheel);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }
    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    private void InitServo() {
        ejector = hardwareMap.get(Servo.class, "ejector");
        spinnerFar = hardwareMap.get(Servo.class, "SpinnerFar");
        spinnerClose = hardwareMap.get(Servo.class, "SpinnerClose");
        trajectoryAngleModifier = hardwareMap.get(Servo.class, "unghituretaoy");
        ejector.setDirection(Servo.Direction.REVERSE);
        ejector.setPosition(ejectorDown);
        spinnerFar.setPosition(0);
        spinnerClose.setPosition(0);
    }

    private void InitLL() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(5);
        limelight.start();

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
        );
        imu.initialize(new IMU.Parameters(orientation));
    }

    private void InitAux() {
        colorSensorSlot1 = hardwareMap.colorSensor.get("Color1");
        colorSensorSlot2 = hardwareMap.colorSensor.get("Color2");
        colorSensorSlot3 = hardwareMap.colorSensor.get("Color3");
    }


    private void SetWheelsPower() {
        double left_x = gamepad1.left_stick_x;
        double left_y = -gamepad1.left_stick_y; // forward is negative
        double right_x = gamepad1.right_stick_x;

        double front_left_pw = left_y + left_x + right_x;
        double back_left_pw = left_y - left_x + right_x;
        double front_right_pw = left_y - left_x - right_x;
        double back_right_pw = left_y + left_x - right_x;

        // Normalize so no motor power exceeds 1.0
        double max = Math.max(Math.abs(front_left_pw),
                Math.max(Math.abs(back_left_pw),
                        Math.max(Math.abs(front_right_pw), Math.abs(back_right_pw))));
        if (max > 1.0) {
            front_left_pw /= max;
            back_left_pw /= max;
            front_right_pw /= max;
            back_right_pw /= max;
        }

        front_left.setPower(front_left_pw);
        back_left.setPower(back_left_pw);
        front_right.setPower(front_right_pw);
        back_right.setPower(back_right_pw);
    }

    private double getHue(int r, int g, int b) {
        int max = Math.max(r, Math.max(g, b));
        int min = Math.min(r, Math.min(g, b));
        if (max == min) return 0.0;

        double chroma = max - min;
        double h;

        if (max == r) h = (double) (g - b) / chroma;
        else if (max == g) h = (double) (b - r) / chroma + 2.0;
        else h = (double) (r - g) / chroma + 4.0;

        h *= 60.0;
        if (h < 0) h += 360.0;

        return h;
    }


    private int colorDetection(ColorSensor sensor) {
        int r = sensor.red();
        int g = sensor.green();
        int b = sensor.blue();
        int alpha = sensor.alpha();

        // Skip if the ball is too dim (likely floor reflection or empty slot)
        if (alpha < 50) return 0;

        // Convert RGB to Hue
        double hue = getHue(r, g, b);

        // Green ball range (tuned for FTC field lighting)
        if (hue >= 130 && hue <= 170) return 1;

        // Purple ball range
        if (hue >= 200 && hue <= 230) return 2;

        // Default: no ball
        return 0;
    }

private int processSensorWithLock(ColorSensor sensor, int[] last5, int index, int lastStable, boolean waitingForClearFlag) {
    if (spinnerMoving) return lastStable; // block all updates while spinner is moving
    if (waitingForClearFlag) {
        int colorNow = colorDetection(sensor);
        if (colorNow == 0) { // cleared
            return 0;
        } else {
            return lastStable; // keep waiting
        }
    }
    last5[index] = colorDetection(sensor);
    int count1 = 0, count2 = 0;
    for (int v : last5) {
        if (v == 1) count1++;
        else if (v == 2) count2++;
    }
    if (count1 >= 3) return 1;
    if (count2 >= 3) return 2;
    return 0;
}


    private void updateColors() {
        Color1 = processSensorWithLock(colorSensorSlot1, last5Sensor1, indexSensor1, Color1, waitingForClear);
        indexSensor1 = (indexSensor1 + 1) % 5;

        Color2 = processSensorWithLock(colorSensorSlot2, last5Sensor2, indexSensor2, Color2, spinnerMoving);
        indexSensor2 = (indexSensor2 + 1) % 5;

        Color3 = processSensorWithLock(colorSensorSlot3, last5Sensor3, indexSensor3, Color3, spinnerMoving);
        indexSensor3 = (indexSensor3 + 1) % 5;

        IntakeSlotColors[0] = Color1;
        IntakeSlotColors[1] = Color2;
        IntakeSlotColors[2] = Color3;
    }


    private void servoLogic() {
        if (gamepad1.dpadRightWasPressed()) {
            slotIntakeIndex++;
            slotIntakeIndex = slotIntakeIndex % 3;
            PosSpinner = slotPositionsIntake[slotIntakeIndex];
        }
        if (gamepad1.dpadLeftWasPressed()){
            slotIntakeIndex--;
            slotIntakeIndex = (slotIntakeIndex + 3) % 3;
            PosSpinner = slotPositionsIntake[slotIntakeIndex];
        }
    }

    private boolean spinnerFull() {
        return (Color1 != 0 && Color2 != 0 && Color3 != 0) || slotIntakeIndex == 3;
    }
    private boolean flywheelReady() {
        rpmBuf[rpmIdx] = getFlywheelRPM();
        rpmIdx = (rpmIdx + 1) % RPM_BUF;

        double mean = 0;
        for (double r : rpmBuf) mean += r;
        mean /= RPM_BUF;

        double var = 0;
        for (double r : rpmBuf) var += (r - mean) * (r - mean);
        var /= RPM_BUF;

        return Math.sqrt(var) < RPM_STD_MAX &&
                Math.abs(mean - flywheelTargetRPM) < RPM_ERR_MAX;
    }

    private double getFlywheelRPM() {
        return flywheel.getVelocity() / FLYWHEEL_TICKS_PER_REV * 60.0;
    }

    private int processIntakeSensor(ColorSensor sensor) {
        int detected = colorDetection(sensor);

        lastNIntake[idxIntake] = detected;
        idxIntake = (idxIntake + 1) % lastNIntake.length;

        int count1 = 0, count2 = 0;
        for (int v : lastNIntake) {
            if (v == 1) count1++;
            else if (v == 2) count2++;
        }

        int finalColor = 0;
        if (count1 >= 2 && count1 > count2) finalColor = 1;
        else if (count2 >= 2 && count2 > count1) finalColor = 2;

        return finalColor;
    }

    private void colorDrivenSpinnerLogicServos() {

        // Servo movement lock window (shorter for faster cycling)
        if(spinnerFull()) return;
        if (spinnerMoving) {
            if (System.currentTimeMillis() - servoMoveStartMs >= SERVO_MOVE_LOCK_MS) {
                spinnerMoving = false;
                detectionLocked = false;
                lastStableIntakeColor = 0;
            } else {
                return;
            }
        }

        // Must clear before we allow another ball
        if (waitingForClear) {
            int intakeColorNow = processIntakeSensor(colorSensorSlot1);
            if (intakeColorNow == 0) {
                waitingForClear = false;
                lastStableIntakeColor = 0;
            }
            return;
        }

        // Intake-facing sensor (change if needed)
        int intakeColor = processIntakeSensor(colorSensorSlot1);

        // FAST trigger: first nonzero after clear
        boolean newColorDetected = (intakeColor != 0 && lastStableIntakeColor == 0);
        if (newColorDetected) {
            lastStableIntakeColor = intakeColor;
        }

        if (newColorDetected && !colorPending && !detectionLocked) {
            colorStartTimeMs = System.currentTimeMillis();
            colorPending = true;
        }

        if (colorPending && (System.currentTimeMillis() - colorStartTimeMs >= DETECT_DELAY_MS)) {

            Color1 = intakeColor;
            slotIntakeIndex++;
            if(slotIntakeIndex == 3) return;
            slotIntakeIndex = slotIntakeIndex % 3;
            PosSpinner = slotPositionsIntake[slotIntakeIndex];

            waitingForClear = true;
            detectionLocked = true;
            spinnerMoving = true;
            servoMoveStartMs = System.currentTimeMillis();

            colorPending = false;
        }
    }

    private void resetLocalization(){
        pinpoint.setPose(startPose);
    }

        private void updateTelemetry() {
        if(telemetryTimer.milliseconds() >= telemetryDelay) {
            if (spinnerState == SpinnerState.OUTTAKE) {
                telemetry.addData("Slot 1", OuttakeSlotColors[0]);
                telemetry.addData("Slot 2", OuttakeSlotColors[1]);
                telemetry.addData("Slot 3", OuttakeSlotColors[2]);
            }
            if (spinnerState == SpinnerState.INTAKE) {
                telemetry.addData("Slot 1", IntakeSlotColors[0]);
                telemetry.addData("Slot 2", IntakeSlotColors[1]);
                telemetry.addData("Slot 3", IntakeSlotColors[2]);
            }

            telemetry.addData("time_outtake", outtakeTimeout.time());
            telemetry.addData("PosSpinner", PosSpinner);

            telemetry.addData("Sensor 1a", colorSensorSlot1.alpha());
            telemetry.addData("Sensor 2a", colorSensorSlot2.alpha());
            telemetry.addData("Sensor 3a", colorSensorSlot3.alpha());
            telemetry.addData("Sensor 1", getHue(colorSensorSlot1.red(), colorSensorSlot1.green(), colorSensorSlot1.blue()));
            telemetry.addData("Sensor 2", getHue(colorSensorSlot2.red(), colorSensorSlot2.green(), colorSensorSlot2.blue()));
            telemetry.addData("Sensor 3", getHue(colorSensorSlot3.red(), colorSensorSlot3.green(), colorSensorSlot3.blue()));
            telemetry.addData("Target (raw)", "%.1f", targetTurretDeg);
            telemetry.addData("Error", "%.1f", error);
            telemetry.addData("Power", "%.2f", power);
            telemetry.addData("Flywheel Target RPM", flywheelTargetRPM);
            telemetry.addData("Flywheel RPM", getFlywheelRPM());
            telemetry.addData("Heading", "%.1f",
                    Math.toDegrees(pose.getHeading()));
            telemetry.addData("trajectoryAngle",trajectoryAngle);
            telemetry.addData("loopTime", loopTime.milliseconds());
            telemetry.addData("turretTicks",turretPos);
            telemetry.addData("Turret", "%.1f", currentTurretDeg);
            telemetry.update();
            telemetryTimer.reset();
        }
    }

    private void runIntake(){
            updateColors();
            colorDrivenSpinnerLogicServos();
        
    }
    private void runOuttake() {
        spinIntake = true;
        final int EJECTOR_UP_DELAY = 250;
        final int EJECTOR_DOWN_DELAY = 200;
        final int SPINNER_SLOT_CHANGE_DELAY = 300;
        mapIntakeToOuttakeSlots();
        double t = outtakeTimeout.milliseconds();
        if(t >= prev_t_outtake){
            switch (outtakeStep%3) {
                case 0:
                    selectNextOuttakeSlot();
                    if(nextOuttakeSlot == -1) {
                        outtakeStep = 0;
                        prev_t_outtake = 0;
                        PosSpinner = slotPositionsIntake[0];
                        spinnerState = SpinnerState.INTAKE;
                        spinIntake = true;
                        slotIntakeIndex = 0;
                        waitingForClear = true;
                        detectionLocked = false;
                        spinnerMoving = true;
                        colorPending = false;
                        lastStableIntakeColor = 0;
                        Arrays.fill(lastNIntake, 0);
                        idxIntake = 0;
                        return;
                    }
                    PosSpinner = slotPositionsOuttake[nextOuttakeSlot];
                    prev_t_outtake = t + SPINNER_SLOT_CHANGE_DELAY * Math.abs(lastOuttakeSlot - nextOuttakeSlot);
                    lastOuttakeSlot = nextOuttakeSlot;
                    break;
                case 1:
                    if(!flywheelReady()) return;
                    ejector.setPosition(ejectorUp);
                    prev_t_outtake = t + EJECTOR_UP_DELAY;
                    break;
                case 2:
                    ejector.setPosition(ejectorDown);
                    prev_t_outtake = t + EJECTOR_DOWN_DELAY;
                    break;
            }
            outtakeStep++;
        }
    }
    private void startIntakeMode(){
        spinnerState = SpinnerState.INTAKE;
        spinIntake = true;
        slotIntakeIndex = 0;
        PosSpinner = slotPositionsIntake[0];
        waitingForClear = true;
        detectionLocked = false;
        spinnerMoving = true;
        colorPending = false;
        lastStableIntakeColor = 0;
        Arrays.fill(lastNIntake, 0);
        idxIntake = 0;
    }



    @Override
    public void runOpMode () {
        InitWheels();
        InitAux();
        InitDc();
        InitLL();
        InitServo();
        initLocalization();

        waitForStart();
        loopTime.reset();
        while (opModeIsActive()) {
            if (PosSpinner >= PosSpinnerMin && PosSpinner <= PosSpinnerMax) {
                spinnerFar.setPosition(PosSpinner);
                spinnerClose.setPosition(PosSpinner);
            }

            servoLogic();
            SetWheelsPower();
            pinpoint.update();
            pose = pinpoint.getPose();
            velocity = pinpoint.getVelocity();
            disableIfNotInLaunchZone();
            updateTelemetry();
            if(aimingEnabled){
                updateShooter();
                updateTrajectoryAngle();
            }
            if(spinnerFull() && !aimingEnabled){
                enableLauncher();
            } else {
                if(!spinnerFull() && launcherEnabled && !aimingEnabled) disableLauncher();
            }
            if (gamepad1.crossWasPressed()  && !gamepad1.crossWasReleased()){
                intake.setPower(-1);
            } else {
                intake.setPower(spinIntake ? 1:0);
            }
            if(gamepad1.touchpadWasPressed()){
                enabledSorting = !enabledSorting;
                gamepad1.rumble(enabledSorting ? 200 : 500);
            }
            if(gamepad1.optionsWasPressed() && gamepad1.shareWasPressed()){
                resetLocalization();
                gamepad1.rumble(200);
            }
            if (gamepad1.circleWasPressed()) {
                startIntakeMode();
            }
            if (gamepad1.right_trigger > 0.8) {
                if(spinnerState == SpinnerState.INTAKE) {
                    Arrays.fill(rpmBuf,getFlywheelRPM());
                    outtakeTimeout.reset();
                    outtakeStep = 0;
                    lastOuttakeSlot = -1;
                    gamepad1.rumble(150);
                }
                spinnerState = SpinnerState.OUTTAKE;
            }

            if (gamepad1.yWasPressed()) {
                spinIntake = !spinIntake;
            }

            if (spinnerState == SpinnerState.INTAKE) {
                runIntake();
            } else {
                runOuttake();
            }
            loopTime.reset();
        }

    }
}