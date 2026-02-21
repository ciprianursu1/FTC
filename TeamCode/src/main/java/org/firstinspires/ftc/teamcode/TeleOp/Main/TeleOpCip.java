package org.firstinspires.ftc.teamcode.TeleOp.Main;

import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp (name = "Cip", group = "Alpha")
public class TeleOpCip extends OpMode {
    DCSpindexer spinner;
    DcMotorEx turret;
    DcMotorEx flywheel;
    DcMotorSimple intake;
    Servo trajectoryAngleModifier;
    DcMotor front_left;
    DcMotor front_right;
    DcMotor back_left;
    DcMotor back_right;
    Limelight3A limelight;
    PinpointLocalizer pinpoint;
    Pose startPose;
    Pose pose;
    IMU imu;
    static final double INCH_PER_METER = 100/2.54;
    static final double FLYWHEEL_TICKS_PER_REV = 28*1.4;
    double targetX = 0;
    double targetY = 144;
    static final double minFlywheelRPM = 1000;
    static final double maxFlywheelRPM = 6000;
    static final double maxTrajectoryAngle = 70;
    static final double minTrajectoryAngle = 50.2;
    final double trajectoryAngleModifierGearRatio = 127/15.0;
    final double trajectoryAnglerMaxTravel = 300.0;
    final double trajectoryAnglePosPerDegree = trajectoryAngleModifierGearRatio/trajectoryAnglerMaxTravel;
    static final double g = 9.81;
    static final double flywheelRadius = 0.048;
    static final double launcherEfficiency = 0.43;
    static final double turretOffsetX = 0.0;
    static final double turretOffsetY = 52/1000.0;
    final double[][] launchZoneBig = {{-18,144},{162,144},{72,55}};
    final double[][] launchZoneSmall = {{40,0},{102,0},{72,32}};
    private boolean aimingEnabled = false;
    private boolean launcherEnabled = false;
    final double absoluteTurretHeight = 0.25; //meters
    final double absoluteTargetHeight = 0.9; //meters
    final double relativeHeight = absoluteTargetHeight - absoluteTurretHeight;
    final double preferredMaxHeightThrow = relativeHeight + 0.3; //meters (relative to turret height)
    final double kP = 0.015;
    final double MAX_POWER_TURRET = 0.4;
    final double startTurretAngle = -180.0;
    final double LEFT_LIMIT = -110;
    final double RIGHT_LIMIT = 110;
    private static final double MOTOR_TICKS_PER_REV = 384.5;
    private static final double MOTOR_TO_TURRET_RATIO = 75.0 / 26.0;

    private static final double TURRET_DEG_PER_TICK =
            360.0 / (MOTOR_TICKS_PER_REV * MOTOR_TO_TURRET_RATIO);

    double turretX = 0.0;
    double turretY = 0.0;
    double flywheelTargetRPM = 0;
    double rpm = 0.0;
    double trajectoryAngle = 70;
    private static final double RPM_TOL = 50;
    private static final long RPM_STABLE_MS = 150;
    private static final double TURRET_TOL_DEG = 2.0;
    private static final double LL_OFFSET_X = -65.5/1000;
    private static final double LL_OFFSET_Y = 181/1000.0;
    private long rpmInRangeSinceMs = 0;
    boolean turretOnTarget = false;
    boolean blue = true;
    public static double kP_v = 30.0;     // try 20–35
    public static double kI_v = 0.0;      // keep 0 for fastest transient
    public static double kD_v = 0.0;      // add small later only if it overshoots/oscillates
    // Correct ballpark for 6000rpm Yellow Jacket (28tpr): ~11.7 at 12V no-load
    public static double kF_v = 14.0;
    enum IntakeState{
        ON,OFF,REVERSE
    }
    int lastTag = 0;
    IntakeState intakeState = IntakeState.OFF;
    IntakeState prevIntakeState = IntakeState.OFF;
    boolean limelightCorrectionMode = false;


    public void init() {

        turret = hardwareMap.get(DcMotorEx.class, "tureta");
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(14, 2.3, 4, 14.45);
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        trajectoryAngleModifier = hardwareMap.get(Servo.class, "unghituretaoy");
        spinner = new DCSpindexer(hardwareMap,"Color1","Color2","Color3","spinner","ejector",telemetry);
        spinner.init();
        intake = hardwareMap.get(DcMotorSimple.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        front_left = hardwareMap.dcMotor.get("lf");
        front_right = hardwareMap.dcMotor.get("rf");
        back_left = hardwareMap.dcMotor.get("lr");
        back_right = hardwareMap.dcMotor.get("rr");
        pinpoint = new PinpointLocalizer(hardwareMap, Constants.localizerConstants);
        startPose = new Pose(64.3, 15.74/2.0, Math.toRadians(90));
        pinpoint.setPose(startPose);
        double[] pose = PoseStorage.loadPose(hardwareMap.appContext);
        double x = pose[0];
        double y = pose[1];
        double heading = pose[2];
        blue = pose[3] == 1;
        if(blue){
            targetX = 0;
            targetY = 144;
        } else {
            targetX = 144;
            targetY = 144;
        }
        pinpoint.setPose(new Pose(x,y,heading));

        front_right.setDirection(DcMotorSimple.Direction.FORWARD);
        back_right.setDirection(DcMotorSimple.Direction.FORWARD);
        front_left.setDirection(DcMotorSimple.Direction.REVERSE);
        back_left.setDirection(DcMotorSimple.Direction.REVERSE);
        spinner.setMotif(21);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        );
        imu.initialize(new IMU.Parameters(orientation));
        imu.resetYaw();
        limelight.pipelineSwitch(4);
        limelight.start();
    }
    public void init_loop() {
        LLResult result = limelight.getLatestResult();
        if (result.isValid() && result.getFiducialResults() != null && !result.getFiducialResults().isEmpty()) {
            int tagID = result.getFiducialResults().get(0).getFiducialId();
            if ((tagID == 21 || tagID == 22 || tagID == 23) && tagID != lastTag) {
                spinner.setMotif(tagID);
                lastTag = tagID;
                telemetry.addLine("Tag " + tagID + " detected");
            }
        } else {
            telemetry.addLine("No fiducial detected or Limelight not connected");
        }
    }
    public void start(){
        limelight.pipelineSwitch(5);
    }
    public void loop(){
        pinpoint.update();
        YawPitchRollAngles ypr = imu.getRobotYawPitchRollAngles();
        pinpoint.setHeading(ypr.getYaw(AngleUnit.RADIANS) + startPose.getHeading());
        pose = pinpoint.getPose();
        velocity = pinpoint.getVelocity();
        if(limelightCorrectionMode) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.getBotpose_MT2() != null) {
                Pose3D LLPose = result.getBotpose_MT2();
                Pose pedroPose = PoseConverter.pose2DToPose(new Pose2D(DistanceUnit.INCH, LLPose.getPosition().x * INCH_PER_METER, LLPose.getPosition().y * INCH_PER_METER, AngleUnit.DEGREES, LLPose.getOrientation().getYaw()), InvertedFTCCoordinates.INSTANCE);
                pedroPose = processPedroPose(pedroPose);
                pinpoint.setPose(pedroPose);
            }
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            limelight.updateRobotOrientation(normalizeAngle(orientation.getYaw() + Math.toDegrees(startPose.getHeading())));
            pinpoint.setHeading(normalizeAngle(orientation.getYaw(AngleUnit.RADIANS) - startPose.getHeading())); // magnetometru rev
        }
        Drive();
        if(aimingEnabled || spinner.spindexerFull()) {
            enableLauncher();
            if(aimingEnabled) computeParameters();
        } else {
            disableLauncher();
        }
        updateFlywheel();
        updateTrajectoryAngle();
        updateTurretAim();
        switch (intakeState){
            case ON:
                intake.setPower(1);
                break;
            case OFF:
                intake.setPower(0);
                break;
            case REVERSE:
                intake.setPower(-1);
                break;
        }
        if(spinner.requestingOuttake){
            spinner.setReady(rpmInRangeStable() && turretOnTarget);
        }
        spinner.update();
        if(gamepad2.circleWasPressed()){
            spinner.cancelOuttake();
            if(intakeState == IntakeState.ON){
                intakeState = IntakeState.OFF;
            } else {
                intakeState = IntakeState.ON;
            }
        }
        if(gamepad2.right_trigger > 0.8){
            spinner.requestOuttake();
        }
        if (gamepad2.cross) {
            if (intakeState != IntakeState.REVERSE) {
                prevIntakeState = intakeState; // remember what we were doing
                intakeState = IntakeState.REVERSE;
            }
        } else {
            // When cross released, return to previous state if we were reversing
            if (intakeState == IntakeState.REVERSE) {
                intakeState = prevIntakeState;
            }
        }
        if(gamepad2.optionsWasPressed() && gamepad2.shareWasPressed()){
            pinpoint.setPose(startPose);
            gamepad2.rumble(100);
        }
        if(gamepad2.touchpadWasPressed()){
            spinner.enabledSorting = !spinner.enabledSorting;
            gamepad2.rumbleBlips(spinner.enabledSorting ? 2 : 1);
        }
        limelightCorrectionMode = gamepad2.left_trigger > 0.5;
        telemetry.update();
    }
    private boolean rpmInRangeStable() {
        boolean inRange = (rpm >= (flywheelTargetRPM - RPM_TOL)) && (rpm <= (flywheelTargetRPM + RPM_TOL));
        long now = System.currentTimeMillis();

        if (!inRange) {
            rpmInRangeSinceMs = 0;
            return false;
        }
        if (rpmInRangeSinceMs == 0) rpmInRangeSinceMs = now;
        return (now - rpmInRangeSinceMs) >= RPM_STABLE_MS;
    }

    private Pose processPedroPose(Pose pedroPose){
        double x = pedroPose.getX();
        double y = pedroPose.getY();
        double heading = pedroPose.getHeading();
        double currentTurretAngle = turret.getCurrentPosition() * TURRET_DEG_PER_TICK + startTurretAngle;
        heading = normalizeAngle(heading - currentTurretAngle);
        x += (Math.cos(heading) * LL_OFFSET_X - Math.sin(heading) * LL_OFFSET_Y)*INCH_PER_METER;
        y += (Math.sin(heading) * LL_OFFSET_X + Math.cos(heading) * LL_OFFSET_Y)*INCH_PER_METER;
        x -= (turretOffsetX * Math.cos(heading) - turretOffsetY * Math.sin(heading))*INCH_PER_METER;
        y -= (turretOffsetX * Math.sin(heading) + turretOffsetY * Math.cos(heading))*INCH_PER_METER;
        pedroPose = new Pose(x,y,heading);
        return pedroPose;
    }
    private void computeParameters() {
        double d = Math.hypot(targetX - turretX, targetY - turretY) * 0.0254;

        if (d <= 0) {
            flywheelTargetRPM = 2000;
            trajectoryAngle = maxTrajectoryAngle;
            return;
        }

        double k = (4.0 * preferredMaxHeightThrow / d)
                * (1.0 - Math.sqrt(1.0 - relativeHeight / preferredMaxHeightThrow));

        double idealAngle = Math.toDegrees(Math.atan(k));
        boolean constrained = (idealAngle > maxTrajectoryAngle || idealAngle < minTrajectoryAngle);

        if (constrained) {
            trajectoryAngle = (idealAngle > maxTrajectoryAngle) ? maxTrajectoryAngle : minTrajectoryAngle;

            double thetaRad = Math.toRadians(trajectoryAngle);
            double cosTheta = Math.cos(thetaRad);
            double tanTheta = Math.tan(thetaRad);

            if (Math.abs(cosTheta) < 1e-6) {
                return;
            }

            double denominator = d * tanTheta - relativeHeight;

            if (denominator <= 0) {
                flywheelTargetRPM = minFlywheelRPM;
                return;
            }

            double exitVelocity = Math.sqrt((g * d * d) /
                    (2 * cosTheta * cosTheta * denominator));

            flywheelTargetRPM = (int)(60 * exitVelocity /
                    (2 * Math.PI * flywheelRadius * launcherEfficiency));

        } else {
            trajectoryAngle = idealAngle;

            double thetaRad = Math.toRadians(trajectoryAngle);
            double sinTheta = Math.sin(thetaRad);

            if (Math.abs(sinTheta) < 1e-6) {
                return;
            }

            double v0 = Math.sqrt(2 * g * preferredMaxHeightThrow) / sinTheta;

            flywheelTargetRPM = (int)(60 * v0 / (2 * Math.PI * flywheelRadius * launcherEfficiency));
        }

        flywheelTargetRPM = Math.max(minFlywheelRPM, Math.min(flywheelTargetRPM, maxFlywheelRPM));
    }
    private void updateTurretAim() {
        disableIfNotInLaunchZone();
        double targetTurretDeg;
        double currentTurretDeg;
        if(aimingEnabled) {
            double robotX = pose.getX();
            double robotY = pose.getY();
            double robotHeading = pose.getHeading();
            double robotHeadingDeg = Math.toDegrees(robotHeading);
            turretX = robotX + turretOffsetX * Math.cos(robotHeading) - turretOffsetY * Math.sin(robotHeading);
            turretY = robotY + turretOffsetX * Math.sin(robotHeading) + turretOffsetY * Math.cos(robotHeading);
            double dx = targetX - turretX;
            double dy = targetY - turretY;

            double fieldAngle = Math.toDegrees(Math.atan2(dy, dx));
            currentTurretDeg = turret.getCurrentPosition() * TURRET_DEG_PER_TICK + startTurretAngle;
            targetTurretDeg = normalizeAngle(fieldAngle - robotHeadingDeg);
        } else {
            currentTurretDeg = turret.getCurrentPosition() * TURRET_DEG_PER_TICK + startTurretAngle;
            targetTurretDeg = startTurretAngle;
        }
        currentTurretDeg=normalizeAngle(currentTurretDeg);
        if(targetTurretDeg < 0 && targetTurretDeg > LEFT_LIMIT){
            targetTurretDeg = LEFT_LIMIT;
        } else if (targetTurretDeg >= 0 && targetTurretDeg < RIGHT_LIMIT) {
            targetTurretDeg = RIGHT_LIMIT;
        }

        double error = normalizeAngle(targetTurretDeg - currentTurretDeg);

        double power = error * kP;
        turretOnTarget = Math.abs(error) <= TURRET_TOL_DEG;
        power = Range.clip(power, -MAX_POWER_TURRET, MAX_POWER_TURRET);
        turret.setPower(power);
    }
    Pose velocity;
    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }
    private void updateLauncher(){
        double ticksPerSecond;
        if(launcherEnabled){
            ticksPerSecond = flywheelTargetRPM * FLYWHEEL_TICKS_PER_REV / 60.0;
        } else {
            ticksPerSecond = 2000 * FLYWHEEL_TICKS_PER_REV / 60.0;
        }
        flywheel.setVelocity(ticksPerSecond);
        rpm = flywheel.getVelocity() / FLYWHEEL_TICKS_PER_REV * 60.0;
    }
    private void updateFlywheel() {
        // Measure RPM
        rpm = flywheel.getVelocity() / FLYWHEEL_TICKS_PER_REV * 60.0;

        double target = flywheelTargetRPM;

        // Convert target to ticks/sec for RUN_USING_ENCODER velocity
        double targetTPS = target * FLYWHEEL_TICKS_PER_REV / 60.0;

        // === Control policy ===
        // In-band (±100rpm): PID velocity hold
        // Below band: full power 1.0
        // Above band: power 0.0
        if (Math.abs(rpm - target) <= RPM_TOL) {
            // PID HOLD
            if (flywheel.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
                flywheel.setPower(0); // clear any open-loop command
                flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            flywheel.setVelocityPIDFCoefficients(kP_v, kI_v, kD_v, kF_v);
            flywheel.setVelocity(targetTPS);
        } else if (rpm < (target - RPM_TOL)) {
            // KICK UP FAST
            if (flywheel.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
                flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            flywheel.setPower(1.0);
        } else {
            // TOO FAST -> CUT POWER
            if (flywheel.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
                flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            flywheel.setPower(0.0);
        }

        // Refresh rpm after command (optional but nice for telemetry stability)
        rpm = flywheel.getVelocity() / FLYWHEEL_TICKS_PER_REV * 60.0;
    }




    private void updateTrajectoryAngle(){
        setTrajectoryAngle(trajectoryAngle);
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
        flywheelTargetRPM = 2000;
        updateLauncher();
    }

    public void disableLauncher(){
        if(!launcherEnabled) return;
        launcherEnabled = false;
        flywheelTargetRPM = 2000;
        updateLauncher();
    }

    public void enableAiming(){
        aimingEnabled = true;
    }

    public void disableAiming(){
        aimingEnabled = false;
    }
    private void Drive() {
        double left_x = gamepad2.left_stick_x;
        double left_y = -gamepad2.left_stick_y;
        double right_x = gamepad2.right_stick_x;

        double front_left_pw  = left_y + left_x + right_x;
        double back_left_pw   = left_y - left_x + right_x;
        double front_right_pw = left_y - left_x - right_x;
        double back_right_pw  = left_y + left_x - right_x;

        double max = Math.max(Math.abs(front_left_pw),
                Math.max(Math.abs(back_left_pw),
                        Math.max(Math.abs(front_right_pw), Math.abs(back_right_pw))));
        if (max > 1.0) {
            front_left_pw  /= max;
            back_left_pw   /= max;
            front_right_pw /= max;
            back_right_pw  /= max;
        }

        front_left.setPower(front_left_pw);
        back_left.setPower(back_left_pw);
        front_right.setPower(front_right_pw);
        back_right.setPower(back_right_pw);
    }
}