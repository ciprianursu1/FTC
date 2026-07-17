package org.firstinspires.ftc.teamcode.pedroPathing.Auto.Freestyle;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Subsystems.ArtifactHandler;
import org.firstinspires.ftc.teamcode.Subsystems.ClosedLoopDC;
import org.firstinspires.ftc.teamcode.Subsystems.ColorSensor;
import org.firstinspires.ftc.teamcode.Subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.Subsystems.Hood;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Limelight;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.PoseStorage;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.RobotConfig;
import org.firstinspires.ftc.teamcode.Subsystems.SlotChanger;
import org.firstinspires.ftc.teamcode.Subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.Subsystems.TransferServo;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSwivel;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public abstract class FreestyleAutoBase extends OpMode {
    protected Follower follower;
    protected Robot robot;
    protected Limelight limelight;
    protected PinpointLocalizer pinpointLocalizer;
    protected int pathState = 0;
    protected boolean pathStarted = false;
    protected Pose pose;
    protected ElapsedTime timer;
    protected int motifTag = RobotConfig.DEFAULT_MOTIF_TAG;

    protected abstract Pose getStartPose();
    protected abstract void initPaths();
    protected abstract void runPathState();

    @Override
    public void init() {
        timer = new ElapsedTime();
        pose = getStartPose();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(pose);

        pinpointLocalizer = new PinpointLocalizer(hardwareMap, Constants.localizerConstants);
        pinpointLocalizer.setPose(pose);

        initPaths();
        robot = createRobot();
        limelight = new Limelight(
                hardwareMap.get(Limelight3A.class, RobotConfig.LIMELIGHT),
                hardwareMap.get(IMU.class, RobotConfig.IMU)
        );
        limelight.init(pose.getHeading(), AngleUnit.RADIANS);
        limelight.changePipeline(RobotConfig.LIMELIGHT_MOTIF_PIPELINE, true);
        robot.init();
    }
    public void start(){
        timer.reset();
    }

    @Override
    public void loop() {
        follower.update();
        pose = follower.getPose();
        pinpointLocalizer.setPose(pose); //Limelight Correction is not needed for the 30s auto period
        runPathState();
        limelight.setTurretYawOffset(robot.artifactHandler.getTurretAngle(), AngleUnit.DEGREES);
        limelight.update();

        telemetry.addLine("=== Auto Path ===");
        telemetry.addData("Path state", pathState);
        telemetry.addData("Busy", follower.isBusy());
        telemetry.addData("LL turret yaw", limelight.getTurretYawOffset());
        telemetry.addData("LL tag", limelight.getTagID());
        telemetry.addData("X", pose.getX());
        telemetry.addData("Y", pose.getY());
        telemetry.addData("Heading", Math.toDegrees(pose.getHeading()));
        robot.update();
    }

    @Override
    public void stop() { // collects info to transfer to the teleop mode
        limelight.changePipeline(RobotConfig.LIMELIGHT_POSITION_PIPELINE, false);
        if (pose != null) {
            PoseStorage.savePose(
                    hardwareMap.appContext,
                    pose.getX(),
                    pose.getY(),
                    pose.getHeading(),
                    true,
                    motifTag
            );
        }
    }

    protected void followPath(PathChain path, int nextPathState, double power) {
        if (!pathStarted) {
            follower.followPath(path, power, true);
            pathStarted = true;
        }
        if (!follower.isBusy()) {
            pathStarted = false;
            pathState = nextPathState;
        }
    }

    private Robot createRobot() {
        DcMotor motorLeftFront = hardwareMap.get(DcMotor.class, RobotConfig.LEFT_FRONT);
        DcMotor motorRightFront = hardwareMap.get(DcMotor.class, RobotConfig.RIGHT_FRONT);
        DcMotor motorLeftRear = hardwareMap.get(DcMotor.class, RobotConfig.LEFT_REAR);
        DcMotor motorRightRear = hardwareMap.get(DcMotor.class, RobotConfig.RIGHT_REAR);

        DcMotorEx motorRawSpindexer = hardwareMap.get(DcMotorEx.class, RobotConfig.SPINDEXER);
        NormalizedColorSensor rawColorSensor1 =
                hardwareMap.get(NormalizedColorSensor.class, RobotConfig.COLOR_1);
        NormalizedColorSensor rawColorSensor2 =
                hardwareMap.get(NormalizedColorSensor.class, RobotConfig.COLOR_2);
        NormalizedColorSensor rawColorSensor3 =
                hardwareMap.get(NormalizedColorSensor.class, RobotConfig.COLOR_3);

        Servo servoEjector = hardwareMap.get(Servo.class, RobotConfig.EJECTOR);
        Servo servoHood = hardwareMap.get(Servo.class, RobotConfig.HOOD_SERVO);
        DcMotor motorRawIntake = hardwareMap.get(DcMotor.class, RobotConfig.INTAKE);
        DcMotorEx motorRawFlywheel = hardwareMap.get(DcMotorEx.class, RobotConfig.FLYWHEEL);
        DcMotorEx motorRawTurret = hardwareMap.get(DcMotorEx.class, RobotConfig.TURRET);
        servoEjector.setDirection(Servo.Direction.REVERSE);
        servoEjector.setPosition(RobotConfig.TRANSFER_DOWN);
        servoHood.setPosition(0);
        motorRawIntake.setDirection(DcMotorSimple.Direction.REVERSE);

        MecanumDrive mecanumDrive = new MecanumDrive(
                gamepad1,
                motorLeftFront,
                motorRightFront,
                motorLeftRear,
                motorRightRear,
                true
        );

        ClosedLoopDC spindexerClosedLoop = new ClosedLoopDC(
                motorRawSpindexer,
                RobotConfig.createSpindexerPID(),
                RobotConfig.SPINDEXER_POWER,
                RobotConfig.SPINDEXER_TICKS_PER_REV
        );

        SlotChanger slotChanger = new SlotChanger(
                spindexerClosedLoop,
                RobotConfig.SPINDEXER_SLOTS,
                RobotConfig.SPINDEXER_OUTTAKE_OFFSET_DEG,
                RobotConfig.SLOT_STALL_TIMEOUT_MS,
                RobotConfig.SLOT_STALL_MIN_MOVEMENT_DEG
        );

        ColorSensor wrapperColorSensor1 = new ColorSensor(rawColorSensor1);
        ColorSensor wrapperColorSensor2 = new ColorSensor(rawColorSensor2);
        ColorSensor wrapperColorSensor3 = new ColorSensor(rawColorSensor3);
        ColorSensor[] majorityVotingSensors = new ColorSensor[]{
                wrapperColorSensor1,
                wrapperColorSensor2,
                wrapperColorSensor3
        };

        Spindexer spindexer = new Spindexer(
                slotChanger,
                wrapperColorSensor1,
                majorityVotingSensors,
                RobotConfig.VERIFICATION_SENSOR_TO_SLOT,
                RobotConfig.tuneV3
        );

        ArtifactHandler artifactHandler = new ArtifactHandler(
                spindexer,
                new TransferServo(
                        servoEjector,
                        RobotConfig.TRANSFER_UP,
                        RobotConfig.TRANSFER_DOWN,
                        RobotConfig.TRANSFER_DOWN
                ),
                new Hood(servoHood, RobotConfig.HOOD_MAX_ANGLE, RobotConfig.HOOD_MIN_ANGLE),
                new Intake(motorRawIntake),
                new Flywheel(
                        motorRawFlywheel,
                        RobotConfig.FLYWHEEL_TICKS_PER_REV,
                        RobotConfig.FLYWHEEL_RPM_DEADBAND,
                        RobotConfig.FLYWHEEL_RADIUS_METERS,
                        RobotConfig.FLYWHEEL_EFFICIENCY,
                        RobotConfig.FLYWHEEL_DISABLE_RPM,
                        RobotConfig.FLYWHEEL_PID
                ),
                new TurretSwivel(
                        new ClosedLoopDC(
                                motorRawTurret,
                                RobotConfig.createTurretPID(),
                                RobotConfig.TURRET_POWER,
                                RobotConfig.TURRET_TICKS_PER_REV
                        ),
                        RobotConfig.TURRET_LIMIT_LEFT,
                        RobotConfig.TURRET_LIMIT_RIGHT,
                        RobotConfig.TURRET_START_ANGLE
                ),
                pinpointLocalizer,
                RobotConfig.BLUE_PRIMARY_TARGET_X,
                RobotConfig.PRIMARY_TARGET_Y,
                RobotConfig.TARGET_Z,
                RobotConfig.TURRET_OFFSET_X_METERS,
                RobotConfig.TURRET_OFFSET_Y_METERS,
                RobotConfig.DEFAULT_MOTIF,
                RobotConfig.TRANSFER_VERIFY_DELAY,
                RobotConfig.TRANSFER_MAX_RETRIES
        );

        return new Robot(
                mecanumDrive,
                artifactHandler,
                gamepad1,
                gamepad2,
                pinpointLocalizer,
                RobotConfig.BLUE_RESET_POSE,
                RobotConfig.autoAimZones,
                RobotConfig.autoAimZoneTargets,
                new double[]{RobotConfig.BLUE_PRIMARY_TARGET_X, RobotConfig.PRIMARY_TARGET_Y},
                new double[]{RobotConfig.SECONDARY_TARGET_X, RobotConfig.SECONDARY_TARGET_Y},
                true,
                true,
                telemetry
        );
    }
}
