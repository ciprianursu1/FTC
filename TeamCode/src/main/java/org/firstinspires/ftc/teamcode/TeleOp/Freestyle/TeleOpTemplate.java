package org.firstinspires.ftc.teamcode.TeleOp.Freestyle;

import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

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

@TeleOp(name="Freestyle")
public class TeleOpTemplate extends OpMode {
    Robot robot;
    Limelight limelight;
    PinpointLocalizer pinpointLocalizer;

    DcMotor motorLeftFront;
    DcMotor motorRightFront;
    DcMotor motorLeftRear;
    DcMotor motorRightRear;

    DcMotorEx motorRawSpindexer;
    NormalizedColorSensor rawColorSensor1;
    NormalizedColorSensor rawColorSensor2;
    NormalizedColorSensor rawColorSensor3;

    Servo servoEjector;
    Servo servoHood;
    DcMotor motorRawIntake;
    DcMotorEx motorRawFlywheel;
    DcMotorEx motorRawTurret;

    Limelight3A rawLimelight3A;
    IMU rawIMU;

    // --- INTERMEDIATE WRAPPER OBJECTS (CUSTOM LAYER) ---
    MecanumDrive mecanumDrive;
    ClosedLoopDC spindexerClosedLoop;
    SlotChanger slotChanger;
    ColorSensor wrapperColorSensor1;
    ColorSensor wrapperColorSensor2;
    ColorSensor wrapperColorSensor3;
    ColorSensor[] majorityVotingSensors;
    Spindexer spindexer;
    TransferServo transferServo;
    Hood hood;
    Intake intake;
    Flywheel flywheel;
    ClosedLoopDC turretClosedLoop;
    TurretSwivel turretSwivel;
    ArtifactHandler artifactHandler;

    @Override
    public void init() {
        double[] savedData = PoseStorage.loadPose(hardwareMap.appContext);
        Pose startPose = new Pose(savedData[0], savedData[1], savedData[2]);
        boolean alliance = savedData[3] == 1; // true = Blue, false = Red
        int tagID = (int) savedData[4];

        pinpointLocalizer = new PinpointLocalizer(hardwareMap, Constants.localizerConstants);
        pinpointLocalizer.setPose(startPose);


        motorLeftFront      = hardwareMap.get(DcMotor.class, RobotConfig.LEFT_FRONT);
        motorRightFront     = hardwareMap.get(DcMotor.class, RobotConfig.RIGHT_FRONT);
        motorLeftRear       = hardwareMap.get(DcMotor.class, RobotConfig.LEFT_REAR);
        motorRightRear      = hardwareMap.get(DcMotor.class, RobotConfig.RIGHT_REAR);

        motorRawSpindexer   = hardwareMap.get(DcMotorEx.class, RobotConfig.SPINDEXER);
        rawColorSensor1     = hardwareMap.get(NormalizedColorSensor.class, RobotConfig.COLOR_1);
        rawColorSensor2     = hardwareMap.get(NormalizedColorSensor.class, RobotConfig.COLOR_2);
        rawColorSensor3     = hardwareMap.get(NormalizedColorSensor.class, RobotConfig.COLOR_3);

        servoEjector        = hardwareMap.get(Servo.class, RobotConfig.EJECTOR);
        servoEjector.setDirection(Servo.Direction.REVERSE);
        servoEjector.setPosition(RobotConfig.TRANSFER_DOWN);
        servoHood           = hardwareMap.get(Servo.class, RobotConfig.HOOD_SERVO);
        servoHood.setPosition(0);
        motorRawIntake      = hardwareMap.get(DcMotor.class, RobotConfig.INTAKE);
        motorRawIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRawFlywheel    = hardwareMap.get(DcMotorEx.class, RobotConfig.FLYWHEEL);
        motorRawTurret      = hardwareMap.get(DcMotorEx.class, RobotConfig.TURRET);

        rawLimelight3A      = hardwareMap.get(Limelight3A.class, RobotConfig.LIMELIGHT);
        rawIMU              = hardwareMap.get(IMU.class, RobotConfig.IMU);
        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        );
        rawIMU.initialize(new IMU.Parameters(orientation));
        mecanumDrive = new MecanumDrive(
                gamepad1,
                motorLeftFront,
                motorRightFront,
                motorLeftRear,
                motorRightRear,
                false
        );

        spindexerClosedLoop = new ClosedLoopDC(
                motorRawSpindexer,
                RobotConfig.spindexerPID,
                RobotConfig.SPINDEXER_POWER,
                RobotConfig.SPINDEXER_TICKS_PER_REV
        );

        slotChanger = new SlotChanger(
                spindexerClosedLoop,
                RobotConfig.SPINDEXER_SLOTS,
                RobotConfig.SPINDEXER_OUTTAKE_OFFSET_DEG,
                RobotConfig.SLOT_STALL_TIMEOUT_MS,
                RobotConfig.SLOT_STALL_MIN_MOVEMENT_DEG
        );

        wrapperColorSensor1 = new ColorSensor(rawColorSensor1);
        wrapperColorSensor2 = new ColorSensor(rawColorSensor2);
        wrapperColorSensor3 = new ColorSensor(rawColorSensor3);

        majorityVotingSensors = new ColorSensor[]{
                wrapperColorSensor1,
                wrapperColorSensor2,
                wrapperColorSensor3
        };

        spindexer = new Spindexer(
                slotChanger,
                wrapperColorSensor1,
                majorityVotingSensors,
                RobotConfig.VERIFICATION_SENSOR_TO_SLOT,
                RobotConfig.tuneV3
        );

        transferServo = new TransferServo(
                servoEjector,
                RobotConfig.TRANSFER_UP,
                RobotConfig.TRANSFER_DOWN,
                RobotConfig.TRANSFER_DOWN
        );

        hood = new Hood(servoHood, RobotConfig.HOOD_MAX_ANGLE, RobotConfig.HOOD_MIN_ANGLE);

        intake = new Intake(motorRawIntake);

        flywheel = new Flywheel(
                motorRawFlywheel,
                RobotConfig.FLYWHEEL_TICKS_PER_REV,
                RobotConfig.FLYWHEEL_RPM_DEADBAND,
                RobotConfig.FLYWHEEL_RADIUS_METERS,
                RobotConfig.FLYWHEEL_EFFICIENCY,
                RobotConfig.FLYWHEEL_DISABLE_RPM,
                RobotConfig.FLYWHEEL_PID
        );

        turretClosedLoop = new ClosedLoopDC(
                motorRawTurret,
                RobotConfig.turretPID,
                RobotConfig.TURRET_POWER,
                RobotConfig.TURRET_TICKS_PER_REV
        );

        turretSwivel = new TurretSwivel(
                turretClosedLoop,
                RobotConfig.TURRET_LIMIT_LEFT,
                RobotConfig.TURRET_LIMIT_RIGHT,
                RobotConfig.TURRET_START_ANGLE
        );

        artifactHandler = new ArtifactHandler(
                spindexer,
                transferServo,
                hood,
                intake,
                flywheel,
                turretSwivel,
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

        robot = new Robot(
                mecanumDrive,
                artifactHandler,
                gamepad1,
                gamepad2,
                pinpointLocalizer,
                (alliance ? RobotConfig.BLUE_RESET_POSE : RobotConfig.RED_RESET_POSE),
                RobotConfig.autoAimZones,
                RobotConfig.autoAimZoneTargets,
                new double[]{(alliance ? RobotConfig.BLUE_PRIMARY_TARGET_X : RobotConfig.RED_PRIMARY_TARGET_X), RobotConfig.PRIMARY_TARGET_Y},
                new double[]{RobotConfig.SECONDARY_TARGET_X, RobotConfig.SECONDARY_TARGET_Y},
                alliance,
                false
        );

        limelight = new Limelight(rawLimelight3A, rawIMU);
        limelight.init(startPose.getHeading(), AngleUnit.RADIANS);
        limelight.changePipeline(RobotConfig.LIMELIGHT_POSITION_PIPELINE, false);

        robot.init();

        switch (tagID) {
            case 21:
                robot.artifactHandler.setMotif(new int[]{1, 2, 2});
                break;
            case 22:
                robot.artifactHandler.setMotif(new int[]{2, 1, 2});
                break;
            case 23:
                robot.artifactHandler.setMotif(new int[]{2, 2, 1});
                break;
        }
    }

    @Override
    public void loop() {
        pinpointLocalizer.update();
        limelight.setTurretYawOffset(robot.artifactHandler.getTurretAngle(), AngleUnit.DEGREES);
        limelight.update();
        if (limelight.isFreshData()) {
            pinpointLocalizer.setPose(limelight.getPose());
        }

        robot.update();
    }
}
