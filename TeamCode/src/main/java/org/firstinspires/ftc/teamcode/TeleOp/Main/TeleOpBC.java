package org.firstinspires.ftc.teamcode.TeleOp.Main;

import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "TeleOPBlueClose", group = "TeleOp")
public class TeleOpBC extends LinearOpMode {
    private Spindexer spindexer;
    private Turret turret;
    private Intake intake;
    private DriveTrain drive;
    private final Pose startPose = new Pose(64.0, 8.0, Math.toRadians(90));
    ElapsedTime telemetryTimer = new ElapsedTime();


    // State variables
    private boolean intakeActive = false;
    private boolean turretEnabled = false;

    // Edge detection
    private boolean lastDpadUp = false;
    private boolean lastB = false;
    private boolean lastA = false;

    @Override
    public void runOpMode() {
        PinpointLocalizer pinpoint = new PinpointLocalizer(hardwareMap, Constants.localizerConstants);
        spindexer = new Spindexer(hardwareMap,
                "spinnerClose", "spinnerFar",
                "ejector",
                "Color1", "Color2", "Color3"
        );
        pinpoint.setStartPose(startPose);
        turret = new Turret(hardwareMap, "flywheel", "tureta", "unghituretaoy", pinpoint);
        intake = new Intake(hardwareMap, "intake");
        drive = new DriveTrain(hardwareMap, gamepad1, "fl", "fr", "rl", "rr");

        intake.init();
        spindexer.init();
        drive.init();
        turret.init(0.0, 144.0);

        telemetry.addLine("Welcome to CIP Airlines");
        telemetry.addLine("Status: Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            drive.update();

            // Dpad Up: Toggle intake motor
            if (gamepad1.dpad_up && !lastDpadUp) {
                intakeActive = !intakeActive;
            }
            lastDpadUp = gamepad1.dpad_up;

            // Dpad Down: Hold to reverse intake
            if (gamepad1.dpad_down) {
                intake.setPower(1.0);
            } else if (intakeActive) {
                intake.setPower(-1.0);
            } else {
                intake.setPower(0);
            }
            if (gamepad1.right_bumper) {
                drive.setPowerMultiplier(0.5);
            } else {
                drive.setPowerMultiplier(1.0);
            }

            if (gamepad2.right_trigger > 0.75) {
                spindexer.spindexerState = Spindexer.SpindexerState.OUTTAKE;
            }

            if (gamepad2.b && !lastB) {
                spindexer.spindexerState = Spindexer.SpindexerState.INTAKE;
                intakeActive = true;
            }
            lastB = gamepad2.b;

            if (gamepad2.a && !lastA) {
                turretEnabled = !turretEnabled;
                if (turretEnabled) {
                    turret.enableAiming();
                } else {
                    turret.disableAiming();
                }
            }
            if(spindexer.SlotChangerFull || spindexer.spindexerState == Spindexer.SpindexerState.OUTTAKE){
                turret.enableLauncher();
            } else {
                turret.disableLauncher();
            }
            lastA = gamepad2.a;
            pinpoint.update();
            spindexer.update();
            turret.update();

            if (telemetryTimer.milliseconds() > 250) {
                Pose currentPose = pinpoint.getPose();
                telemetry.addData("Balls",spindexer.ballsLoaded);
                telemetry.addData("Slot Changer Full", spindexer.SlotChangerFull);
                telemetry.addData("Color1",spindexer.IntakeSensor.getDetectedColor());
                telemetry.addData("Color2",spindexer.AuxSensor1.getDetectedColor());
                telemetry.addData("Color3",spindexer.AuxSensor2.getDetectedColor());
                telemetry.addData("Hue1",spindexer.IntakeSensor.hue);
                telemetry.addData("Hue2",spindexer.AuxSensor1.hue);
                telemetry.addData("Hue3",spindexer.AuxSensor2.hue);
                telemetry.addData("Sat1", spindexer.IntakeSensor.sat);
                telemetry.addData("Sat2", spindexer.AuxSensor1.sat);
                telemetry.addData("Sat3", spindexer.AuxSensor2.sat);
                telemetry.addData("X", currentPose.getX());
                telemetry.addData("Y", currentPose.getY());
                telemetry.addData("Heading", Math.toDegrees(currentPose.getHeading()));
                telemetry.addData("Spindexer State", spindexer.spindexerState);
                telemetry.addData("Intake Active", intakeActive);
                telemetry.addData("Turret Enabled", turretEnabled);
                telemetry.addData("Flywheel Target RPM", turret.flywheelTargetRPM);
                telemetry.update();
                telemetryTimer.reset();
            }
        }
    }
}
