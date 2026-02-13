package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Servo Increment Test", group="Test")
public class ServoTest extends LinearOpMode {

    private Servo testServo;
    private double servoPosition = 0.5;   // Start in the middle
    private final double increment = 0.01; // How fast it moves

    @Override
    public void runOpMode() {

        testServo = hardwareMap.get(Servo.class, "ejector");
        testServo.setPosition(servoPosition);

        telemetry.addLine("RB = Increase | LB = Decrease");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Increase while holding right bumper
            if (gamepad1.right_bumper) {
                servoPosition += increment;
            }

            // Decrease while holding left bumper
            if (gamepad1.left_bumper) {
                servoPosition -= increment;
            }

            // Clamp between 0 and 1
            servoPosition = Math.max(0.0, Math.min(1.0, servoPosition));

            testServo.setPosition(servoPosition);

            telemetry.addData("Servo Position", servoPosition);
            telemetry.update();

            sleep(20); // Small delay for smoother movement
        }
    }
}
