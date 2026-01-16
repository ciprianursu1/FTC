
package org.firstinspires.ftc.teamcode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name = "LimitSwitchSimple")
public class limitswich extends LinearOpMode {

    DcMotor motor;
    TouchSensor limit;

    @Override
    public void runOpMode() {

        limit = hardwareMap.get(TouchSensor.class, "limit");

        // IMPORTANT: digital input

        waitForStart();

        while (opModeIsActive()) {

            // For REV Touch Sensor:
            // pressed = false


            telemetry.addData("Limit pressed", limit.isPressed());
            telemetry.update();
        }
    }
}
