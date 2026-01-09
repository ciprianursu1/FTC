package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="ColorDetection+cevaspinner")
public class spinner extends LinearOpMode {

    ElapsedTime pidTimer = new ElapsedTime();
    static final int POSITION_TOLERANCE = 10;
    int[] last5Sensor1 = new int[5];
    int[] last5Sensor2 = new int[5];
    int[] last5Sensor3 = new int[5];
    int indexSensor1 = 0;
    int indexSensor2 = 0;
    int indexSensor3 = 0;

    int[] slots = new int[3];

    int lastStableColorSensor1 = 0;
    int lastStableColorSensor2 = 0;
    int lastStableColorSensor3 = 0;

    boolean spinnerBusy = false;

    ColorSensor colorsensorSLot1;
    ColorSensor colorsensorSLot2;
    ColorSensor colorsensorSLot3;

    DcMotorEx spinner;
    DcMotor intake;
    DcMotor tureta;
    Servo ejector;
    DcMotor front_left;
    DcMotor front_right;
    DcMotor back_left;
    DcMotor back_right;

    static final double TICKS_PER_REV = 384.5;
    static final double TICKS_PER_DEGREE = TICKS_PER_REV / 360.0;

    // PID coefficients (P is tunable)
    double P = 0.0101;
    double I = 0.0000;
    double D = 0.0015;

    double integralSum = 0;
    double lastError = 0;

    int targetTicks = 0;


    private void InitWheels()
    {
        front_left = hardwareMap.dcMotor.get("lf");
        front_right = hardwareMap.dcMotor.get("rf");
        back_left = hardwareMap.dcMotor.get("lr");
        back_right = hardwareMap.dcMotor.get("rr");

        front_right.setDirection(DcMotorSimple.Direction.FORWARD);
        back_right.setDirection(DcMotorSimple.Direction.FORWARD);
        front_left.setDirection(DcMotorSimple.Direction.REVERSE);
        back_left.setDirection(DcMotorSimple.Direction.REVERSE);
    }


    private void SetWheelsPower() {
        double left_x  = gamepad1.left_stick_x;
        double left_y  = -gamepad1.left_stick_y; // forward is negative
        double right_x = gamepad1.right_stick_x;

        // Calculate raw motor powers
        double front_left_pw  = left_y + left_x + right_x;
        double back_left_pw   = left_y - left_x + right_x;
        double front_right_pw = left_y - left_x - right_x;
        double back_right_pw  = left_y + left_x - right_x;

        // Normalize so no motor power exceeds 1.0
        double max = Math.max(Math.abs(front_left_pw),
                Math.max(Math.abs(back_left_pw),
                        Math.max(Math.abs(front_right_pw), Math.abs(back_right_pw))));

        if (max > 1.0) {
            front_left_pw  /= max;
            back_left_pw   /= max;
            front_right_pw /= max;
            back_right_pw  /= max;
        }

        // Apply power
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
        if (max == r) h = (double)(g - b) / chroma;
        else if (max == g) h = (double)(b - r) / chroma + 2.0;
        else h = (double)(r - g) / chroma + 4.0;
        h *= 60.0;
        if (h < 0) h += 360.0;
        return h;
    }

    private int smekerie(ColorSensor colorSensor) {
        int r = colorSensor.red();
        int g = colorSensor.green();
        int b = colorSensor.blue();
        int alpha = colorSensor.alpha();
        double h = getHue(r, g, b);
        int detected;
        if (alpha < 100 && (h == 150 || h == 144)) detected = 0;
        else if ((h > 215) || (alpha < 100 && (h == 160 || h == 180))) detected = 2;
        else if (h > 135 && h < 160) detected = 1;
        else if ((h == 210 || h == 220 || h == 225 || h == 200) && alpha < 100) detected = 2;
        else detected = 0;
        return detected;
    }
    //smekerie returneaza: 0 daca e pl
    //smekereie returneaza 1 daca verde
    //smekerie returneazax 2 daca mov

    private int processSingleSensor(ColorSensor sensor, int[] last5, int index) {
        int detected = smekerie(sensor);
        //detected e culoarea detectata de senzor
        last5[index] = detected;
        //index e pozitia din vector unde imi pune culoarea
        index = (index + 1) % 5;
        int count1 = 0; //nr verzi
        int count2 = 0; //nr movi
        for (int v : last5) {
            if (v == 1) count1++;
            else if (v == 2) count2++;
        }
        int finalColor = 0;
        if (count1 >= 3 && count1 > count2) finalColor = 1;
        else if (count2 >= 3 && count2 > count1) finalColor = 2;
        //if-ul asta preia maximul din cele 5 valore
        return finalColor;
    }
// returneaza culoarea predominanta din 5 in 5 iteratii ale loop ului.
    private void updateAllSlots() {
        slots[0] = processSingleSensor(colorsensorSLot1, last5Sensor1, indexSensor1);
        indexSensor1 = (indexSensor1 + 1) % 5;
        slots[1] = processSingleSensor(colorsensorSLot2, last5Sensor2, indexSensor2);
        indexSensor2 = (indexSensor2 + 1) % 5;
        slots[2] = processSingleSensor(colorsensorSLot3, last5Sensor3, indexSensor3);
        indexSensor3 = (indexSensor3 + 1) % 5;
    }

    private void colorDrivenSpinnerLogic() {
        boolean newColorDetected = false;
        if ((slots[0] != 0 && slots[0] != lastStableColorSensor1) || (slots[1] != 0 && slots[1] != lastStableColorSensor2) || (slots[2] != 0 && slots[2] != lastStableColorSensor3) ) newColorDetected = true;
        if (newColorDetected && !spinnerBusy) {
            spinnerBusy = true;

            targetTicks =targetTicks + (int)(120 * TICKS_PER_DEGREE);

            double currentPos = spinner.getCurrentPosition();
            double error = targetTicks - currentPos;

            integralSum += error;
            double derivative = error - lastError;
            lastError = error;

            double pidOutput = error * P + integralSum * I + derivative * D;
            pidOutput = Math.max(-1, Math.min(1, pidOutput));

            spinner.setPower(pidOutput);
           /* spinner.setTargetPosition(spinner.getCurrentPosition() + 560);// aici ar veni logica de PID din codu mare
            spinner.setPower(0.5);*/
            //vedem empiric cat dureaza exact sa se dea ejector ul sa se duca sus si jos sau bagam un isBusy
            //ne putem folosi de update urile astea pana stim ca toate slot urile sunt ocupate apoi updatam de abia in outtake
        }
        lastStableColorSensor1 = slots[0];
        lastStableColorSensor2 = slots[1];
        lastStableColorSensor3 = slots[2];
        if (spinnerBusy && !spinner.isBusy()) {
            spinner.setPower(0);//la fel si aici
            spinnerBusy = false;
        }
    }

    private void updateTelemetry() {
        telemetry.addData("Slot 1", slots[0]);
        telemetry.addData("Slot 2", slots[1]);
        telemetry.addData("Slot 3", slots[2]);
        telemetry.addData("Pregatit de lanasare", spinnerBusy);
        telemetry.update();
    }

    @Override
    public void runOpMode() {
        InitWheels();
      // colorsensorSLot1 = hardwareMap.colorSensor.get("Color1");
        //colorsensorSLot2 = hardwareMap.colorSensor.get("Color2");
        //colorsensorSLot3 = hardwareMap.colorSensor.get("Color3");
        spinner = hardwareMap.get(DcMotorEx.class, "spinner");
        intake = hardwareMap.get(DcMotor.class, "intake");
       // ejector = hardwareMap.get(Servo.class,"ejector");
        tureta = (DcMotor) hardwareMap.get(DcMotor.class,"tureta");
        spinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        spinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spinner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        tureta.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();
        pidTimer.reset();
        while (opModeIsActive()) {
            if (gamepad1.dpadDownWasPressed())
            {
                intake.setPower(1);
            }
            if (gamepad1.psWasPressed())
            {
                intake.setPower(0);
            }

            double currentPos = spinner.getCurrentPosition();
            double error = targetTicks - currentPos;

            double dt = pidTimer.seconds();
            pidTimer.reset();

            if (Math.abs(error) > POSITION_TOLERANCE) {
                integralSum += error * dt;
                double derivative = (error - lastError) / dt;
                lastError = error;

                double pidOutput = error * P + integralSum * I + derivative * D;
                pidOutput = Math.max(-0.5, Math.min(0.5, pidOutput));

                spinner.setPower(pidOutput);
            } else {
                spinner.setPower(0);
                integralSum = 0;
            }

            if (gamepad1.dpadRightWasPressed())
            {
                targetTicks += (int)(120 * TICKS_PER_DEGREE);
                integralSum = 0;   // reset integral on new target
                lastError = 0;
            }


           // updateAllSlots();
            //colorDrivenSpinnerLogic();
            //updateTelemetry();
            SetWheelsPower();
            idle();
        }
    }
}
