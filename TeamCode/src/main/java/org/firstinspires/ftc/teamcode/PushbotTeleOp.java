package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by bryce on 10/10/16.
 */

public class PushbotTeleOp extends OpMode {
    DcMotor LDrive, RDrive, TowerArm, Test;
    Servo LServo, RServo;
    ColorSensor color;

    @Override
    public void init() {
        //Assigning motors + servos
        LDrive = hardwareMap.dcMotor.get("LDrive");
        RDrive = hardwareMap.dcMotor.get("RDrive");
        Test = hardwareMap.dcMotor.get("Test");

        TowerArm = hardwareMap.dcMotor.get("TowerArm");

        LServo = hardwareMap.servo.get("LServo");
        RServo = hardwareMap.servo.get("RServo");

        //Reverses motor direction to make it go the correct way
        LDrive.setDirection(DcMotor.Direction.REVERSE);
        TowerArm.setDirection(DcMotor.Direction.REVERSE);

        //color stuff
        //float colorValues[] = {0F,0F,0F};
        //final float values[] = colorValues;
        color = hardwareMap.colorSensor.get("color sensor");
        color.enableLed(true);


    }

    @Override
    public void loop() {

        float leftY1 = gamepad1.left_stick_y;
        float rightY1 = gamepad1.right_stick_y;

        float rightY2 = gamepad2.right_stick_y;
        float leftY2 = gamepad2.left_stick_y;

        //set motor controls
        LDrive.setPower(leftY1);
        RDrive.setPower(rightY1);
        Test.setPower(leftY2);

        TowerArm.setPower(rightY2 / 2);
        //servo controls
        if (gamepad2.left_bumper) {
            LServo.setPosition(-1);
        }
        if (gamepad2.right_bumper) {
            RServo.setPosition(1);
        }
        if (gamepad2.left_trigger > 0) {
            LServo.setPosition(1);
        }
        if (gamepad2.right_trigger > 0) {
            RServo.setPosition(-1);
        }

        // Color.RGBToHSV(color.red() * 8, color.green() * 8, color.blue() * 8, colorValues);

        telemetry.addData("Red  ", color.red());
        telemetry.addData("Green", color.green());
        telemetry.addData("Blue ", color.blue());

    }
}
