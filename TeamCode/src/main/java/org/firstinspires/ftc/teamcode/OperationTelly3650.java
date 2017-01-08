package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="Operation: Telly", group="3650")
public class OperationTelly3650 extends OpMode {

    //assigning state variables
    DcMotor rDrive, lDrive, collector, shooter;
    Servo forePush, aftPush, ballServo;
    ColorSensor colorSensor;
    OpticalDistanceSensor ods;
    LightSensor light;
    double aftNeutral, foreNeutral;




    @Override
    public void init() {

        // linking variables to hardware components
        lDrive = hardwareMap.dcMotor.get("lDrive");
        rDrive = hardwareMap.dcMotor.get("rDrive");
        collector = hardwareMap.dcMotor.get("collector");
        shooter = hardwareMap.dcMotor.get("shooter");
        colorSensor = hardwareMap.colorSensor.get("colorSensor");
        ballServo = hardwareMap.servo.get("ballServo");

        ods = hardwareMap.opticalDistanceSensor.get("ods");
        light = hardwareMap.lightSensor.get("light");

        //rest positions for servos
        aftNeutral = 1.00; //check these!!!
        foreNeutral = .1;

        //button pushing servos
        forePush = hardwareMap.servo.get("forePush");
        aftPush = hardwareMap.servo.get("aftPush");

        //Reversing direction of R Drive so it spins the correct way
        rDrive.setDirection(DcMotor.Direction.REVERSE);
        rDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        colorSensor.enableLed(false);
        forePush.setPosition(foreNeutral);
        aftPush.setPosition(aftNeutral);

    }

    @Override
    public void loop() {

        //set motor controls to joystick
        lDrive.setPower(gamepad1.left_stick_y*.7);
        rDrive.setPower(gamepad1.right_stick_y*.7);

        if(gamepad1.right_bumper){
            //aftPush.setPosition(aftPush.getPosition()-.1);
            forePush.setPosition(forePush.getPosition()+.1);
        }
        else if(gamepad1.left_bumper){
            aftPush.setPosition(aftPush.getPosition()-.1);
            //forePush.setPosition(forePush.getPosition()-.1);
        }
        else{
            aftPush.setPosition(aftNeutral);
            forePush.setPosition(foreNeutral);
        }


        if (gamepad2.dpad_down && gamepad2.right_trigger == 0){
            shooter.setPower(-1.0);
        }
        else if (!gamepad2.dpad_down && gamepad2.right_trigger > 0){
            shooter.setPower(gamepad2.right_trigger);
        }
        else{
            shooter.setPower(0);
        }


        //not sure which one works, so commenting this out for now

        /*
        if (gamepad2.dpad_down && gamepad2.right_trigger == 0){
            shooter.setPower(-1.0);
        }
        else if (!gamepad2.dpad_down && gamepad2.right_trigger == 1){
            shooter.setPower(1.0);
        }
        else{
            shooter.setPower(0);
        }*/




        if (gamepad2.left_bumper){
            collector.setPower(-1.0);
        }
        else if(gamepad2.right_bumper){
            collector.setPower(1.00);
        }
        else {
            collector.setPower(0);
        }

        /*if (gamepad2.dpad_up){//up
            ballServo.setPosition(1.00);
        }
        else{//down
            ballServo.setPosition(0.14);
        }*/

        //shows values from color sensor on driver station phone
        telemetry.addData("Distance value",ods.getLightDetected());
        telemetry.addData("Light", light.getLightDetected());
        telemetry.addData("Red ", colorSensor.red());
        telemetry.addData("Green ", colorSensor.green());
        telemetry.addData("Blue ", colorSensor.blue());
        telemetry.addData("Aft ", aftPush.getPosition());
        telemetry.addData("Fore ", forePush.getPosition());
    }
}
