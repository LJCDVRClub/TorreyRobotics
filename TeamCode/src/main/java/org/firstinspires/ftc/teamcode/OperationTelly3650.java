package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Bryce on 11/2/2016.
 */
@TeleOp(name="Operation: Telly", group="3650")
public class OperationTelly3650 extends OpMode {

    //assigning state variables
    DcMotor rDrive, lDrive, collector, shooter;
    ColorSensor colorSensor;
    OpticalDistanceSensor ods;
    LightSensor light;




    @Override
    public void init() {

        // linking variables to hardware components
        lDrive = hardwareMap.dcMotor.get("lDrive");
        rDrive = hardwareMap.dcMotor.get("rDrive");
        collector = hardwareMap.dcMotor.get("collector");
        shooter = hardwareMap.dcMotor.get("shooter");
        colorSensor = hardwareMap.colorSensor.get("colorSensor");

        ods = hardwareMap.opticalDistanceSensor.get("ods");
        light = hardwareMap.lightSensor.get("light");

        //Reversing direction of R Drive so it spins the correct way
        rDrive.setDirection(DcMotor.Direction.REVERSE);
        rDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        colorSensor.enableLed(false);

    }

    @Override
    public void loop() {

        //set motor controls to joystick
        lDrive.setPower(gamepad1.left_stick_y*.7);
        rDrive.setPower(gamepad1.right_stick_y*.7);


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

        //shows values from color sensor on driver station phone
        telemetry.addData("Distance value",ods.getLightDetected());
        telemetry.addData("Light", light.getLightDetected());
        telemetry.addData("Red ", colorSensor.red());
        telemetry.addData("Green ", colorSensor.green());
        telemetry.addData("Blue ", colorSensor.blue());
    }
}
