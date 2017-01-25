package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Bryce on 1/20/2017.
 */
@Autonomous(name = "Automagical: Blue", group = "3650")
public class Automagical_BLUE3650 extends LinearOpMode {
    ColorSensor colorSensor;
    LightSensor light;
    Servo forePush, aftPush;
    DcMotor lDrive, rDrive, collector, shooter;
    double lThresh, aftNeutral, foreNeutral;


    @Override
    public void runOpMode() throws InterruptedException {

        lThresh = 0.090; //anything higher is white


        //rest positions for servos
        aftNeutral = .1;
        foreNeutral = 1;

        //button pushing servos
        forePush = hardwareMap.servo.get("forePush");
        aftPush = hardwareMap.servo.get("aftPush");

        lDrive = hardwareMap.dcMotor.get("lDrive");
        rDrive = hardwareMap.dcMotor.get("rDrive");
        collector = hardwareMap.dcMotor.get("collector");
        shooter = hardwareMap.dcMotor.get("shooter");

        colorSensor = hardwareMap.colorSensor.get("colorSensor");

        colorSensor.enableLed(false);
        light = hardwareMap.lightSensor.get("light");


        lDrive.setDirection(DcMotor.Direction.REVERSE);

        //set servos to rest position
        forePush.setPosition(foreNeutral);
        aftPush.setPosition(aftNeutral);


        waitForStart(); //waits for start button to be pressed

        //sets motors to run with encoder
        lDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //move to shooting position
        rDrive.setTargetPosition(rDrive.getCurrentPosition() + 1200);
        lDrive.setTargetPosition(lDrive.getCurrentPosition() + 1220);
        rDrive.setPower(.4);
        lDrive.setPower(.4);

        //spin up shooter
        shooter.setPower(1.00);

        Thread.sleep(2500);

        //stop and start shooting
        lDrive.setPower(0);
        rDrive.setPower(0);

        //start shooting
        collector.setPower(-1.00);
        Thread.sleep(2500);

        //wait for shooter to speed down
        collector.setPower(0);
        shooter.setPower(.4);
        Thread.sleep(1000);
        shooter.setPower(0);

        rDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //spin towards right wall
        lDrive.setTargetPosition(lDrive.getCurrentPosition() + 620);
        rDrive.setTargetPosition(rDrive.getCurrentPosition() + 2300);
        lDrive.setPower(.4);
        rDrive.setPower(.4);

        Thread.sleep(4000);

        rDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //drive into wall (almost)
        lDrive.setTargetPosition(lDrive.getCurrentPosition() - 3000);
        rDrive.setTargetPosition(rDrive.getCurrentPosition() - 3000);
        lDrive.setPower(.35);
        rDrive.setPower(.35);

        Thread.sleep(2750);

        rDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //spin right to be parallel with beacons
        rDrive.setTargetPosition(rDrive.getCurrentPosition() + 930);
        lDrive.setTargetPosition(lDrive.getCurrentPosition() - 770);
        rDrive.setPower(.3);
        lDrive.setPower(.3);
        Thread.sleep(2000);

        //sets motors back to normal mode
        rDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //drive slowly in order to detect white line
        rDrive.setPower(-.17);
        lDrive.setPower(-.17);

        //while the line is not detected ...
        while (light.getLightDetected() < lThresh) {
            continue;
        }
        //stop once detected
        rDrive.setPower(0);
        lDrive.setPower(0);
        Thread.sleep(1000);

        //detect if red
        if (colorSensor.red() < colorSensor.blue()) {
            //hit button with servo
            aftPush.setPosition(1.0);
            Thread.sleep(1500);
            //bring back servo
            aftPush.setPosition(aftNeutral);
        }//if not, use other servo
        else if (colorSensor.blue() <= colorSensor.red()) {
            //hit button with other servo
            forePush.setPosition(0);
            Thread.sleep(1500);
            //bring servo back
            forePush.setPosition(foreNeutral);
        } else {
            //run away
        }


        //repeat
        rDrive.setPower(-.17);
        lDrive.setPower(-.17);
        Thread.sleep(1500);

        while (light.getLightDetected() < lThresh) {
            continue;
        }
        rDrive.setPower(0);
        lDrive.setPower(0);
        Thread.sleep(1000);

        if (colorSensor.red() < colorSensor.blue()) {
            //hit button with servo
            aftPush.setPosition(1.0);
            Thread.sleep(1500);
            //bring back servo
            aftPush.setPosition(aftNeutral);
        } else if (colorSensor.blue() <= colorSensor.red()) {
            //hit button with other servo
            forePush.setPosition(0);
            Thread.sleep(1500);
            //bring servo back
            forePush.setPosition(foreNeutral);
        } else {
            //run away
        }
        Thread.sleep(1000);
    }
}
