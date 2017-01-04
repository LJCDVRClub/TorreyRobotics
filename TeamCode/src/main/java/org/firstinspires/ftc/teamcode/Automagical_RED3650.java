package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;


/**
 * Created by Bryce on 11/14/2016.
 */
@Autonomous(name = "Automagical: RED", group = "3650")
public class Automagical_RED3650 extends LinearOpMode{

    ColorSensor colorSensor;
    OpticalDistanceSensor ods;
    LightSensor light;
    Servo forePush, aftPush;
    DcMotor lDrive, rDrive, collector, shooter;
    double lThresh, dThresh, aftNeutral, foreNeutral;




    @Override
    public void runOpMode() throws InterruptedException {

        lThresh = 0.25; //anything higher is white
        double perfectLight = .25; //check this!!  then replace it with lThresh
        dThresh = 0.0015; //needed distance for CS

        //rest positions for servos
        aftNeutral = 0;
        foreNeutral = -.5;

        //button pushing servos
        forePush = hardwareMap.servo.get("forePush");
        aftPush = hardwareMap.servo.get("aftPush");

        lDrive = hardwareMap.dcMotor.get("lDrive");
        rDrive = hardwareMap.dcMotor.get("rDrive");
        collector = hardwareMap.dcMotor.get("collector");
        shooter = hardwareMap.dcMotor.get("shooter");

        colorSensor = hardwareMap.colorSensor.get("colorSensor");
        ods = hardwareMap.opticalDistanceSensor.get("ods");
        colorSensor.enableLed(false);
        light = hardwareMap.lightSensor.get("light");


        rDrive.setDirection(DcMotor.Direction.REVERSE);

        //shooter.setDirection(DcMotor.Direction.REVERSE);

        forePush.setPosition(foreNeutral);
        aftPush.setPosition(aftNeutral);



        waitForStart(); //waits for start button to be pressed

        lDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rDrive.setTargetPosition(rDrive.getCurrentPosition()+1100);
        lDrive.setTargetPosition(lDrive.getCurrentPosition()+1100);
        rDrive.setPower(.4);
        lDrive.setPower(.4);

        //spin up shooter
        shooter.setPower(1.00);

        Thread.sleep(2500);

        lDrive.setPower(0);
        rDrive.setPower(0);

        //start shooting
        collector.setPower(-1.00);
        Thread.sleep(1000);

        //wait for shooter to speed up again
        collector.setPower(0);
        shooter.setPower(.4);
        Thread.sleep(1000);
        shooter.setPower(0);

        rDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //backwards is forwards for some reason
        lDrive.setPower(-.5);
        rDrive.setPower(-.5);

        while(ods.getLightDetected() <= 0){
            continue;
        }

        lDrive.setPower(0);
        rDrive.setPower(0);

        Thread.sleep(750);

        rDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rDrive.setTargetPosition(rDrive.getCurrentPosition()-1200);
        lDrive.setTargetPosition(lDrive.getCurrentPosition()+1200);
        rDrive.setPower(.5);
        lDrive.setPower(.5);
        Thread.sleep(2000);

        rDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rDrive.setPower(.3);
        lDrive.setPower(.3);

        while(light.getLightDetected() < lThresh){
            continue;
        }
        rDrive.setPower(0);
        lDrive.setPower(0);

        if(colorSensor.red() > colorSensor.blue() && colorSensor.red() > 1){
            //hit button with servo
            aftPush.setPosition(aftNeutral + .7);
            Thread.sleep(1000);
            //bring back servo
            aftPush.setPosition(aftNeutral);
        }
        else if(colorSensor.blue() >= colorSensor.red()){
            //hit button with other servo
            forePush.setPosition(foreNeutral + .7);
            Thread.sleep(1000);
            //bring servo back
            forePush.setPosition(foreNeutral);
        }
        else{
            //run away
        }

        //check to see how much time is left
        //if enough left, go for next beacon




    }
}
