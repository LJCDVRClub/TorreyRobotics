package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;



@Autonomous(name = "NO WORKY! Automagical: Blue", group = "3650")
public class Automagical_RED3650 extends LinearOpMode{

    ColorSensor colorSensor;
    OpticalDistanceSensor ods;
    LightSensor light;
    Servo forePush, aftPush;
    DcMotor lDrive, rDrive, collector, shooter;
    double lThresh, dThresh, aftNeutral, foreNeutral;




    @Override
    public void runOpMode() throws InterruptedException {

        lThresh = 0.075; //anything higher is white
        double perfectLight = .25; //check this!!  then replace it with lThresh
        //dThresh = 0.0015; //needed distance for CS

        //rest positions for servos
        aftNeutral = 1.00;
        foreNeutral = .1;

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


        lDrive.setDirection(DcMotor.Direction.REVERSE);

        //shooter.setDirection(DcMotor.Direction.REVERSE);

        forePush.setPosition(foreNeutral);
        aftPush.setPosition(aftNeutral);



        waitForStart(); //waits for start button to be pressed

        lDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rDrive.setTargetPosition(rDrive.getCurrentPosition()+1400);
        lDrive.setTargetPosition(lDrive.getCurrentPosition()+1420);
        rDrive.setPower(.4);
        lDrive.setPower(.4);

        //spin up shooter
        //shooter.setPower(1.00);

        Thread.sleep(2500);

        lDrive.setPower(0);
        rDrive.setPower(0);

        //start shooting
        //collector.setPower(-1.00);
        Thread.sleep(1500);

        //wait for shooter to speed down
        collector.setPower(0);
        //shooter.setPower(.4);
        Thread.sleep(1000);
        shooter.setPower(0);

        lDrive.setTargetPosition(lDrive.getCurrentPosition()+300);
        rDrive.setTargetPosition(rDrive.getCurrentPosition()+2000);
        lDrive.setPower(.4);
        rDrive.setPower(.4);

        Thread.sleep(3500);



        lDrive.setTargetPosition(lDrive.getCurrentPosition()+2730);
        rDrive.setTargetPosition(rDrive.getCurrentPosition()+2730);
        lDrive.setPower(.35);
        rDrive.setPower(.35);

        Thread.sleep(2750);


        rDrive.setTargetPosition(rDrive.getCurrentPosition()-1070);
        lDrive.setTargetPosition(lDrive.getCurrentPosition()+600);
        rDrive.setPower(.3);
        lDrive.setPower(.3);
        Thread.sleep(2000);

        rDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rDrive.setPower(.12);
        lDrive.setPower(.12);

        while(light.getLightDetected() < lThresh){
            continue;
        }
        rDrive.setPower(0);
        lDrive.setPower(0);
        Thread.sleep(1500);

        if(colorSensor.red() > colorSensor.blue()){
            //hit button with servo
            aftPush.setPosition(aftNeutral - .4);
            Thread.sleep(2500);
            //bring back servo
            aftPush.setPosition(aftNeutral);
        }
        else if(colorSensor.blue() >= colorSensor.red()){
            //hit button with other servo
            forePush.setPosition(foreNeutral + .4);
            Thread.sleep(2500);
            //bring servo back
            forePush.setPosition(foreNeutral);
        }
        else{
            //run away
        }
        Thread.sleep(1000);


        rDrive.setPower(.15);
        lDrive.setPower(.15);
        Thread.sleep(500);

        while(light.getLightDetected() < lThresh){
            continue;
        }
        rDrive.setPower(0);
        lDrive.setPower(0);
        Thread.sleep(2000);

        if(colorSensor.red() > colorSensor.blue()){
            //hit button with servo
            aftPush.setPosition(aftNeutral - .4);
            Thread.sleep(2500);
            //bring back servo
            aftPush.setPosition(aftNeutral);
        }
        else if(colorSensor.blue() >= colorSensor.red()){
            //hit button with other servo
            forePush.setPosition(foreNeutral + .4);
            Thread.sleep(2500);
            //bring servo back
            forePush.setPosition(foreNeutral);
        }
        else{
            //run away
        }
        Thread.sleep(1000);

        //check to see how much time is left
        //if enough left, go for next beacon




    }
}
