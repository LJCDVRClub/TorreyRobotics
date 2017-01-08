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
public class Automagical_BLUE3650 extends LinearOpMode{

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

        rDrive.setTargetPosition(rDrive.getCurrentPosition()+1500);
        lDrive.setTargetPosition(lDrive.getCurrentPosition()+1500);
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

        //wait for shooter to speed down
        collector.setPower(0);
        shooter.setPower(.4);
        Thread.sleep(1000);
        shooter.setPower(0);

        lDrive.setTargetPosition(lDrive.getCurrentPosition()+400);
        rDrive.setTargetPosition(rDrive.getCurrentPosition()+2000);
        lDrive.setPower(.4);
        rDrive.setPower(.4);

        Thread.sleep(4500);


        rDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        lDrive.setPower(.2);
        rDrive.setPower(.2);

        while(ods.getLightDetected() <= 0){
            continue;
        }

        lDrive.setPower(0);
        rDrive.setPower(0);

        Thread.sleep(750);


        rDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rDrive.setTargetPosition(rDrive.getCurrentPosition()-1100);
        lDrive.setTargetPosition(lDrive.getCurrentPosition()+650);
        rDrive.setPower(.3);
        lDrive.setPower(.3);
        Thread.sleep(2000);

        rDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rDrive.setPower(.2);
        lDrive.setPower(.2);

        while(light.getLightDetected() < lThresh){
            continue;
        }
        rDrive.setPower(0);
        lDrive.setPower(0);
        Thread.sleep(2000);

        if(colorSensor.blue() > colorSensor.red()){
            //hit button with servo
            aftPush.setPosition(aftNeutral - .4);
            Thread.sleep(2500);
            //bring back servo
            aftPush.setPosition(aftNeutral);
        }
        else if(colorSensor.red() >= colorSensor.blue()){
            //hit button with other servo
            forePush.setPosition(foreNeutral + .4);
            Thread.sleep(2500);
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
