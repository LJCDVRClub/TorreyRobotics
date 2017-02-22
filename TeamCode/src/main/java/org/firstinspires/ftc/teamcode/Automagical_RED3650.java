package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;


@Autonomous(name = "Automagical: RED", group = "3650")
public class Automagical_RED3650 extends LinearOpMode{

    /*
    ColorSensor hw.colorSensor;
    LightSensor hw.light;
    TouchSensor hw.lTouch, hw.rTouch;
    Servo hw.forePush, hw.aftPush;
    DcMotor hw.lDrive, hw.hw.rDrive, hw.collector, hw.shooter;
    double lThresh, aftNeutral, foreNeutral;
    */


    Hardware_3650 hw = new Hardware_3650(hardwareMap);


    @Override
    public void runOpMode() throws InterruptedException {


        
        //commented out until Hardware class tested
        /*
        lThresh = 0.08; //anything higher is white


        //rest positions for servos
        aftNeutral = .1;
        foreNeutral = 1;

        //button pushing servos
        hw.forePush = hardwareMap.servo.get("hw.forePush");
        hw.aftPush = hardwareMap.servo.get("hw.aftPush");

        hw.lDrive = hardwareMap.dcMotor.get("hw.lDrive");
        hw.rDrive = hardwareMap.dcMotor.get("hw.rDrive");
        hw.collector = hardwareMap.dcMotor.get("hw.collector");
        hw.shooter = hardwareMap.dcMotor.get("hw.shooter");

        hw.colorSensor = hardwareMap.hw.colorSensor.get("hw.colorSensor");

        hw.colorSensor.enableLed(false);
        hw.light = hardwareMap.hw.lightSensor.get("hw.light");

        hw.lTouch = hardwareMap.touchSensor.get("hw.lTouch");
        hw.rTouch = hardwareMap.touchSensor.get("hw.rTouch");

        hw.lDrive.setDirection(DcMotor.Direction.REVERSE);

        //set servos to rest position
        hw.forePush.setPosition(foreNeutral);
        hw.aftPush.setPosition(aftNeutral);
        */


        waitForStart(); //waits for start button to be pressed

        //sets motors to run with encoder
        hw.rDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hw.lDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hw.lDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hw.rDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //move to shooting position
        hw.rDrive.setTargetPosition(hw.rDrive.getCurrentPosition()+1200);
        hw.lDrive.setTargetPosition(hw.lDrive.getCurrentPosition()+1220);
        hw.rDrive.setPower(.4);
        hw.lDrive.setPower(.4);

        //spin up hw.shooter
        hw.shooter.setPower(.9);

        Thread.sleep(1500);

        //stop and start shooting
        hw.lDrive.setPower(0);
        hw.rDrive.setPower(0);

        //start shooting
        hw.collector.setPower(-1.00);
        Thread.sleep(2500);

        //wait for hw.shooter to speed down
        hw.collector.setPower(0);
        hw.shooter.setPower(.4);
        Thread.sleep(1000);
        hw.shooter.setPower(0);


        //spin towards left wall
        hw.lDrive.setTargetPosition(hw.lDrive.getCurrentPosition()+600);
        hw.rDrive.setTargetPosition(hw.rDrive.getCurrentPosition()+2380);
        hw.lDrive.setPower(.4);
        hw.rDrive.setPower(.4);

        Thread.sleep(3000);

        hw.lDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hw.rDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //drive into wall

        //drive fast to save time
        hw.rDrive.setPower(.9);
        hw.lDrive.setPower(.9);
        Thread.sleep(500);

        //use touch sensors to be perpendicular to wall
        hw.lDrive.setPower(.16);
        hw.rDrive.setPower(.16);
        while(!(hw.rTouch.isPressed()) || !(hw.lTouch.isPressed())){
            if(hw.rTouch.isPressed()){
                hw.rDrive.setPower(0);
            }
            else if(hw.lTouch.isPressed()){
                hw.lDrive.setPower(0);
            }

        }
        hw.lDrive.setPower(0);
        hw.rDrive.setPower(0);

        Thread.sleep(500);

        hw.rDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hw.lDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hw.lDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hw.rDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        hw.rDrive.setTargetPosition(hw.rDrive.getCurrentPosition()-400);
        hw.lDrive.setTargetPosition(hw.lDrive.getCurrentPosition()-400);
        hw.rDrive.setPower(.4);
        hw.lDrive.setPower(.4);

        Thread.sleep(2000);


        //spin right to be parallel with beacons
        hw.rDrive.setTargetPosition(hw.rDrive.getCurrentPosition()-970);
        hw.lDrive.setTargetPosition(hw.lDrive.getCurrentPosition()+900);
        hw.rDrive.setPower(.3);
        hw.lDrive.setPower(.3);
        Thread.sleep(2000);

        //sets motors back to normal mode
        hw.rDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hw.lDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //drive slowly in order to detect white line
        hw.rDrive.setPower(.16);
        hw.lDrive.setPower(.16);

        //while the line is not detected ...
        while(hw.light.getLightDetected() < hw.lThresh){
            continue;
        }
        //stop once detected
        hw.rDrive.setPower(0);
        hw.lDrive.setPower(0);
        Thread.sleep(1000);

        //detect if red
        if(hw.colorSensor.red() > hw.colorSensor.blue()){
            //hit button with servo
            hw.aftPush.setPosition(1.0);
            Thread.sleep(1500);
            //bring back servo
            hw.aftPush.setPosition(hw.aftNeutral);
        }//if not, use other servo
        else if(hw.colorSensor.blue() >= hw.colorSensor.red()){
            //hit button with other servo
            hw.forePush.setPosition(0);
            Thread.sleep(1500);
            //bring servo back
            hw.forePush.setPosition(hw.foreNeutral);
        }
        else{
            //run away
        }



        //repeat
        hw.rDrive.setPower(.16);
        hw.lDrive.setPower(.16);
        Thread.sleep(3000);

        while(hw.light.getLightDetected() < .1){
            continue;
        }
        hw.rDrive.setPower(0);
        hw.lDrive.setPower(0);
        Thread.sleep(1000);

        if(hw.colorSensor.red() > hw.colorSensor.blue()){
            //hit button with servo
            hw.aftPush.setPosition(1.0);
            Thread.sleep(1500);
            //bring back servo
            hw.aftPush.setPosition(hw.aftNeutral);
        }
        else if(hw.colorSensor.blue() >= hw.colorSensor.red()){
            //hit button with other servo
            hw.forePush.setPosition(0);
            Thread.sleep(1500);
            //bring servo back
            hw.forePush.setPosition(hw.foreNeutral);
        }
        else{
            //run away
        }
        Thread.sleep(1000);





    }
}
