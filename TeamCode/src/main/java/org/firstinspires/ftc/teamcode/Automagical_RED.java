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
public class Automagical_RED extends LinearOpMode{

    ColorSensor colorSensor;
    OpticalDistanceSensor ods;
    LightSensor light;
    Servo colorServo;
    DcMotor lDrive, rDrive, collector, shooter;
    double lThresh, dThresh;

    @Override
    public void runOpMode() throws InterruptedException {

        lThresh = 0.25; //anything higher is white
        double perfectLight = .25; //check this!!  then replace it with lThresh
        dThresh = 0.0015; //needed distance for CS

        lDrive = hardwareMap.dcMotor.get("lDrive");
        rDrive = hardwareMap.dcMotor.get("rDrive");
        collector = hardwareMap.dcMotor.get("collector");
        shooter = hardwareMap.dcMotor.get("shooter");
        colorServo = hardwareMap.servo.get("colorServo");
        colorSensor = hardwareMap.colorSensor.get("colorSensor");
        ods = hardwareMap.opticalDistanceSensor.get("ods");
        colorSensor.enableLed(false);
        light = hardwareMap.lightSensor.get("light");

        //backwards is forwards
        rDrive.setDirection(DcMotor.Direction.REVERSE);
        colorServo.setPosition(1.00);
        //shooter.setDirection(DcMotor.Direction.REVERSE);



        waitForStart(); //waits for start button to be pressed

        rDrive.setPower(-.25);
        lDrive.setPower(-.25);

        while (light.getLightDetected() < lThresh){
            continue;
        }
        rDrive.setPower(0);
        lDrive.setPower(0);
        Thread.sleep(750);

        while (light.getLightDetected() < lThresh){
          rDrive.setPower(.25);
        }

        while (ods.getLightDetected() < dThresh){
          double correction = (perfectLight - light.getLightDetected());

          if(correction <= 0 ){
            lDrive.setPower(.25);
            rDrive.setPower(.25 - correction);
          }
          else {
            lDrive.setPower(.25 + correction);
            rDrive.setPower(.25);
          }
        }
        rDrive.setPower(0);
        lDrive.setPower(0);

        Thread.sleep(750);

        if(colorSensor.red() > colorSensor.blue()){
          rDrive.setPower(.3);
          lDrive.setPower(.3);
        }
        else {
          colorServo.setPosition(-1.00);
          Thread.sleep(750);
          rDrive.setPower(.3);
          lDrive.setPower(.3);
        }

        Thread.sleep(250);
        rDrive.setPower(-1.00);
        lDrive.setPower(-1.00);
        Thread.sleep(500);

        rDrive.setPower(0);
        lDrive.setPower(0);



    }
}
