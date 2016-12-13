package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Bryce on 12/2/2016.
 */
@Autonomous(name="Automagical Blue (Simple)", group = "3650")
public class AutoBlueSimple3650 extends LinearOpMode {

    /**
    * COMMENTS ABOUT WHAT'S GOING ON IN AUTO RED (SIMPLE)
     * They are basically the same thing, but the spin direction is reversed
     */

    Servo colorServo, ballServo;
    DcMotor lDrive, rDrive, collector, shooter;

    @Override
    public void runOpMode() throws InterruptedException {
        lDrive = hardwareMap.dcMotor.get("lDrive");
        rDrive = hardwareMap.dcMotor.get("rDrive");
        collector = hardwareMap.dcMotor.get("collector");
        shooter = hardwareMap.dcMotor.get("shooter");
        colorServo = hardwareMap.servo.get("colorServo");
        ballServo =hardwareMap.servo.get("ballServo");
        lDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        //shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        //sets motors for distance
        rDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //move to shooting position
        rDrive.setTargetPosition(rDrive.getCurrentPosition()+1500);
        lDrive.setTargetPosition(lDrive.getCurrentPosition()+1500);
        rDrive.setPower(.4);
        lDrive.setPower(.4);

        //sleep (wait) so it can finish moving


        //start spinning up shooter

        shooter.setPower(1.00);
        Thread.sleep(3300);
        lDrive.setPower(0);
        rDrive.setPower(0);

        //spin up collector
        collector.setPower(-1.00);
        //Thread.sleep(750);
        //move flippy servo up and shoot
        //ballServo.setPosition(1.00);
        Thread.sleep(1000);
        collector.setPower(0);
        Thread.sleep(500);
        collector.setPower(-1.00);
        Thread.sleep(1000);
        //reset servo and spin down motors
        //ballServo.setPosition(0.14);
        collector.setPower(0);
        shooter.setPower(.4);
        Thread.sleep(2000);
        shooter.setPower(0);

        //do a 360
        lDrive.setTargetPosition(lDrive.getCurrentPosition()-2500);
        rDrive.setTargetPosition(rDrive.getCurrentPosition()+2500);
        rDrive.setPower(.5);
        lDrive.setPower(.5);
        Thread.sleep(3500);

        //ram the ball!!!! (and hopefully not the center post)
        rDrive.setPower(.4);
        lDrive.setPower(.4);
        rDrive.setTargetPosition(rDrive.getCurrentPosition()-2800);
        lDrive.setTargetPosition(lDrive.getCurrentPosition()-2800);
        Thread.sleep(5000);



    }
}
