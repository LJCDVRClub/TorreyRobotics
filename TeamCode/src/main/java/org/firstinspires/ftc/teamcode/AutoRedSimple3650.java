package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Bryce on 12/2/2016.
 */
@Autonomous(name = "Automagical Red (Simple)", group = "3650")
public class AutoRedSimple3650 extends LinearOpMode{
    DcMotor lDrive, rDrive, collector, shooter;

    @Override
    public void runOpMode() throws InterruptedException {
        lDrive = hardwareMap.dcMotor.get("lDrive");
        rDrive = hardwareMap.dcMotor.get("rDrive");
        collector = hardwareMap.dcMotor.get("collector");
        shooter = hardwareMap.dcMotor.get("shooter");

        lDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        //sets motors for distance
        rDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //move to shooting position
        rDrive.setTargetPosition(rDrive.getCurrentPosition()+1650);
        lDrive.setTargetPosition(lDrive.getCurrentPosition()+1650);
        rDrive.setPower(.4);
        lDrive.setPower(.4);




        //start spinning up shooter

        shooter.setPower(1.00);
        //sleep (wait) so it can finish moving
        Thread.sleep(3300);

        lDrive.setPower(0);
        rDrive.setPower(0);

        //spin up collector
        collector.setPower(-1.00);

        Thread.sleep(750);
        collector.setPower(0);
        Thread.sleep(1500);
        collector.setPower(-1.00);
        Thread.sleep(1500);

        //spin down motors
        collector.setPower(0);
        shooter.setPower(.4);
        Thread.sleep(2000);
        shooter.setPower(0);

        //do a 360
        rDrive.setTargetPosition(rDrive.getCurrentPosition()-1550);
        lDrive.setTargetPosition(lDrive.getCurrentPosition()+1550);
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
