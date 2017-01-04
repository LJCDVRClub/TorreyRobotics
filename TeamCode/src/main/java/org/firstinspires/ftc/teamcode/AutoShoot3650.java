package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by Bryce on 12/14/2016.
 */
@Autonomous(name = "Automagically Shoot", group = "3650")
public class AutoShoot3650 extends LinearOpMode {

    DcMotor rDrive, lDrive, shooter, collector;

    @Override
    public void runOpMode() throws InterruptedException {
        lDrive = hardwareMap.dcMotor.get("lDrive");
        rDrive = hardwareMap.dcMotor.get("rDrive");

        collector = hardwareMap.dcMotor.get("collector");
        shooter = hardwareMap.dcMotor.get("shooter");

        lDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        lDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart(); //wait for start button

        //move up to firing position
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
        Thread.sleep(750);

        //fire second ball
        collector.setPower(-1.00);
        Thread.sleep(1000);

        //stop collector, spin down shooter
        collector.setPower(0);
        shooter.setPower(.4);
        Thread.sleep(2000);

        shooter.setPower(0);

    }
}
