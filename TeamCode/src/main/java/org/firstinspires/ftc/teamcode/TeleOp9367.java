package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
/**
 * Created by williamhuang on 14/12/16.
 */

//TODO: put more comments!! I have no idea what is going on
//I think you need to have different names in .get for the different drives
//-Bryce

@TeleOp(name="TeleOp 9367", group="9367")
public class TeleOp9367 extends OpMode{
    //assigning state variables
    DcMotor rfDrive, lfDrive, rrDrive, lrDrive;



    @Override
    public void init() {

        // linking variables to hardware components
        lfDrive = hardwareMap.dcMotor.get("lfDrive");
        rfDrive = hardwareMap.dcMotor.get("rfDrive");
        lrDrive = hardwareMap.dcMotor.get("lrDrive");
        rrDrive = hardwareMap.dcMotor.get("rrDrive");





    }

    @Override
    public void loop() {

        if(gamepad1.dpad_left){
            lfDrive.setPower(1.0);
            lrDrive.setPower(-1.0);
            rfDrive.setPower(-1.0);
            rrDrive.setPower(1.0);
        }

        if(gamepad1.dpad_right){
            lfDrive.setPower(-1.0);
            lrDrive.setPower(1.0);
            rfDrive.setPower(1.0);
            rrDrive.setPower(-1.0);
        }

        if(gamepad1.dpad_up){
            lfDrive.setPower(1.0);
            lrDrive.setPower(1.0);
            rfDrive.setPower(1.0);
            rrDrive.setPower(1.0);
        }

        if(gamepad1.dpad_down){
            lfDrive.setPower(-1.0);
            lrDrive.setPower(-1.0);
            rfDrive.setPower(-1.0);
            rrDrive.setPower(-1.0);
        }

        lfDrive.setPower(-gamepad1.left_stick_y+0.5*gamepad1.left_stick_x);
        lrDrive.setPower(-gamepad1.left_stick_y+0.5*gamepad1.left_stick_x);
        rfDrive.setPower(gamepad1.left_stick_y+0.5*gamepad1.left_stick_x);
        rrDrive.setPower(gamepad1.left_stick_y+0.5*gamepad1.left_stick_x);


    }

}
