package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="NewBotTeleOp9367", group="9367")
public class NewBotTeleOp9367 extends OpMode {

    //assigning state variables
    DcMotor frDrive, flDrive, rrDrive, rlDrive;//, collector, shooter, elevator;
    double slowMotionFactor = 0.3, turningFactor = 0.7;



    @Override
    public void init() {

        // linking variables to hardware components
        flDrive = hardwareMap.dcMotor.get("flDrive");
        frDrive = hardwareMap.dcMotor.get("frDrive");
        rlDrive = hardwareMap.dcMotor.get("rlDrive");
        rrDrive = hardwareMap.dcMotor.get("rrDrive");

    }

    @Override
    public void loop() {

        frDrive.setDirection(DcMotorSimple.Direction.REVERSE);


        //Slow Motion Mode
        if(gamepad1.right_bumper) {
            flDrive.setPower(slowMotionFactor * ((gamepad1.left_stick_y - gamepad1.left_stick_x) + (gamepad1.right_stick_y - turningFactor * gamepad1.right_stick_x)));
            frDrive.setPower(slowMotionFactor * ((gamepad1.left_stick_y + gamepad1.left_stick_x) + (gamepad1.right_stick_y + turningFactor * gamepad1.right_stick_x)));
            rlDrive.setPower(slowMotionFactor * ((gamepad1.left_stick_y + gamepad1.left_stick_x) + (gamepad1.right_stick_y - turningFactor * gamepad1.right_stick_x)));
            rrDrive.setPower(slowMotionFactor * ((gamepad1.left_stick_y - gamepad1.left_stick_x) + (gamepad1.right_stick_y + turningFactor * gamepad1.right_stick_x)));
        }


        //Normal Mode
        else {
            flDrive.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x) + (gamepad1.right_stick_y - turningFactor * gamepad1.right_stick_x));
            frDrive.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x) + (gamepad1.right_stick_y + turningFactor * gamepad1.right_stick_x));
            rlDrive.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x) + (gamepad1.right_stick_y - turningFactor * gamepad1.right_stick_x));
            rrDrive.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x) + (gamepad1.right_stick_y + turningFactor * gamepad1.right_stick_x));
        }








        //set shooter
    /*    if(gamepad1.a) {

            if (trigger) {
                setTime = System.currentTimeMillis();
                trigger = false;
            }

            else {
                if (System.currentTimeMillis() - setTime > 350 && System.currentTimeMillis() - setTime < 450) {
                    shooter.setPower(0);
                }
                else if (System.currentTimeMillis() - setTime > 450) {
                    shooter.setPower(1.0);
                }
                else {
                    shooter.setPower(-0.15);
                }
            }

        }


        else{
            shooter.setPower(0);
            trigger = true;
        }
*/




    }
}