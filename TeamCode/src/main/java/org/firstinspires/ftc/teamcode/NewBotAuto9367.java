/*package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.LightSensor;

import java.util.ArrayList;
import java.util.Set;

@Autonomous(name="NewBotAuto9367", group = "9367")
public class NewBotAuto9367 extends LinearOpMode {

    DcMotor flDrive, frDrive, rlDrive, rrDrive, collector, shooter;
    Servo ballServo;
    ColorSensor colorSensor;
    LightSensor lightSensor;

    boolean releaseOneBall = false;
    double ballServoBlockPosition = 0;
    double ballServoReleasePosition = 1;
    long setTime = 0;
    long ballServoReleaseTime = 1000;
    long ballServoReleaseWaitTime = 500;
    long shooterResetTime = 1000;
    long shooterShootingTime = 1500;
    double lightSensorThreshold = 0.11;



    @Override
    public void runOpMode() throws InterruptedException {
        flDrive = hardwareMap.dcMotor.get("flDrive");
        frDrive = hardwareMap.dcMotor.get("frDrive");
        rlDrive = hardwareMap.dcMotor.get("rlDrive");
        rrDrive = hardwareMap.dcMotor.get("rrDrive");
        //   collector = hardwareMap.dcMotor.get("collector");
        //   shooter = hardwareMap.dcMotor.get("shooter");
        //   ballServo = hardwareMap.servo.get("ballServo");
        //   colorSensor = hardwareMap.colorSensor.get("colorSensor");
        //    lightSensor = hardwareMap.lightSensor.get("lightSensor");
        MoveUsingEncoder moveEnco = new MoveUsingEncoder();

        waitForStart();

        SetAllVariablesToDefault();

        /*

        releaseOneBall = false;
        ShootBall(releaseOneBall);

        Thread.sleep(500);

        releaseOneBall = true;
        ShootBall(releaseOneBall);

        Thread.sleep(500);


        //test encoders for spinning
        double[] clockwise = {0, 0, 0, 0};
        moveEnco.ClockwiseTurnEncoder(0.2, 1000);
        clockwise = RecordCurrentPosition();

        telemetry.addData("Clockwise fl", clockwise[0]);
        telemetry.addData("Clockwise fr", clockwise[1]);
        telemetry.addData("Clockwise rl", clockwise[2]);
        telemetry.addData("Clockwise rr", clockwise[3]);

        Thread.sleep(3000);

        double[] counterClockwise = {0, 0, 0, 0};
        moveEnco.CounterClockwiseTurnEncoder(0.2, 1000);
        counterClockwise = RecordCurrentPosition();

        telemetry.addData("Counterclockwise fl", counterClockwise[0]);
        telemetry.addData("Counterclockwise fr", counterClockwise[1]);
        telemetry.addData("Counterclockwise rl", counterClockwise[2]);
        telemetry.addData("Counterclockwise rr", counterClockwise[3]);



        moveEnco.RightEncoder(0.4,1000);
        moveEnco.ForwardEncoder(0.4, 1000);


    }


    //Below are functions
    void SetAllVariablesToDefault() {
        colorSensor.enableLed(false);
        ballServo.setPosition(ballServoBlockPosition);
        flDrive.setPower(0);
        frDrive.setPower(0);
        rlDrive.setPower(0);
        rrDrive.setPower(0);
        shooter.setPower(0);
        collector.setPower(0);
    }


    void SetAllDrivesToRunToPosition() {
        flDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rlDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rrDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    void SetAllDrivesToUsingEncoders() {
        flDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rlDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rrDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    void StopDrivesIfFinished() {
        while (!flDrive.isBusy() && !frDrive.isBusy() && !rlDrive.isBusy() && !rrDrive.isBusy()) {
            flDrive.setPower(0);
            frDrive.setPower(0);
            rlDrive.setPower(0);
            rrDrive.setPower(0);
        }
    }


    void WaitFor(long time) {
        setTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - setTime < time) {
            continue;
        }
    }


    void ShootBall(boolean releaseBall) {

        SetAllVariablesToDefault();

        if (releaseBall) {

            ballServo.setPosition(ballServoReleasePosition);
            collector.setPower(1);

            //wait until time is reached
            WaitFor(ballServoReleaseTime);
            ballServo.setPosition(ballServoBlockPosition);
            collector.setPower(0);

            //wait until ball is in position for shooting
            WaitFor(ballServoReleaseWaitTime);

            shooter.setPower(-0.5);

            WaitFor(shooterResetTime);

            shooter.setPower(1);

            WaitFor(shooterShootingTime);

            shooter.setPower(0);
        }


        //If a ball is already on the ramp
        else {

            shooter.setPower(-0.5);
            WaitFor(shooterResetTime);
            shooter.setPower(1);
            WaitFor(shooterShootingTime);
            shooter.setPower(0);

        }
    }


    void Forward(double power) {

        SetAllVariablesToDefault();
        SetAllDrivesToRunToPosition();

        flDrive.setPower(power);
        frDrive.setPower(power);
        rlDrive.setPower(power);
        rrDrive.setPower(power);

    }

    void Backward(double power) {

        SetAllVariablesToDefault();
        SetAllDrivesToRunToPosition();

        flDrive.setPower(-power);
        frDrive.setPower(-power);
        rlDrive.setPower(-power);
        rrDrive.setPower(-power);

    }

    void Left(double power) {

        SetAllVariablesToDefault();
        SetAllDrivesToRunToPosition();

        flDrive.setPower(-power);
        frDrive.setPower(power);
        rlDrive.setPower(power);
        rrDrive.setPower(-power);


    }

    void Right(double power) {

        SetAllVariablesToDefault();
        SetAllDrivesToRunToPosition();

        flDrive.setPower(power);
        frDrive.setPower(-power);
        rlDrive.setPower(-power);
        rrDrive.setPower(power);


    }

    void ClockwiseTurn(double power) {

        SetAllVariablesToDefault();
        SetAllDrivesToRunToPosition();

        flDrive.setPower(power);
        frDrive.setPower(-power);
        rlDrive.setPower(power);
        rrDrive.setPower(-power);


    }

    void CounterClockwiseTurn(double power) {

        SetAllVariablesToDefault();
        SetAllDrivesToRunToPosition();

        flDrive.setPower(-power);
        frDrive.setPower(power);
        rlDrive.setPower(-power);
        rrDrive.setPower(power);


    }


/*
    public class MoveUsingEncoder{

        MoveUsingEncoder() {
            SetAllVariablesToDefault();
            SetAllDrivesToRunToPosition();
        }

        void ForwardEncoder(double power, int distance){

            int flDriveMovement = flDrive.getCurrentPosition() + distance;
            int frDriveMovement = frDrive.getCurrentPosition() + distance;
            int rlDriveMovement = rlDrive.getCurrentPosition() + distance;
            int rrDriveMovement = rrDrive.getCurrentPosition() + distance;

            double flPower = power;
            double frPower = power;
            double rlPower = power;
            double rrPower = power;

            flDrive.setTargetPosition(flDriveMovement);
            frDrive.setTargetPosition(frDriveMovement);
            rlDrive.setTargetPosition(rlDriveMovement);
            rrDrive.setTargetPosition(rrDriveMovement);

            flDrive.setPower(flPower);
            frDrive.setPower(frPower);
            rlDrive.setPower(rlPower);
            rrDrive.setPower(rrPower);

            StopDrivesIfFinished();
        }


        void BackwardEncoder(double power, int distance){

            int flDriveMovement = flDrive.getCurrentPosition() - distance;
            int frDriveMovement = frDrive.getCurrentPosition() - distance;
            int rlDriveMovement = rlDrive.getCurrentPosition() - distance;
            int rrDriveMovement = rrDrive.getCurrentPosition() - distance;

            double flPower = -power;
            double frPower = -power;
            double rlPower = -power;
            double rrPower = -power;

            flDrive.setTargetPosition(flDriveMovement);
            frDrive.setTargetPosition(frDriveMovement);
            rlDrive.setTargetPosition(rlDriveMovement);
            rrDrive.setTargetPosition(rrDriveMovement);

            flDrive.setPower(flPower);
            frDrive.setPower(frPower);
            rlDrive.setPower(rlPower);
            rrDrive.setPower(rrPower);

            StopDrivesIfFinished();
        }

        void LeftEncoder(double power, int distance){

            int flDriveMovement = flDrive.getCurrentPosition() - distance;
            int frDriveMovement = frDrive.getCurrentPosition() + distance;
            int rlDriveMovement = rlDrive.getCurrentPosition() + distance;
            int rrDriveMovement = rrDrive.getCurrentPosition() - distance;

            double flPower = -power;
            double frPower = +power;
            double rlPower = +power;
            double rrPower = -power;

            flDrive.setTargetPosition(flDriveMovement);
            frDrive.setTargetPosition(frDriveMovement);
            rlDrive.setTargetPosition(rlDriveMovement);
            rrDrive.setTargetPosition(rrDriveMovement);

            flDrive.setPower(flPower);
            frDrive.setPower(frPower);
            rlDrive.setPower(rlPower);
            rrDrive.setPower(rrPower);

            StopDrivesIfFinished();

        }

        void RightEncoder(double power, int distance){

            int flDriveMovement = flDrive.getCurrentPosition() + distance;
            int frDriveMovement = frDrive.getCurrentPosition() - distance;
            int rlDriveMovement = rlDrive.getCurrentPosition() - distance;
            int rrDriveMovement = rrDrive.getCurrentPosition() + distance;

            double flPower = power;
            double frPower = -power;
            double rlPower = -power;
            double rrPower = power;

            flDrive.setTargetPosition(flDriveMovement);
            frDrive.setTargetPosition(frDriveMovement);
            rlDrive.setTargetPosition(rlDriveMovement);
            rrDrive.setTargetPosition(rrDriveMovement);

            flDrive.setPower(flPower);
            frDrive.setPower(frPower);
            rlDrive.setPower(rlPower);
            rrDrive.setPower(rrPower);

            StopDrivesIfFinished();

        }

        void ClockwiseTurnEncoder(double power, int distance){

            int flDriveMovement = flDrive.getCurrentPosition() + distance;
            int frDriveMovement = frDrive.getCurrentPosition() - distance;
            int rlDriveMovement = rlDrive.getCurrentPosition() + distance;
            int rrDriveMovement = rrDrive.getCurrentPosition() - distance;

            double flPower = -power;
            double frPower = power;
            double rlPower = -power;
            double rrPower = power;

            flDrive.setTargetPosition(flDriveMovement);
            frDrive.setTargetPosition(frDriveMovement);
            rlDrive.setTargetPosition(rlDriveMovement);
            rrDrive.setTargetPosition(rrDriveMovement);

            flDrive.setPower(flPower);
            frDrive.setPower(frPower);
            rlDrive.setPower(rlPower);
            rrDrive.setPower(rrPower);

            StopDrivesIfFinished();

        }

        void CounterClockwiseTurnEncoder(double power, int distance){

            int flDriveMovement = flDrive.getCurrentPosition() - distance;
            int frDriveMovement = frDrive.getCurrentPosition() + distance;
            int rlDriveMovement = rlDrive.getCurrentPosition() - distance;
            int rrDriveMovement = rrDrive.getCurrentPosition() + distance;

            double flPower = -power;
            double frPower = power;
            double rlPower = -power;
            double rrPower = power;


            flDrive.setTargetPosition(flDriveMovement);
            frDrive.setTargetPosition(frDriveMovement);
            rlDrive.setTargetPosition(rlDriveMovement);
            rrDrive.setTargetPosition(rrDriveMovement);

            flDrive.setPower(flPower);
            frDrive.setPower(frPower);
            rlDrive.setPower(rlPower);
            rrDrive.setPower(rrPower);

            StopDrivesIfFinished();
        }

        void CombineMovements(ArrayList<MoveUsingEncoder> ary){

            int flDriveMovement = 0;
            int frDriveMovement = 0;
            int rlDriveMovement = 0;
            int rrDriveMovement = 0;

            double flPower = 0;
            double frPower = 0;
            double rlPower = 0;
            double rrPower = 0;

            for(int i = 0; i < ary.size(); i++){
                flDriveMovement += ary.get(i).;
            }
        }

    }


    void startToMove(double flPower, double frPower, double rlPower, double rrPower, int flDriveMovement, int frDriveMovement, int rlDriveMovement, int rrDriveMovement){
        flDrive.setTargetPosition(flDriveMovement);
        frDrive.setTargetPosition(frDriveMovement);
        rlDrive.setTargetPosition(rlDriveMovement);
        rrDrive.setTargetPosition(rrDriveMovement);

        flDrive.setPower(flPower);
        frDrive.setPower(frPower);
        rlDrive.setPower(rlPower);
        rrDrive.setPower(rrPower);
    }


    public class MoveUsingEncoder{

        int flDriveMovement, frDriveMovement, rlDriveMovement, rrDriveMovement;
        double flPower, frPower, rlPower, rrPower;

        MoveUsingEncoder() {
            SetAllVariablesToDefault();
            SetAllDrivesToRunToPosition();
        }

        MoveUsingEncoder(String direction, double inputPower, int inputDistance){

            SetAllVariablesToDefault();
            SetAllDrivesToRunToPosition();

            if(direction.equalsIgnoreCase("forward")){
                flDriveMovement = flDrive.getCurrentPosition() + inputDistance;
                frDriveMovement = frDrive.getCurrentPosition() + inputDistance;
                rlDriveMovement = rlDrive.getCurrentPosition() + inputDistance;
                rrDriveMovement = rrDrive.getCurrentPosition() + inputDistance;

                flPower = inputPower;
                frPower = inputPower;
                rlPower = inputPower;
                rrPower = inputPower;

                startToMove(flPower, frPower, rlPower, rrPower, flDriveMovement, frDriveMovement, rlDriveMovement, rrDriveMovement);
            }

            if(direction.equalsIgnoreCase("backward")){
                flDriveMovement = flDrive.getCurrentPosition() - inputDistance;
                frDriveMovement = frDrive.getCurrentPosition() - inputDistance;
                rlDriveMovement = rlDrive.getCurrentPosition() - inputDistance;
                rrDriveMovement = rrDrive.getCurrentPosition() - inputDistance;

                flPower = -inputPower;
                frPower = -inputPower;
                rlPower = -inputPower;
                rrPower = -inputPower;

                startToMove(flPower, frPower, rlPower, rrPower, flDriveMovement, frDriveMovement, rlDriveMovement, rrDriveMovement);
            }

            if(direction.equalsIgnoreCase("left")){
                flDriveMovement = flDrive.getCurrentPosition() - inputDistance;
                frDriveMovement = frDrive.getCurrentPosition() + inputDistance;
                rlDriveMovement = rlDrive.getCurrentPosition() + inputDistance;
                rrDriveMovement = rrDrive.getCurrentPosition() - inputDistance;

                flPower = -inputPower;
                frPower = inputPower;
                rlPower = inputPower;
                rrPower = -inputPower;

                startToMove(flPower, frPower, rlPower, rrPower, flDriveMovement, frDriveMovement, rlDriveMovement, rrDriveMovement);
            }

            if(direction.equalsIgnoreCase("right")){
                flDriveMovement = flDrive.getCurrentPosition() + inputDistance;
                frDriveMovement = frDrive.getCurrentPosition() - inputDistance;
                rlDriveMovement = rlDrive.getCurrentPosition() - inputDistance;
                rrDriveMovement = rrDrive.getCurrentPosition() + inputDistance;

                flPower = inputPower;
                frPower = -inputPower;
                rlPower = -inputPower;
                rrPower = inputPower;

                startToMove(flPower, frPower, rlPower, rrPower, flDriveMovement, frDriveMovement, rlDriveMovement, rrDriveMovement);
            }

            if(direction.equalsIgnoreCase("clockwise")){
                flDriveMovement = flDrive.getCurrentPosition() + inputDistance;
                frDriveMovement = frDrive.getCurrentPosition() - inputDistance;
                rlDriveMovement = rlDrive.getCurrentPosition() + inputDistance;
                rrDriveMovement = rrDrive.getCurrentPosition() - inputDistance;

                flPower = inputPower;
                frPower = -inputPower;
                rlPower = inputPower;
                rrPower = -inputPower;

                startToMove(flPower, frPower, rlPower, rrPower, flDriveMovement, frDriveMovement, rlDriveMovement, rrDriveMovement);
            }

            if(direction.equalsIgnoreCase("counterclockwise")){
                flDriveMovement = flDrive.getCurrentPosition() - inputDistance;
                frDriveMovement = frDrive.getCurrentPosition() + inputDistance;
                rlDriveMovement = rlDrive.getCurrentPosition() - inputDistance;
                rrDriveMovement = rrDrive.getCurrentPosition() + inputDistance;

                flPower = -inputPower;
                frPower = inputPower;
                rlPower = -inputPower;
                rrPower = inputPower;

                startToMove(flPower, frPower, rlPower, rrPower, flDriveMovement, frDriveMovement, rlDriveMovement, rrDriveMovement);
            }

            StopDrivesIfFinished();
        }





        void ClockwiseTurnEncoder(double power, int distance){

            int flDriveMovement = flDrive.getCurrentPosition() + distance;
            int frDriveMovement = frDrive.getCurrentPosition() - distance;
            int rlDriveMovement = rlDrive.getCurrentPosition() + distance;
            int rrDriveMovement = rrDrive.getCurrentPosition() - distance;

            double flPower = -power;
            double frPower = power;
            double rlPower = -power;
            double rrPower = power;

            flDrive.setTargetPosition(flDriveMovement);
            frDrive.setTargetPosition(frDriveMovement);
            rlDrive.setTargetPosition(rlDriveMovement);
            rrDrive.setTargetPosition(rrDriveMovement);

            flDrive.setPower(flPower);
            frDrive.setPower(frPower);
            rlDrive.setPower(rlPower);
            rrDrive.setPower(rrPower);

            StopDrivesIfFinished();

        }

        void CounterClockwiseTurnEncoder(double power, int distance){

            int flDriveMovement = flDrive.getCurrentPosition() - distance;
            int frDriveMovement = frDrive.getCurrentPosition() + distance;
            int rlDriveMovement = rlDrive.getCurrentPosition() - distance;
            int rrDriveMovement = rrDrive.getCurrentPosition() + distance;

            double flPower = -power;
            double frPower = power;
            double rlPower = -power;
            double rrPower = power;


            flDrive.setTargetPosition(flDriveMovement);
            frDrive.setTargetPosition(frDriveMovement);
            rlDrive.setTargetPosition(rlDriveMovement);
            rrDrive.setTargetPosition(rrDriveMovement);

            flDrive.setPower(flPower);
            frDrive.setPower(frPower);
            rlDrive.setPower(rlPower);
            rrDrive.setPower(rrPower);

            StopDrivesIfFinished();
        }

        void CombineMovements(ArrayList<MoveUsingEncoder> ary){

            int flDriveMovement = 0;
            int frDriveMovement = 0;
            int rlDriveMovement = 0;
            int rrDriveMovement = 0;

            double flPower = 0;
            double frPower = 0;
            double rlPower = 0;
            double rrPower = 0;

            for(int i = 0; i < ary.size(); i++){
                flDriveMovement += ary.get(i).;
            }
        }

    }











    double[] RecordCurrentPosition(){
        double[] ary = {flDrive.getCurrentPosition(), frDrive.getCurrentPosition(), rlDrive.getCurrentPosition(), rrDrive.getCurrentPosition()};
        return ary;
    }


    boolean AreAbsoluteValueSimilar(double[] a, double[] b){

        double sumOfDifference = 0;
        double positiveThreshold = 5; //guessed
        double negativeThreshold = -5; //guessed

        for(int i = 0; i < a.length; i++){
            sumOfDifference += (Math.abs(a[i]) - Math.abs(b[i]));
        }

        if(sumOfDifference/4 < positiveThreshold && sumOfDifference/4 > negativeThreshold){
            return true;
        }

        else{
            return false;
        }

    }


    double[] ArrayAddition(double[] a, double[] b){
        double[] sum = new double[a.length];
        for(int i = 0; i < sum.length; i++){
            sum[i] = a[i] + b[i];
        }
        return sum;
    }




    void SearchForWhiteLine(boolean ifRedTeam){

        SetAllVariablesToDefault();
        SetAllDrivesToUsingEncoders();
        double[] initialPosition = RecordCurrentPosition();
        double[] maxMovement = {800,800,800,800};

        if(ifRedTeam){
            while(!AreAbsoluteValueSimilar(RecordCurrentPosition(), ArrayAddition(initialPosition, maxMovement)) || lightSensor.getLightDetected() < lightSensorThreshold){
                Backward(0.1);
            }


        }


    }

}*/