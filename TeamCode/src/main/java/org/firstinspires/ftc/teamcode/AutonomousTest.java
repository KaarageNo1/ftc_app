package org.firstinspires.ftc.teamcode;

import android.renderscript.ScriptIntrinsicResize;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;

/**
 * Created by Dani on 9/27/2017.
 */
@Autonomous(name = "AutonomousTest", group = "Autonomous")

public class AutonomousTest extends LinearOpMode {

    DcMotor motorFrontRight = null;
    DcMotor motorFrontLeft = null;
    DcMotor motorRearRight = null;
    DcMotor motorRearLeft = null;

    GyroSensor gyro = null;


    @Override
    public void runOpMode() throws InterruptedException {

        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorRearRight = hardwareMap.dcMotor.get("motorRearRight");
        motorRearLeft = hardwareMap.dcMotor.get("motorRearLeft");


        motorFrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRearRight.setDirection(DcMotorSimple.Direction.FORWARD);
        motorRearLeft.setDirection(DcMotorSimple.Direction.REVERSE);



        waitForStart();

        gyro.calibrate();

        moveForwardTime(MOVE_SPEED, 5000);
       // turnLeftTime(MOVE_SPEED, 3000);
        moveForwardTime(MOVE_SPEED, 1000);
        //turnLeftGyro(MOVE_SPEED);
        turnToLeftDegrees(MOVE_SPEED, 270);
        stopMovement();
    }

    public double MOVE_SPEED = 0.8;


    public void moveForward(double power){

        motorFrontRight.setPower(power);
        motorFrontLeft.setPower(power);
        motorRearRight.setPower(power);
        motorRearLeft.setPower(power);
    }


    public void moveForwardTime (double power, long time) throws InterruptedException{
        moveForward(power);
        Thread.sleep(time);
    }


    public void turnLeft(double power){
        motorFrontRight.setPower(power);
        motorFrontLeft.setPower(-power);
        motorRearLeft.setPower(power);
        motorRearRight.setPower(-power);
    }

    /*public void turnLeftTime(double power,long time) throws InterruptedException{
        turnLeft(power);

        Thread.sleep(time);
    }*/

    /*public void turnLeftGyro(double power){
        while(gyro.getHeading() > 270 || gyro.getHeading() < 260) {
            turnLeft(power);
        }
    }*/

    public void turnToLeftDegrees(double power, double degree){
        while(gyro.getHeading() != degree) {
            turnLeft(power);
        }
    }


    public void stopMovement(){
        moveForward(0);
    }

}
