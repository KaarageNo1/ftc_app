package org.firstinspires.ftc.teamcode;

import android.renderscript.ScriptIntrinsicResize;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

/**
 * Created by Dani on 9/27/2017.
 */
@Autonomous(name = "AutonomousTest", group = "Autonomous")

public class AutonomousTest extends LinearOpMode {


    //*************************
    //components initialization
    //*************************
    DcMotor motorFrontRight = null;
    DcMotor motorFrontLeft = null;
    DcMotor motorRearRight = null;
    DcMotor motorRearLeft = null;

    GyroSensor gyro = null;

    OpticalDistanceSensor distanceSensor = null;

    //******************
    //static variables
    //******************
    public double MOVE_SPEED = 0.5;
    public double ODS_SCALE_MULTIPLIER = 46;



    @Override
    public void runOpMode() throws InterruptedException {

        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorRearRight = hardwareMap.dcMotor.get("motorRearRight");
        motorRearLeft = hardwareMap.dcMotor.get("motorRearLeft");
        gyro = hardwareMap.gyroSensor.get("gyro");
        distanceSensor = hardwareMap.opticalDistanceSensor.get("distanceSensor");


        motorFrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRearRight.setDirection(DcMotorSimple.Direction.FORWARD);
        motorRearLeft.setDirection(DcMotorSimple.Direction.REVERSE);



        waitForStart();

        gyro.calibrate();

        moveForwardTime(MOVE_SPEED, 5000);
        moveForwardTime(MOVE_SPEED, 1000);
        turnLeftToDegrees(MOVE_SPEED, 270);
        turnRightToDegrees(MOVE_SPEED, 0);
        moveForward(MOVE_SPEED);

        double reflectedDistance;

        do {
            reflectedDistance = linearAlgorithmODS(distanceSensor.getLightDetected());
        }while(reflectedDistance > 2);

        stopMovement();
    }



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


    public void turn(double power, String direction){

        if(direction.equals("right")){
            power = -power;
        }

        motorFrontRight.setPower(power);
        motorFrontLeft.setPower(-power);
        motorRearLeft.setPower(-power);
        motorRearRight.setPower(power);
    }

    public void turnLeftToDegrees(double power, double degree){
        while(gyro.getHeading() != degree) {
            turn(power,"left");
        }
    }

    public void turnRightToDegrees(double power, double degree){
        while(gyro.getHeading() != degree) {
            turn(power,"right");
        }
    }


    public void stopMovement(){
        moveForward(0);
    }


    public double linearAlgorithmODS(double lightIntensity){
        //*********************************************************************************
        //makes the value returned by the sensor proportional with the distance in centimeters
        //inverse square root (intensity at the power 0.5)
        //*********************************************************************************

        return Math.pow(lightIntensity,-0.5)*ODS_SCALE_MULTIPLIER;
    }

}
