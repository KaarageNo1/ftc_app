package org.firstinspires.ftc.teamcode;

import android.renderscript.ScriptIntrinsicResize;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
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
    //components declaration
    //*************************
    DcMotor motorFrontRight = null;
    DcMotor motorFrontLeft = null;
    DcMotor motorRearRight = null;
    DcMotor motorRearLeft = null;

    GyroSensor gyro = null;

    OpticalDistanceSensor distanceSensor = null;

    ColorSensor color_sensor = null;

    //******************
    //static variables
    //******************
    public double MOVE_SPEED = 0.5;
    public double TURN_SPEED = 0.1;
    public double ODS_SCALE_MULTIPLIER = 46;



    @Override
    public void runOpMode() throws InterruptedException {


        //*************************
        //components initialization
        //*************************
        motorFrontRight = hardwareMap.dcMotor.get("right_drive_front");
        motorFrontLeft = hardwareMap.dcMotor.get("left_drive_front");
        motorRearRight = hardwareMap.dcMotor.get("right_drive_back");
        motorRearLeft = hardwareMap.dcMotor.get("left_drive_back");
        gyro = hardwareMap.gyroSensor.get("gyro");
        distanceSensor = hardwareMap.opticalDistanceSensor.get("ods");
        color_sensor = hardwareMap.colorSensor.get("color");


        motorFrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRearRight.setDirection(DcMotorSimple.Direction.FORWARD);
        motorRearLeft.setDirection(DcMotorSimple.Direction.REVERSE);



        //**************
        //main program
        //**************
        waitForStart();

        double reflectedDistance;

        gyro.calibrate();                                                                           //calibrates the gyroscope to position 0

        moveForwardTime(MOVE_SPEED, 5000);
        stopMovement();
        //moveForwardTime(MOVE_SPEED, 1000);
        gyro.calibrate();
        turnRightToDegrees(TURN_SPEED, 90);
        stopMovement();
        //turnRightToDegrees(TURN_SPEED, 180);
        moveForward(MOVE_SPEED);



        /*do {
            //reflectedDistance = linearAlgorithmODS(distanceSensor.getLightDetected());              //translates the output data from the ODS to an approximation of the distance in cm
            reflectedDistance = distanceSensor.getLightDetected();

        }while(reflectedDistance < 500);     */                                                         //if the sensor reads less than 2 cm the motors stop

        stopMovement();                                                                             //stops the movement


        //readColor();                                                                              //prototype of reading the color using the hue scale
        //telemetry.addData("");

    }


    //*********
    //methods
    //*********


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
        turn(power, "right");
        gyro.calibrate();
        
        int heading = gyro.getHeading();

        while(!(heading > degree-10 && heading < degree +10) ) {

            heading = gyro.getHeading();
            //turn(power,"right");
            //telemetry.addData("Degrees",gyro.getHeading());
        }
        stopMovement();
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

    public char readColor(){

        int hue = color_sensor.argb();

        if(hue < 60 || hue > 300){

            return 'r';
        }
        if(hue > 60 && hue < 180){
            return 'g';
        }
        if(hue > 180 && hue < 300){
            return 'b';
        }


        return 'x';
    }

}
