package org.firstinspires.ftc.teamcode;

import android.renderscript.ScriptIntrinsicResize;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


//--------------------------------------------------------------
//**************************************************************
//--------------------------------------------------------------
//TASK CA SA VAD CA FACETI SI VOI CEVA:
//
//functia getColor() returneaza valoarea de lumina rosie. Folositi-o ca si model. NU O FOLOSITI IN COD.
//folosind functiile de aflare a intensitatii culorilor separat (color_sensor.blue()/.red()/.green())
//sa se creeze un algoritm de a misca motorul barei de metal din fata spre culoara albastra, initializand motorul
//si hardware.map-ul lui. Nu trebuie sa il numiti special in harware map.
//Asta e tot.
//--------------------------------------------------------------
//**************************************************************
//--------------------------------------------------------------





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
    public double TURN_SPEED = 0.2;
    public double ODS_SCALE_MULTIPLIER = 46;

    ElapsedTime timer = new ElapsedTime();


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


        timer.reset();

        gyro.calibrate();

        while (gyro.isCalibrating())  {

            telemetry.addData("calibrating", Math.round(timer.seconds()));

            telemetry.update();

            sleep(50);

        }

        //**************
        //main program
        //**************
        waitForStart();


        moveForwardTime(MOVE_SPEED, 5000);
        stopMovement();
        calibrateGyro();
        turnRightToDegrees(TURN_SPEED, 270);
        sleep(2000);
        getColor();

        stopMovement();
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

    public void moveForwardODS (double power, double distance){
        double reflectedDistance;
        //moveForward(power);

        do{
            reflectedDistance = distanceSensor.getLightDetected();

            telemetry.addData("Distance", reflectedDistance);
            telemetry.update();

        }while(reflectedDistance < distance);

        stopMovement();

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
        turn(power, "left");


        int heading = gyro.getHeading();

        while(!(heading > degree-10 && heading < degree +10) ) {

            heading = gyro.getHeading();
            telemetry.addData("Degrees",heading);
            telemetry.update();
        }
        stopMovement();
    }

    public void turnRightToDegrees(double power, double degree){
        turn(power, "right");

        
        int heading = gyro.getHeading();

        while(!(heading > degree-10 && heading < degree +10) ) {

            heading = gyro.getHeading();
            telemetry.addData("Degrees",heading);
            telemetry.update();
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

        return Math.sqrt(1/lightIntensity)*ODS_SCALE_MULTIPLIER;
    }

    public void calibrateGyro(){
        gyro.calibrate();

        while (gyro.isCalibrating())  {

            telemetry.addData("calibrating", Math.round(timer.seconds()));

            telemetry.update();

            sleep(50);

        }
    }

    public void getColor(){

        color_sensor.enableLed(false);

        while(!isStopRequested()) {

            double color = color_sensor.red();

            telemetry.addData("Color number", color*10);
            telemetry.update();
        }
    }

}
