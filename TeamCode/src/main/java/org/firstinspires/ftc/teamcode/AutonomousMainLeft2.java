package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

@Autonomous(name = "AutonomousMainLeft (Blocks to Java)", preselectTeleOp = "DriveOp Main Duocontrol")
public class AutonomousMainLeft extends LinearOpMode {

    private DcMotor backRight;
    private DcMotor frontRight;
    private DcMotor frontLeft;
    private DcMotor backLeft;
    private BNO055IMU imu;


    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode()
    {

        frontLeft  = hardwareMap.get(DcMotorEx.class, "frontLeft");
        backLeft  = hardwareMap.get(DcMotorEx.class, "backLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        // Wait for start command from Driver Station.
        Init_IMU();
        waitForStart();
        // Get a list of recognitions from TFOD.

        MoveForward(500);
            // Put loop blocks here.
        telemetry.update();
            // Put run blocks here.
            // Deactivate TFOD.
            // Put loop blocks here.
    }

    private void MoveForward(int distance) {
        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        backLeft.setDirection(DcMotorEx.Direction.REVERSE);
        frontRight.setDirection(DcMotorEx.Direction.FORWARD);
        backRight.setDirection(DcMotorEx.Direction.FORWARD);
        Move_To_Position(distance);
    }
    /**
     * Describe this function...
     */