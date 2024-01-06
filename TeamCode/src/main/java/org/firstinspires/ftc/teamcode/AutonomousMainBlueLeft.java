package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "AutonomousMainBlueLeft", preselectTeleOp = "DriveOp Main Duocontrol")
public class AutonomousMainBlueLeft extends LinearOpMode {

    private DcMotor backRight;
    private DcMotor frontRight;
    private DcMotor frontLeft;
    private DcMotor backLeft;
    private BNO055IMU imu;

    private Servo clawLeft;
    private Servo clawRight;

    private DcMotor armMotor;


    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {

        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        clawRight = hardwareMap.get(Servo.class, "clawRight");
        clawLeft = hardwareMap.get(Servo.class, "clawLeft");
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");

        // Wait for start command from Driver Station.
        Init_IMU();
        waitForStart();
        // Get a list of recognitions from TFOD.

        //open claw
        clawLeft.setPosition(0.9);
        clawRight.setPosition(0.9);

        MoveLeft(2500);

        //close claw
        clawLeft.setPosition(0.46);
        clawRight.setPosition(0.460);

        // Put loop blocks here.
        telemetry.update();
        // Put run blocks here.
        // Deactivate TFOD.
        // Put loop blocks here.
    }

    private void Init_IMU() {
        BNO055IMU.Parameters IMUparameters;

        // Create a new IMU parameters
        IMUparameters = new BNO055IMU.Parameters();
        // Use degrees as our angle unit
        // Set the IMU mode to IMU so it automatically calibrates
        IMUparameters.mode = BNO055IMU.SensorMode.ACCONLY;
        IMUparameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        // Use meters per second per second as unit of acceleration
        IMUparameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        // Warn driver this may take several seconds!
        imu.initialize(IMUparameters);
        telemetry.addData("Status", "Init-IMU...Please wait");
        telemetry.update();
        // Initialize IMU using these parameters
        imu.initialize(IMUparameters);
        telemetry.addData("Status", "IMU initialized");
        telemetry.update();
    }

    private void Move_To_Position(int targetPosition) {
        // Reset the encodersasd
        backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        // Put motors in encoder mode
        backLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        // Turn on the motors using a moderate power
        backLeft.setPower(0.9);
        backRight.setPower(0.9);
        frontLeft.setPower(0.9);
        frontRight.setPower(0.9);
        // Loop until the motor reaches its target position
        while (backLeft.getCurrentPosition() < targetPosition && frontLeft.getCurrentPosition() < targetPosition) {
            // Nothing while the robot moves forward
        }
        // Turn the motors off
        backLeft.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        frontRight.setPower(0);
        // Sleep a quarter second to let the robot stop
        sleep(1000);
    }

    private void MoveForward(int distance) {
        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        backLeft.setDirection(DcMotorEx.Direction.REVERSE);
        frontRight.setDirection(DcMotorEx.Direction.FORWARD);
        backRight.setDirection(DcMotorEx.Direction.FORWARD);
        Move_To_Position(distance);
    }

    private void MoveRight(int turnRate) {
        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        backLeft.setDirection(DcMotorEx.Direction.FORWARD);
        frontRight.setDirection(DcMotorEx.Direction.REVERSE);
        backRight.setDirection(DcMotorEx.Direction.FORWARD);
        Move_To_Position(turnRate);
    }

    private void MoveLeft(int turnRate) {
        frontLeft.setDirection(DcMotorEx.Direction.FORWARD);
        backLeft.setDirection(DcMotorEx.Direction.REVERSE);
        frontRight.setDirection(DcMotorEx.Direction.FORWARD);
        backRight.setDirection(DcMotorEx.Direction.REVERSE);
        Move_To_Position(turnRate);
    }
}
    /**
     * Describe this function...
     */