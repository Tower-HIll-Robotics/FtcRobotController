package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.Tfod;

import java.util.List;

@Autonomous(name = "AutonomousMainLeft (Blocks to Java)", preselectTeleOp = "DriveOp Main Duocontrol")
public class AutonomousMainLeft extends LinearOpMode {

    private VuforiaCurrentGame vuforiaPOWERPLAY;
    private Tfod tfod;
    private Servo ClawLeftAsServo;
    private Servo ClawRightAsServo;
    private DcMotor ArmMotorAsDcMotor;
    private DcMotor backRight;
    private DcMotor frontRight;
    private DcMotor frontLeft;
    private DcMotor backLeft;
    private BNO055IMU imu;

    Recognition recognition;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        List<Recognition> recognitions;
        int index;

        vuforiaPOWERPLAY = new VuforiaCurrentGame();
        tfod = new Tfod();
        ClawLeftAsServo = hardwareMap.get(Servo.class, "ClawLeftAsServo");
        ClawRightAsServo = hardwareMap.get(Servo.class, "ClawRightAsServo");
        ArmMotorAsDcMotor = hardwareMap.get(DcMotor.class, "ArmMotorAsDcMotor");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        // Sample TFOD Op Mode
        // Initialize Vuforia.
        vuforiaPOWERPLAY.initialize(
                "", // vuforiaLicenseKey
                hardwareMap.get(WebcamName.class, "Webcam 1"), // cameraName
                "", // webcamCalibrationFilename
                false, // useExtendedTracking
                false, // enableCameraMonitoring
                VuforiaLocalizer.Parameters.CameraMonitorFeedback.NONE, // cameraMonitorFeedback
                0, // dx
                0, // dy
                0, // dz
                AxesOrder.XZY, // axesOrder
                90, // firstAngle
                90, // secondAngle
                0, // thirdAngle
                true); // useCompetitionFieldTargetLocations
        tfod.useDefaultModel();
        // Set min confidence threshold to 0.7
        tfod.initialize(vuforiaPOWERPLAY, (float) 0.7, true, true);
        // Initialize TFOD before waitForStart.
        // Activate TFOD here so the object detection labels are visible
        // in the Camera Stream preview window on the Driver Station.
        tfod.activate();
        // Enable following block to zoom in on target.
        tfod.setZoom(1, 16 / 9);
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        // Wait for start command from Driver Station.
        Init_IMU();
        waitForStart();
        // Get a list of recognitions from TFOD.
        recognitions = tfod.getRecognitions();
        // If list is empty, inform the user. Otherwise, go
        // through list and display info for each recognition.
        if (JavaUtil.listLength(recognitions) == 0) {
            ClawLeftAsServo.setDirection(Servo.Direction.FORWARD);
            ClawRightAsServo.setDirection(Servo.Direction.REVERSE);
            ClawRightAsServo.setPosition(0.9);
            ClawLeftAsServo.setPosition(0.9);
            parking_right();
        } else {
            index = 0;
            // Iterate through list and call a function to
            // display info for each recognized object.
            for (Recognition recognition_item : recognitions) {
                recognition = recognition_item;
                // Display info.
                displayInfo(index);
                // Increment index.
                index = index + 1;
            }
            ClawLeftAsServo.setDirection(Servo.Direction.FORWARD);
            ClawRightAsServo.setDirection(Servo.Direction.REVERSE);
            ClawRightAsServo.setPosition(0.9);
            ClawLeftAsServo.setPosition(0.9);
            parking_forward();
            MoveBack2();
            turn45CCW2();
            ArmMotorAsDcMotor.setPower(0.6);
            sleep(3000);
            MoveSmall2();
            ArmMotorAsDcMotor.setPower(0);
            sleep(1000);
            ClawRightAsServo.setPosition(0.3);
            ClawLeftAsServo.setPosition(0.3);
            sleep(1000);
            MoveBack3();
            turn45CC2();
            sleep(1000);
            ArmMotorAsDcMotor.setPower(-0.3);
            sleep(300);
            // Put loop blocks here.
            telemetry.update();
            // Put run blocks here.
            // Deactivate TFOD.
            tfod.deactivate();
            // Put loop blocks here.
            if (recognition.getLabel().equals("1 Bolt")) {
                parking_left2();
            }
            if (recognition.getLabel().equals("2 Bulb")) {
                sleep(1000);
            }
            if (recognition.getLabel().equals("3 Panel")) {
                parking_right2();
            }
            telemetry.update();
        }

        vuforiaPOWERPLAY.close();
        tfod.close();
    }

    /**
     * Describe this function...
     */
    private void turn45CCW2() {
        turnRight(540);
    }

    /**
     * Describe this function...
     */
    private void turnLeft(int turnRate) {
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        Move_To_Position(turnRate);
    }

    /**
     * Describe this function...
     */
    private void turn45CC2() {
        turnLeft(490);
    }

    /**
     * Describe this function...
     */
    private void turn45CCW() {
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        Move_To_Position(570);
        backLeft.setPower(0.6);
        frontLeft.setPower(0.6);
        Move_To_Position(-570);
        backRight.setPower(-0.6);
        frontRight.setPower(-0.6);
        while (backLeft.isBusy() && backRight.isBusy() && frontLeft.isBusy() && frontRight.isBusy()) {
        }
        backLeft.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        frontRight.setPower(0);
    }

    /**
     * Describe this function...
     */
    private void MoveSmall2() {
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        Move_To_Position(430);
    }

    /**
     * Describe this function...
     */
    private void turnRight(int turnRate) {
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        Move_To_Position(turnRate);
    }

    /**
     * Describe this function...
     */
    private void MoveBack2() {
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        Move_To_Position(500);
    }

    /**
     * Describe this function...
     */
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

    /**
     * Describe this function...
     */
    private void MoveBack3() {
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        Move_To_Position(300);
    }

    /**
     * Describe this function...
     */
    private void turn45CC() {
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        Move_To_Position(570);
        backLeft.setPower(0.6);
        frontLeft.setPower(0.6);
        Move_To_Position(-570);
        backRight.setPower(-0.6);
        frontRight.setPower(-0.6);
        while (backLeft.isBusy() && backRight.isBusy() && frontLeft.isBusy() && frontRight.isBusy()) {
        }
        backLeft.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        frontRight.setPower(0);
    }

    /**
     * Describe this function...
     */
    private float getZaxisOrientation() {
        Orientation Angles;

        Angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return Angles.firstAngle;
    }

    /**
     * Describe this function...
     */
    private void parking_forward() {
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        Move_To_Position(3000);
    }

    /**
     * Describe this function...
     */
    private void parking_left2() {
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        turnLeft(1232);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        Move_To_Position(1650);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    /**
     * Describe this function...
     */
    private void parking_forward2() {
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        Move_To_Position(1);
    }

    /**
     * Describe this function...
     */
    private void MoveBack() {
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        Move_To_Position(-625);
        backLeft.setPower(-0.6);
        backRight.setPower(-0.6);
        frontLeft.setPower(-0.6);
        frontRight.setPower(-0.6);
        while (backLeft.isBusy() && backRight.isBusy() && frontLeft.isBusy() && frontRight.isBusy()) {
        }
        backLeft.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        frontRight.setPower(0);
    }

    /**
     * Describe this function...
     */
    private void parking_left() {
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        Move_To_Position(2500);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        turnLeft(1232);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        Move_To_Position(1800);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    /**
     * Describe this function...
     */
    private void Move_To_Position(int targetPosition) {
        // Reset the encoders
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Put motors in encoder mode
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

    /**
     * Describe this function...
     */
    private void MoveSmall() {
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        Move_To_Position(625);
        backLeft.setPower(0.6);
        backRight.setPower(0.6);
        frontLeft.setPower(0.6);
        frontRight.setPower(0.6);
        while (backLeft.isBusy() && backRight.isBusy() && frontLeft.isBusy() && frontRight.isBusy()) {
        }
        backLeft.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        frontRight.setPower(0);
    }

    /**
     * Describe this function...
     */
    private void RotateCCW(
            // TODO: Enter the type for argument named targetOrientationAngle
            UNKNOWN_TYPE targetOrientationAngle) {
        float ZOrientation;

        ZOrientation = getZaxisOrientation();
        backLeft.setPower(-0.2);
        frontLeft.setPower(-0.2);
        backRight.setPower(0.2);
        frontRight.setPower(0.2);
        while (ZOrientation < targetOrientationAngle) {
            ZOrientation = getZaxisOrientation();
        }
        backLeft.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        frontRight.setPower(0);
        sleep(250);
    }

    /**
     * Describe this function...
     */
    private void parking_right() {
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        Move_To_Position(2500);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        turnRight(1200);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        Move_To_Position(1750);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    /**
     * Describe this function...
     */
    private void parking_right2() {
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        turnRight(1000);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        Move_To_Position(1900);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    /**
     * Display info (using telemetry) for a recognized object.
     */
    private void displayInfo(int i) {
        // Display label info.
        // Display the label and index number for the recognition.
        telemetry.addData("label " + i, recognition.getLabel());
        // Display upper corner info.
        // Display the location of the top left corner
        // of the detection boundary for the recognition
        telemetry.addData("Left, Top " + i, Double.parseDouble(JavaUtil.formatNumber(recognition.getLeft(), 0)) + ", " + Double.parseDouble(JavaUtil.formatNumber(recognition.getTop(), 0)));
        // Display lower corner info.
        // Display the location of the bottom right corner
        // of the detection boundary for the recognition
        telemetry.addData("Right, Bottom " + i, Double.parseDouble(JavaUtil.formatNumber(recognition.getRight(), 0)) + ", " + Double.parseDouble(JavaUtil.formatNumber(recognition.getBottom(), 0)));
    }
}