package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.List;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name = "Aligner", preselectTeleOp = "DriveOp Main Duocontrol")
public class Aligner extends LinearOpMode {

    boolean USE_WEBCAM;
    AprilTagProcessor myAprilTagProcessor;
    VisionPortal myVisionPortal;

    private DcMotor backRight;
    private DcMotor frontRight;
    private DcMotor frontLeft;
    private DcMotor backLeft;
    private BNO055IMU imu;

    /**
     * This function is executed when this OpMode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        // This 2023-2024 OpMode illustrates the basics of AprilTag recognition and pose estimation.
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        // Wait for start command from Driver Station.
        Init_IMU();
        waitForStart();
        USE_WEBCAM = true;
        // Initialize AprilTag before waitForStart.
        initAprilTag();
        // Wait for the match to begin.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                // Put loop blocks here.
                telemetryAprilTag();
                List<AprilTagDetection> myAprilTagDetections;
                AprilTagDetection myAprilTagDetection;
                int myAprilTagCode;

                // Get a list of AprilTag detections.
                myAprilTagDetections = myAprilTagProcessor.getDetections();
                // Push telemetry to the Driver Station.
                telemetry.update();
                for (AprilTagDetection myAprilTagDetection_item : myAprilTagDetections) {
                    myAprilTagDetection = myAprilTagDetection_item;
                    myAprilTagCode = myAprilTagDetection.id;
                    if ((myAprilTagDetection.metadata != null) && (myAprilTagCode == 1)){
                            if (myAprilTagDetection.ftcPose.x < -2) {
                                MoveLeft(20);
                            } else if (myAprilTagDetection.ftcPose.x > 2) {
                                MoveRight(20);
                            }
                        }
                    }
                }
                // Share the CPU.
                sleep(20);
            }
        }

    /**
     * Initialize AprilTag Detection.
     */
    private void initAprilTag() {
        // First, create an AprilTagProcessor.
        myAprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        // Next, create a VisionPortal.
        if (USE_WEBCAM) {
            myVisionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), myAprilTagProcessor);
        } else {
            myVisionPortal = VisionPortal.easyCreateWithDefaults(BuiltinCameraDirection.BACK, myAprilTagProcessor);
        }
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
        // Reset the encoders
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
    /**
     * Display info (using telemetry) for a recognized AprilTag.
     */
    private void telemetryAprilTag() {
        List<AprilTagDetection> myAprilTagDetections;
        AprilTagDetection myAprilTagDetection;

        // Get a list of AprilTag detections.
        myAprilTagDetections = myAprilTagProcessor.getDetections();
        telemetry.addData("# AprilTags Detected", JavaUtil.listLength(myAprilTagDetections));
        // Iterate through list and call a function to
        // display info for each recognized AprilTag.
        for (AprilTagDetection myAprilTagDetection_item : myAprilTagDetections) {
            myAprilTagDetection = myAprilTagDetection_item;
            // Display info about the detection.
            telemetry.addLine("");
            if (myAprilTagDetection.metadata != null) {
                telemetry.addLine("==== (ID " + myAprilTagDetection.id + ") " + myAprilTagDetection.metadata.name);
                telemetry.addLine("XYZ " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.x, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.y, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.z, 6, 1) + "  (inch)");
                telemetry.addLine("PRY " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.pitch, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.roll, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.yaw, 6, 1) + "  (deg)");
                telemetry.addLine("RBE " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.range, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.bearing, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.elevation, 6, 1) + "  (inch, deg, deg)");
            } else {
                telemetry.addLine("==== (ID " + myAprilTagDetection.id + ") Unknown");
                telemetry.addLine("Center " + JavaUtil.formatNumber(myAprilTagDetection.center.x, 6, 0) + "" + JavaUtil.formatNumber(myAprilTagDetection.center.y, 6, 0) + " (pixels)");
            }
        }
        telemetry.addLine("");
        telemetry.addLine("key:");
        telemetry.addLine("XYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");
    }
}
