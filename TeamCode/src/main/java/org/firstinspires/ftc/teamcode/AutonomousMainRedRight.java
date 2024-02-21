package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;


import java.util.List;


import java.util.List;

@Autonomous(name = "AutonomousMainRedRight", preselectTeleOp = "DriveOp Main Duocontrol")
public class AutonomousMainRedRight extends LinearOpMode {

    private DcMotor backRight;
    private DcMotor frontRight;
    private DcMotor frontLeft;
    private DcMotor backLeft;
    private BNO055IMU imu;

    private Servo clawLeft;
    private Servo clawRight;

    private DcMotor armMotor;

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera


    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;
    private int markerPosition;


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
        clawRight.setDirection(Servo.Direction.REVERSE);



        // Wait for start command from Driver Station.
        Init_IMU();
        initTfod();
        waitForStart();
        // Get a list of recognitions from TFOD.
        sleep(3000);


        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());

            // if x < (some range for left side) and x > (some range for left side)
            // 2 means center
            if ((x > 355) && (x < 450)) {
                markerPosition = 2;
                telemetry.addData("Position: ","Center");
            }
            // 1 means left most
            else if ((x > 0) && (x < 150)) {
                markerPosition = 1;
                telemetry.addData("Position: ","Left");
            }
            //3 means right most
            else {
                markerPosition = 3;
                telemetry.addData("Position: ","Right");
            }

            telemetry.update();

        }   // end for() loop


        clawLeft.setPosition(0.05);
        clawRight.setPosition(0.2);

        MoveForward(850);

        if (markerPosition == 1) {
            TurnLeft(750);

            sleep(1000);

            clawLeft.setPosition(0.36);
            clawRight.setPosition(0.5);
        }

        else if (markerPosition == 2) {
            clawLeft.setPosition(0.36);
            clawRight.setPosition(0.5);

            sleep(1000);

            TurnLeft(750);
        }
        else if (markerPosition == 3) {
            TurnRight(750);

            sleep(1000);

            clawLeft.setPosition(0.36);
            clawRight.setPosition(0.5);

            sleep(500);
            TurnLeft(750);
            sleep(500);
            TurnLeft(750);
        }
        MoveBackward(850);

        sleep(1000);

        Arm_To_Position(10);
        outWrist.setPosition(0.5);


    }   // end method telemetryTfod()



        // Put loop blocks here.
        // Put run blocks here.
        // Deactivate TFOD.
        // Put loop blocks here.


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

    private void Arm_To_Position(int targetPosition) {
        // Reset the encoders
        armMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        // Put motors in encoder mode
        armMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);


        // Turn on the motors using a moderate power
        armMotor.setPower(0.5);

        // Loop until the motor reaches its target position
        while (armMotor.getCurrentPosition() < targetPosition) {
            // Nothing while the robot moves forward
        }
        // Turn the motors off
        armMotor.setPower(0);

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
    private void MoveBackward(int distance) {
        frontLeft.setDirection(DcMotorEx.Direction.FORWARD);
        backLeft.setDirection(DcMotorEx.Direction.FORWARD);
        frontRight.setDirection(DcMotorEx.Direction.REVERSE);
        backRight.setDirection(DcMotorEx.Direction.REVERSE);
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

    private void TurnLeft(int turnRate) {
        frontLeft.setDirection(DcMotorEx.Direction.FORWARD);
        backLeft.setDirection(DcMotorEx.Direction.FORWARD);
        frontRight.setDirection(DcMotorEx.Direction.FORWARD);
        backRight.setDirection(DcMotorEx.Direction.FORWARD);
        Move_To_Position(turnRate);
    }
    private void TurnRight(int turnRate) {
        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        backLeft.setDirection(DcMotorEx.Direction.REVERSE);
        frontRight.setDirection(DcMotorEx.Direction.REVERSE);
        backRight.setDirection(DcMotorEx.Direction.REVERSE);
        Move_To_Position(turnRate);
    }

    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()
        .setModelFileName("Marker")

        // Use setModelAssetName() if the TF Model is built in as an asset.
        // Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
        //.setModelAssetName(TFOD_MODEL_ASSET)
        //.setModelFileName(TFOD_MODEL_FILE)

        //.setModelLabels(LABELS)
        //.setIsModelTensorFlow2(true)
        //.setIsModelQuantized(true)
        //.setModelInputSize(300)
        //.setModelAspectRatio(16.0 / 9.0)

        .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
        builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableCameraMonitoring(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no pro
        // cessors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        //tfod.setMinResultConfidence(0.75f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

        }   // end method initTfod()
}
    /**
     * Describe this function...
     */