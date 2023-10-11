package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;



@TeleOp(name = "DriveOpMainDuocontrol (Blocks to Java)")
public class old_driveop extends LinearOpMode {

    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor ArmMotor;
    private Servo ClawRight;
    private Servo ClawLeft;

    double rightPower;
    double Power;
    double triggerPower;
    String driveMode;
    double leftPower;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        Gamepad previousGamePad1 = new Gamepad();
        Gamepad currentGamePad1 = new Gamepad();
        Gamepad previousGamePad2 = new Gamepad();
        Gamepad currentGamePad2 = new Gamepad();


        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        ArmMotor = hardwareMap.get(DcMotor.class, "ArmMotor");
        ClawRight = hardwareMap.get(Servo.class, "ClawRight");
        ClawLeft = hardwareMap.get(Servo.class, "ClawLeft");

        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        ClawRight.setDirection(Servo.Direction.REVERSE);
        Power = 0.7;
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                previousGamePad1.copy(currentGamePad1);
                currentGamePad1.copy(gamepad1);
                previousGamePad2.copy(currentGamePad2);
                currentGamePad2.copy(gamepad2);

                if (currentGamePad2.left_trigger != 0) {
                    triggerPower = gamepad2.left_trigger * 0.5;
                }
                if (currentGamePad1.right_trigger != 0) {
                    triggerPower = -(gamepad2.right_trigger * 1);
                }
                if (currentGamePad1.right_stick_x == 0) {
                    driveMode = "straight";
                    leftPower = currentGamePad1.left_stick_y * Power;
                    rightPower = currentGamePad1.left_stick_y * Power;
                } else {
                    driveMode = "turn";
                    leftPower = currentGamePad1.left_stick_y * Power - gamepad1.right_stick_x * Power;
                    rightPower = currentGamePad1.left_stick_y * Power + gamepad1.right_stick_x * Power;
                }
                if (gamepad2.a) {
                    ClawLeft.setPosition(0.9);
                    ClawRight.setPosition(0.9);
                }
                if (gamepad2.b) {
                    ClawLeft.setPosition(0.4);
                    ClawRight.setPosition(0.4);
                }
                ArmMotor.setPower(triggerPower);
                backLeft.setPower(leftPower);
                frontLeft.setPower(leftPower);
                backRight.setPower(rightPower);
                frontRight.setPower(rightPower);
                telemetry.addData("Drive Mode: ", driveMode);
                telemetry.addData("Left Power:", leftPower);
                telemetry.addData("Right Power:", rightPower);
                telemetry.addData("Trigger Power:", triggerPower);
                telemetry.update();
            }
        }
    }
}