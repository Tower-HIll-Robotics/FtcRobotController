package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class old_driveop {


@TeleOp(name = "DriveOpMainDuocontrol (Blocks to Java)")
public class DriveOpMainDuocontrol extends LinearOpMode {

    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor ArmMotor;
    private Servo ClawRight;
    private Servo ClawLeft;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        double rightPower;
        double Power;
        double triggerPower;
        String driveMode;
        double leftPower;

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
                if (gamepad2.left_trigger != 0) {
                    triggerPower = gamepad2.left_trigger * 0.5;
                }
                if (gamepad2.right_trigger != 0) {
                    triggerPower = -(gamepad2.right_trigger * 1);
                }
                if (gamepad1.right_stick_x == 0) {
                    driveMode = "straight";
                    leftPower = gamepad1.left_stick_y * Power;
                    rightPower = gamepad1.left_stick_y * Power;
                } else {
                    driveMode = "turn";
                    leftPower = gamepad1.left_stick_y * Power - gamepad1.right_stick_x * Power;
                    rightPower = gamepad1.left_stick_y * Power + gamepad1.right_stick_x * Power;
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