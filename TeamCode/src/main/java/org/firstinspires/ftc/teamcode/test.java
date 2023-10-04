package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class test extends OpMode {
    DcMotor motor;

    @Override
    public void init() {

        telemetry.addData("Intialization: ", "is a success");
        telemetry.update();
    }
    @Override
    public void loop() {

    }

    }