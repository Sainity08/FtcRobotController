package org.firstinspires.ftc.teamcode.Nero.Tests;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class NeroIntakeTest extends OpMode {
    public DcMotor leftIntake;
    public DcMotor rightIntake;
    public boolean lastButtonState = false;
    public boolean motorRunning = false;

    @Override
    public void init() {
        leftIntake = hardwareMap.get(DcMotor.class, "left");
        rightIntake = hardwareMap.get(DcMotor.class, "right");
        leftIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        rightIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    @Override
    public void loop() {
        boolean input1 = gamepad1.left_bumper;

        if (input1 && !lastButtonState) {
            motorRunning = !motorRunning;
        }
        leftIntake.setPower(motorRunning ? 1 : 0.0);
        rightIntake.setPower(motorRunning ? 1 : 0.0);
        lastButtonState = input1;
    }
}
   