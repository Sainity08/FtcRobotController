package org.firstinspires.ftc.teamcode.Nero.Tests;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class NeroIntakeTest extends OpMode {
    public DcMotor leftIntake;
    public DcMotor rightIntake;
    public Servo hardstop;
    public boolean lastButtonState = false;
    public boolean motorRunning = false;

    @Override
    public void init() {
        leftIntake = hardwareMap.get(DcMotor.class, "left");
        rightIntake = hardwareMap.get(DcMotor.class, "right");
        hardstop = hardwareMap.get(Servo.class, "Hardstop");
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
        telemetry.addData("Servo Position", hardstop.getPosition());
    }
}
   