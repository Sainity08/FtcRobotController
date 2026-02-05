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
    public boolean lastButtonState1 = false;
    public boolean motorRunning1 = false;

    @Override
    public void init() {
        leftIntake = hardwareMap.get(DcMotor.class, "leftI");
        rightIntake = hardwareMap.get(DcMotor.class, "rightI");
        hardstop = hardwareMap.get(Servo.class, "hardstop");
        leftIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        rightIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    @Override
    public void loop() {
        boolean input1 = gamepad1.left_bumper;
        boolean input2 = gamepad1.a;

        if (input1 && !lastButtonState) {
            motorRunning = !motorRunning;
        }
        leftIntake.setPower(motorRunning ? 1 : 0.0);
        rightIntake.setPower(motorRunning ? 1 : 0.0);
        lastButtonState = input1;

        if (input2 && !lastButtonState1) {
            motorRunning1 = !motorRunning1;
        }
        hardstop.setPosition(motorRunning1? 0:1);
        lastButtonState1 = input2;
    }

    public double exponential_decay(double startingPower, double finalPower, double timeInterval, double decaySpeed, double elapsedTime) {
        double normalizedTime = elapsedTime / timeInterval;
        return finalPower - (startingPower - finalPower) *
                (Math.exp(Math.pow(normalizedTime, elapsedTime)) - Math.exp(-1)) /
                (1 - Math.exp(-1));
    }
}
   