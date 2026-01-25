package org.firstinspires.ftc.teamcode.Nero.Tests;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.PID.FlywheelPID;

@TeleOp
public class NeroIntakeToShooter extends OpMode {
    public DcMotor leftIntake;
    public DcMotor rightIntake;
    public Servo hardstop;
    public boolean lastButtonState = false;
    public boolean motorRunning = false;
    public DcMotorEx leftFlywheel = null;
    public DcMotorEx rightFlywheel = null;
    public boolean lastButtonState1 = false;
    public boolean motorRunning1 = false;
    private FlywheelPID pid;
    private double targetRPM = 5000;
    private boolean upPressed = false;
    private boolean downPressed = false;
    private final double rpmStep = 50;

    @Override
    public void init() {
        leftIntake = hardwareMap.get(DcMotor.class, "left");
        rightIntake = hardwareMap.get(DcMotor.class, "right");
        hardstop = hardwareMap.get(Servo.class, "Hardstop");
        leftIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        rightIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFlywheel = hardwareMap.get(DcMotorEx.class, "1");
        rightFlywheel = hardwareMap.get(DcMotorEx.class, "2");
        leftFlywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightFlywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftFlywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        rightFlywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        pid = new FlywheelPID(0.015, 0.0005, 0.005,0.7);
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

        boolean input2 = gamepad1.x;

        if (gamepad1.dpad_up && !upPressed) {
            targetRPM += rpmStep;
            upPressed = true;
        }
        if (!gamepad1.dpad_up) {
            upPressed = false;
        }
        if (gamepad1.dpad_down && !downPressed) {
            targetRPM -= rpmStep;
            if (targetRPM < 0) targetRPM = 0;
            downPressed = true;
        }
        if (!gamepad1.dpad_down) {
            downPressed = false;
        }


        double leftVel = leftFlywheel.getVelocity();
        double rightVel = rightFlywheel.getVelocity();
        double avgRPM = (leftVel + rightVel) / 2 / 28.0 * 60.0;

        if (input2 && !lastButtonState1) {
            motorRunning1 = !motorRunning1;
        }
        leftFlywheel.setPower(pid.getPower(targetRPM, avgRPM, motorRunning1));
        rightFlywheel.setPower(pid.getPower(targetRPM, avgRPM, motorRunning1));
        lastButtonState1 = input2;

        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Current RPM", avgRPM);
        telemetry.update();
    }
}
