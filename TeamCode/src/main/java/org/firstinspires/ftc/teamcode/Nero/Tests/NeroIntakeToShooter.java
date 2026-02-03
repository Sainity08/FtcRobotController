package org.firstinspires.ftc.teamcode.Nero.Tests;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Nero.Mecanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.Nero.PID.NeroFlywheelPIDF;
import org.firstinspires.ftc.teamcode.PID.FlywheelPID;

@TeleOp
public class NeroIntakeToShooter extends OpMode {
    public DcMotor leftIntake;
    public DcMotor rightIntake;
    public Servo hardstop, hood;
    public boolean lastButtonState = false;
    public boolean motorRunning = false;
    public DcMotorEx leftFlywheel = null;
    public DcMotorEx rightFlywheel = null;
    private boolean leftPressed = false;
    private boolean rightPressed = false;
    private final double hoodStep = .05;
    private double targetHood = 0.5;
    public boolean lastButtonState1 = false;
    public boolean motorRunning1 = false;
    Drivetrain drive = new Drivetrain();

    public boolean lastButtonState2 = false;
    public boolean motorRunning2 = false;
    private NeroFlywheelPIDF pid;
    private double targetRPM = 4800;
    private boolean upPressed = false;
    private boolean downPressed = false;
    private final double rpmStep = 50;

    @Override
    public void init() {
        leftIntake = hardwareMap.get(DcMotor.class, "leftI");
        rightIntake = hardwareMap.get(DcMotor.class, "rightI");
        hardstop = hardwareMap.get(Servo.class, "hardstop");
        drive.init(hardwareMap);
        leftIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        rightIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFlywheel = hardwareMap.get(DcMotorEx.class, "1");
        rightFlywheel = hardwareMap.get(DcMotorEx.class, "2");
        hood = hardwareMap.get(Servo.class,"hood");
        leftFlywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightFlywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftFlywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        rightFlywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        pid = new NeroFlywheelPIDF(0.000005,0,0,0.0002);
    }
    @Override
    public void loop() {
        boolean input1 = gamepad1.left_bumper;
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;
        drive.driveRobotRelative(y, x, turn);

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

        if (gamepad1.dpad_left && !leftPressed) {
            targetHood += hoodStep;
            leftPressed = true;
        }
        if (!gamepad1.dpad_left) {
            leftPressed = false;
        }
        if (gamepad1.dpad_right && !rightPressed) {
            targetHood -= hoodStep;
            if (targetHood < 0) targetHood = 0;
            rightPressed = true;
        }
        if (!gamepad1.dpad_right) {
            rightPressed = false;
        }
        hood.setPosition(targetHood);

        boolean input3 = gamepad1.a;
        double leftVel = leftFlywheel.getVelocity();
        double rightVel = rightFlywheel.getVelocity();
        double avgRPM = (leftVel + rightVel) / 2 / 28.0 * 60.0;


        if (input2 && !lastButtonState1) {
            motorRunning1 = !motorRunning1;
        }
        double flywheelPower = (pid.calculate(targetRPM, avgRPM));

        if (!motorRunning1){
            pid.reset();
        }
        leftFlywheel.setPower(motorRunning1? flywheelPower : 0);
        rightFlywheel.setPower(motorRunning1? flywheelPower : 0);
        lastButtonState1 = input2;

        if (input3 && !lastButtonState2) {
            motorRunning2 = !motorRunning2;
        }
        hardstop.setPosition(motorRunning2? 0:1);
        lastButtonState2 = input3;

        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Current RPM", avgRPM);
        telemetry.addData("Power", flywheelPower);
        telemetry.update();
    }
}
