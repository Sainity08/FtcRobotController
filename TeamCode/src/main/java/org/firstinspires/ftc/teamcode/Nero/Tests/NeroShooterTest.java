package org.firstinspires.ftc.teamcode.Nero.Tests;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.PID.FlywheelPID;

@TeleOp
public class NeroShooterTest extends OpMode {
    public DcMotorEx leftFlywheel = null;
    public DcMotorEx rightFlywheel = null;
    public Servo hood;
    public boolean lastButtonState = false;
    public boolean motorRunning = false;
    private FlywheelPID pid;
    private double targetRPM = 5000;
    private double targetHood = 0.5;
    private boolean upPressed = false;
    private boolean downPressed = false;
    private boolean leftPressed = false;
    private boolean rightPressed = false;
    private final double hoodStep = .05;
    private final double rpmStep = 50;

    @Override
    public void init(){
        leftFlywheel = hardwareMap.get(DcMotorEx.class, "1");
        rightFlywheel = hardwareMap.get(DcMotorEx.class, "2");
        leftFlywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightFlywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        hood = hardwareMap.get(Servo.class,"hood");
        leftFlywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        rightFlywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        pid = new FlywheelPID(0, 0, 0.001,0);
    }

    @Override
    public void loop(){
        boolean input1 = gamepad1.x;

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


        double leftVel = leftFlywheel.getVelocity();
        double rightVel = rightFlywheel.getVelocity();
        double avgRPM = (leftVel + rightVel) / 2 / 28.0 * 60.0;

        if (input1 && !lastButtonState) {
            motorRunning = !motorRunning;
        }
        leftFlywheel.setPower(pid.calculate(targetRPM, avgRPM, 0.02));
        rightFlywheel.setPower(pid.calculate(targetRPM, avgRPM, 0.02));
        lastButtonState = input1;

        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Current RPM", avgRPM);
        telemetry.update();
    }
}
