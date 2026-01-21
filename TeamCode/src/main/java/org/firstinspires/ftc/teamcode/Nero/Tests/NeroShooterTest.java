package org.firstinspires.ftc.teamcode.Nero.Tests;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.PID.FlywheelPID;

@TeleOp
public class NeroShooterTest extends OpMode {
    public DcMotorEx leftFlywheel = null;
    public DcMotorEx rightFlywheel = null;
    public boolean lastButtonState = false;
    public boolean motorRunning = false;
    private FlywheelPID pid;
    private double targetRPM = 5000;
    private boolean upPressed = false;
    private boolean downPressed = false;
    private final double rpmStep = 50;

    @Override
    public void init(){
        leftFlywheel = hardwareMap.get(DcMotorEx.class, "flywheelL");
        rightFlywheel = hardwareMap.get(DcMotorEx.class, "flywheelR");
        leftFlywheel.setDirection(DcMotorEx.Direction.REVERSE);
        rightFlywheel.setDirection(DcMotorEx.Direction.FORWARD);
        leftFlywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightFlywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftFlywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        rightFlywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        pid = new FlywheelPID(0.015, 0.0005, 0.005,0.7);
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


        double leftVel = leftFlywheel.getVelocity();
        double rightVel = rightFlywheel.getVelocity();
        double avgRPM = (leftVel + rightVel) / 2 / 28.0 * 60.0;

        if (input1 && !lastButtonState) {
            motorRunning = !motorRunning;
        }
        leftFlywheel.setPower(pid.getPower(targetRPM, avgRPM, motorRunning));
        rightFlywheel.setPower(pid.getPower(targetRPM, avgRPM, motorRunning));
        lastButtonState = input1;

        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Current RPM", avgRPM);
        telemetry.update();
    }
}
