package org.firstinspires.ftc.teamcode.Nero.Mecanisms;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Nero.PID.NeroFlywheelPIDF;

public class Shooter {
    private NeroFlywheelPIDF pid;
    public Servo hood;
    public DcMotorEx leftFlywheel = null;
    public DcMotorEx rightFlywheel = null;
    public double targetRPM = 4800;
    private boolean upPressed = false;
    private boolean downPressed = false;
    private final double rpmStep = 50;
    public boolean lastButtonState = false;
    public boolean motorRunning = false;

    public void init(HardwareMap hwMap) {
        leftFlywheel = hwMap.get(DcMotorEx.class, "1");
        rightFlywheel = hwMap.get(DcMotorEx.class, "2");
        hood = hwMap.get(Servo.class,"hood");
        leftFlywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightFlywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftFlywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        rightFlywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        pid = new NeroFlywheelPIDF(0.000005,0,0,0.0002);
    }

    public double RPM(double distance){
        double speed = 0;
        return speed;
    }
    public double hood(double distance){
        double hood = 0.5;
        return hood;
    }

    public void setRPM(boolean up, boolean down){
        if (up && !upPressed) {
            targetRPM += rpmStep;
            upPressed = true;
        }
        if (!up) {
            upPressed = false;
        }
        if (down && !downPressed) {
            targetRPM -= rpmStep;
            if (targetRPM < 0) targetRPM = 0;
            downPressed = true;
        }
        if (!down) {
            downPressed = false;
        }
    }

    public static double distance2D(double x1, double y1, double x2, double y2) {
        double dx = x2 - x1;
        double dy = y2 - y1;
        return Math.sqrt(dx * dx + dy * dy);
    }

    public void Shooter(boolean input){
        double leftVel = leftFlywheel.getVelocity();
        double rightVel = rightFlywheel.getVelocity();
        double avgRPM = (leftVel + rightVel) / 2 / 28.0 * 60.0;
        double flywheelPower = (pid.calculate(targetRPM, avgRPM));
        hood.setPosition(.9);
        if (input && !lastButtonState) {
            motorRunning = !motorRunning;
        }
        if (!motorRunning){
            pid.reset();
        }
        leftFlywheel.setPower(motorRunning? flywheelPower : 0);
        rightFlywheel.setPower(motorRunning? flywheelPower : 0);
        lastButtonState = input;
    }


}
