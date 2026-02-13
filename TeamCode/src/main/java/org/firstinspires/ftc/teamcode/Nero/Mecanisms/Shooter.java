package org.firstinspires.ftc.teamcode.Nero.Mecanisms;

import android.annotation.SuppressLint;

import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Nero.PID.NeroFlywheelPIDF;

@SuppressLint("NotConstructor")
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
    public double speed;
    public double hoodPos;
    public double flywheelPower;
    double turn;
    private boolean leftPressed = false;
    private boolean rightPressed = false;
    private final double hoodStep = .01;
    public double avgRPM;
    private boolean autoAim = false;
    private boolean xWasPressed = false;

    public double targetHood = 0.5;

    public void init(HardwareMap hwMap) {
        leftFlywheel = hwMap.get(DcMotorEx.class, "1");
        rightFlywheel = hwMap.get(DcMotorEx.class, "2");
        hood = hwMap.get(Servo.class,"hood");
        leftFlywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightFlywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftFlywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        rightFlywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        pid = new NeroFlywheelPIDF(0.0000001,0.0000001,0.000010,0.000195);

    }

    public double RPM(double distance){
        if(distance > 71 && distance < 128){
            speed = -307 + 78.7 * distance - 0.297 * Math.pow(distance, 2);
        }else {
            if(distance < 50){
                speed = 3775;
            }
            if (distance < 71 && distance > 50) {
                speed = 3850;
            }
            if (distance > 128){
                speed = 4900;
            }
        }
        speed = MathFunctions.clamp(speed,0,4900);
        return speed;
    }
    public double hood(double distance){
            if (distance > 34 && distance < 80) {
                hoodPos = -1.21 + 0.0626 * distance - 0.000961 * Math.pow(distance , 2) + 0.00000504 * Math.pow(distance , 3);
            } else {
                if(distance > 80 && distance < 96.23819) {
                    hoodPos = 0.23;
                }
                if(distance > 96.23819) {
                    hoodPos = -48.7 + 1.15 * distance - 0.00888 * Math.pow(distance, 2) + 0.000023 * Math.pow(distance, 3);
                }
            }

        hoodPos = MathFunctions.clamp(hoodPos,0,1);
        return hoodPos;
    }

//    private double AutoAim(boolean input, boolean isValid, double tx){
//        if (input && !xWasPressed) {
//            autoAim = !autoAim;
//        }
//        xWasPressed = input;
//
//        if (autoAim && isValid) {
//            double kP = 0.01;
//            turn = kP * tx;
//            if (Math.abs(tx) < 1.0) {
//                turn = 0;
//            }
//        }
//        return turn;
//    }

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

    public void setHood(boolean up, boolean down){
        if (up && !leftPressed) {
            targetHood += hoodStep;
            leftPressed = true;
        }
        if (!up) {
            leftPressed = false;
        }
        if (down && !rightPressed) {
            targetHood -= hoodStep;
            if (targetHood < 0) targetHood = 0;
            rightPressed = true;
        }
        if (!down) {
            rightPressed = false;
        }
    }

    public static double distance2D(double x1, double y1, double x2, double y2) {
        double dx = x2 - x1;
        double dy = y2 - y1;
        return Math.sqrt(dx * dx + dy * dy);
    }

    public double tyDistance(double ty){
        double camera_height = 13.0295039;
        double tag_height = 29.5;
        double angle_mount = 25;
        double theta_offset = 0;
        double distance;
        if (ty>14){
            distance = 20;
        } else{
            distance = (tag_height - camera_height)/(Math.tan(((Math.PI)/180)*(ty + angle_mount + theta_offset)));
        }
        return distance;
    }

    public void ShooterGo(boolean input){
        double leftVel = leftFlywheel.getVelocity();
        double rightVel = rightFlywheel.getVelocity();
        avgRPM = (leftVel + rightVel) / 2 / 28.0 * 60.0;
        flywheelPower = (pid.calculate(speed, avgRPM));
        if (input && !lastButtonState) {
            motorRunning = !motorRunning;
        }
        if (!motorRunning){
            pid.reset();
        }
        leftFlywheel.setPower(motorRunning? 0 : flywheelPower);
        rightFlywheel.setPower(motorRunning? 0 : flywheelPower);
        hood.setPosition(hoodPos);
        lastButtonState = input;

        if(!motorRunning){
            pid.reset();
        }


    }

    public void ShooterTune(boolean input){
        double leftVel = leftFlywheel.getVelocity();
        double rightVel = rightFlywheel.getVelocity();
        avgRPM = (leftVel + rightVel) / 2 / 28.0 * 60.0;
        flywheelPower = (pid.calculate(targetRPM, avgRPM));
        if (input && !lastButtonState) {
            motorRunning = !motorRunning;
        }
        if (!motorRunning){
            pid.reset();
        }
        leftFlywheel.setPower(motorRunning? 0 : flywheelPower);
        rightFlywheel.setPower(motorRunning? 0 : flywheelPower);
        hood.setPosition(targetHood);
        lastButtonState = input;

        if(!motorRunning){
            pid.reset();
        }


    }


}
