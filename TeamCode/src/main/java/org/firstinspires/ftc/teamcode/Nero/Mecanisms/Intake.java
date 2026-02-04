package org.firstinspires.ftc.teamcode.Nero.Mecanisms;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Intake {
    private DcMotor leftIntake, rightIntake;
    public Servo hardstop;
    public boolean lastButtonState = false;
    public boolean motorRunning = false;
    public boolean lastButtonState1 = false;
    public boolean motorRunning1 = false;
    public boolean lastButtonState2 = false;
    public boolean motorRunning2 = false;

    public void init(HardwareMap hwMap) {
        leftIntake = hwMap.get(DcMotor.class, "leftI");
        rightIntake = hwMap.get(DcMotor.class, "rightI");
        hardstop = hwMap.get(Servo.class, "hardstop");
        leftIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        rightIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void intake(boolean input1) {

        if (input1 && !lastButtonState) {
            motorRunning = !motorRunning;
        }
        leftIntake.setPower(motorRunning ? 1 : 0.0);
        rightIntake.setPower(motorRunning ? 1 : 0.0);
        lastButtonState = input1;
        if(motorRunning){
            hardstop.setPosition(0);
        }
    }

    public void intakeShoot(boolean input2){
        if (input2 && !lastButtonState1) {
            motorRunning1 = !motorRunning1;
        }
        hardstop.setPosition(motorRunning1? 0:1);
        leftIntake.setPower(motorRunning1 ? 0.8 : 0.0);
        rightIntake.setPower(motorRunning1 ? 0.8 : 0.0);
        lastButtonState1 = input2;
    }

    public void outtake(boolean input3){
        if (input3 && !lastButtonState2) {
            motorRunning2 = !motorRunning2;
        }
        leftIntake.setPower(motorRunning2 ? -.5 : 0.0);
        rightIntake.setPower(motorRunning2 ? -.5 : 0.0);
        lastButtonState2 = input3;
    }

}
