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

    public void init() {
        leftIntake = hardwareMap.get(DcMotor.class, "leftI");
        rightIntake = hardwareMap.get(DcMotor.class, "rightI");
        hardstop = hardwareMap.get(Servo.class, "hardstop");
        leftIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        rightIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void intake(boolean input1, boolean input2) {

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
}
