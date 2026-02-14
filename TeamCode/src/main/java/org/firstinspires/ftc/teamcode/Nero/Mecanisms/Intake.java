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
    Shooter shooter = new Shooter();
    private double bounds = 75;
    public double intakePower;
    public double actualPower;
    public boolean canIntake;
    private double hardstopPos;

    public void init(HardwareMap hwMap) {
        leftIntake = hwMap.get(DcMotor.class, "leftI");
        rightIntake = hwMap.get(DcMotor.class, "rightI");
        hardstop = hwMap.get(Servo.class, "hardstop");
        leftIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        rightIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void intake(boolean intake, boolean outtake, boolean scoring, double distance) {

        if (intake && !lastButtonState) {
            motorRunning = !motorRunning;
        }
        lastButtonState = intake;

        if (motorRunning) {
            intakePower = 1;
            hardstopPos = 1;
        } else {
            if (outtake) {
                intakePower = -.5;
                hardstopPos = 0;
            } else {
                if (scoring && shooter.avgRPM < (shooter.speed + bounds) && shooter.avgRPM > (shooter.speed - bounds)) {
                    if (distance <= 120){
                        intakePower = 0.75;
                        hardstopPos = 0;
                    }
                    if (distance > 120) {
                        intakePower = 0.25;
                        hardstopPos = 0;
                    }
                } else {
                    intakePower = 0;
                }
            }
        }

        leftIntake.setPower(intakePower);
        rightIntake.setPower(intakePower);
        hardstop.setPosition(hardstopPos);
        actualPower = (((leftIntake.getPower()) + rightIntake.getPower()) / 2);
        canIntake = (shooter.avgRPM < (shooter.targetRPM + bounds) && shooter.avgRPM > (shooter.targetRPM - bounds));
    }

    public void autonIntake(boolean intake, boolean outtake, boolean scoring) {

        if(intake){
            intakePower = 1;
            hardstopPos = 1;
        } else{
            if(outtake){
                intakePower = -.5;
                hardstopPos = 1;
            } else {
                if(scoring){
                    intakePower = 0.75;
                    hardstopPos = 0;
                }
                else{
                    intakePower = 0;
                }
            }
        }

        leftIntake.setPower(intakePower);
        rightIntake.setPower(intakePower);
        hardstop.setPosition(hardstopPos);
    }
    public void backAutonIntake(boolean intake, boolean outtake, boolean scoring) {

        if(intake){
            intakePower = 1;
            hardstopPos = 1;
        } else{
            if(outtake){
                intakePower = -.5;
                hardstopPos = 1;
            } else {
                if(scoring){
                    intakePower = 0.25;
                    hardstopPos = 0;
                }
                else{
                    intakePower = 0;
                }
            }
        }

        leftIntake.setPower(intakePower);
        rightIntake.setPower(intakePower);
        hardstop.setPosition(hardstopPos);
    }

//    public void intakeShoot(boolean input2){
//        hardstop.setPosition(input2? 0:1);
//        leftIntake.setPower(input2 ? 0.8 : 0.0);
//        rightIntake.setPower(input2 ? 0.8 : 0.0);
//    }
//
//    public void outtake(boolean input3) {
//        leftIntake.setPower(input3 ? -.5 : 0.0);
//        rightIntake.setPower(input3 ? -.5 : 0.0);
//    }

}
