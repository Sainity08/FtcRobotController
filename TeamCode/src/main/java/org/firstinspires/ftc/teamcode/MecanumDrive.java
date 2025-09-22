package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class MecanumDrive {
    private DcMotor frontLeftMotor,backLeftMotor,frontRightMotor,backRightMotor;
    private IMU imu;
    public void init(HardwareMap hwMap) {
        frontLeftMotor = hwMap.get(DcMotor.class, "motor1");
        backLeftMotor = hwMap.get(DcMotor.class, "motor3");
        frontRightMotor = hwMap.get(DcMotor.class, "motor0");
        backRightMotor = hwMap.get(DcMotor.class, "motor2");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hwMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);

        imu.initialize(new IMU.Parameters(RevOrientation));
    }

    public void drive(double power, double theta, double turn) {

        sin = Math.sin(theta - Math.PI/4);
        cos = Math.cos(theta - Math.PI/4);
        max = Math.max(Math.abs(sin),Math.abs(cos));
        speed = 0.75;
        newPower = power * speed;

        leftFront = newPower * cos/max;
        rightFront = newPower * sin/max;
        leftRear = newPower * sin/max;
        rightRear = newPower * cos/max;

        If ((power + Math.abs(turn))>1){
            leftFront /= newPower + Math.abs(turn);
            rightFront /= newPower + Math.abs(turn);
            leftRear /= newPower + Math.abs(turn);
            rightRear /= newPower + Math.abs(turn);
        }
    }
}
