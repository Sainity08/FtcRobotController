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

        double sin = Math.sin(theta - Math.PI/4);
        double cos = Math.cos(theta - Math.PI/4);
        double max = Math.max(Math.abs(sin),Math.abs(cos));
        double speed = 0.75;
        double newPower = power * speed;

        double leftFront = newPower * cos/max;
        double rightFront = newPower * sin/max;
        double leftRear = newPower * sin/max;
        double rightRear = newPower * cos/max;

        if ((power + Math.abs(turn))>1) {
            leftFront /= newPower + Math.abs(turn);
            rightFront /= newPower + Math.abs(turn);
            leftRear /= newPower + Math.abs(turn);
            rightRear /= newPower + Math.abs(turn);
        }
    }

    public void driveFieldRelative(double y, double x, double turn){
        double theta = Math.atan2(y,x);
        double r = Math.hypot(x,y);

        theta = AngleUnit.normalizeRadians(theta - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        this.drive(r,theta,turn);
    }

    public void driveRobotRelative(double y, double x, double turn){
        double theta = Math.atan2(y,x);
        double r = Math.hypot(x,y);

        this.drive(r,theta,turn);
    }
}
