package org.firstinspires.ftc.teamcode.Nero.Mecanisms;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class AutoAim {

    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    private IMU imu;

    // ================= AUTO AIM =================
    private boolean autoAimEnabled = false;
    private boolean lastToggleState = false;

    private double kP = 0.025;
    private double kD = 0.025;


    private double lastError = 0;
    private long lastTime = System.nanoTime();

    // target field position (SET THIS FROM OPMODE)
    private double targetX = 0;
    private double targetY = 0;

    // ============================================

    public void init(HardwareMap hwMap) {
        frontLeftMotor = hwMap.get(DcMotor.class, "leftFront");
        backLeftMotor = hwMap.get(DcMotor.class, "leftBack");
        frontRightMotor = hwMap.get(DcMotor.class, "rightFront");
        backRightMotor = hwMap.get(DcMotor.class, "rightBack");

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hwMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);

        imu.initialize(new IMU.Parameters(RevOrientation));
    }

    // ================= PUBLIC CONTROL =================

    public void setAutoAimTarget(double x, double y) {
        this.targetX = x;
        this.targetY = y;
    }

    public void toggleAutoAim(boolean buttonPressed) {
        if (buttonPressed && !lastToggleState) {
            autoAimEnabled = !autoAimEnabled;
        }
        lastToggleState = buttonPressed;
    }

    // Call this from TeleOp when using Pedro
    public void driveFieldRelativeAutoAim(
            double y,
            double x,
            double manualTurn,
            double robotX,
            double robotY,
            double robotHeadingRadians
    ) {

        double turn;

        if (autoAimEnabled) {
            turn = computeAutoAimTurn(robotX, robotY, robotHeadingRadians);
        } else {
            turn = manualTurn;
        }

        driveRobotRelative(y, x, turn);
    }

    // ================= AUTO AIM PID =================

    private double computeAutoAimTurn(double robotX, double robotY, double robotHeading) {

        double targetAngle = Math.atan2(targetY - robotY, targetX - robotX);
        double error = wrapAngle(targetAngle - robotHeading);

        long now = System.nanoTime();
        double dt = (now - lastTime) / 1e9;
        lastTime = now;

        double derivative = (error - lastError) / dt;
        lastError = error;

        return kP * error + kD * derivative;
    }

    private double wrapAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    // ================= YOUR ORIGINAL DRIVE =================

    public void drive(double power, double theta, double turn) {

        double sin = Math.sin(theta - Math.PI / 4);
        double cos = Math.cos(theta - Math.PI / 4);
        double max = Math.max(Math.abs(sin), Math.abs(cos));

        double leftFront = power * cos / max - turn;
        double rightFront = power * sin / max + turn;
        double leftRear = power * sin / max - turn;
        double rightRear = power * cos / max + turn;

        if ((power + Math.abs(turn)) > 1) {
            leftFront /= power + Math.abs(turn);
            rightFront /= power + Math.abs(turn);
            leftRear /= power + Math.abs(turn);
            rightRear /= power + Math.abs(turn);
        }

        frontLeftMotor.setPower(leftFront);
        frontRightMotor.setPower(rightFront);
        backRightMotor.setPower(rightRear);
        backLeftMotor.setPower(leftRear);
    }

    public void driveFieldRelative(double y, double x, double turn) {
        double theta = Math.atan2(y, x);
        double r = Math.hypot(x, y);

        theta = AngleUnit.normalizeRadians(
                theta - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)
        );

        this.drive(r, theta, turn);
    }

    public void driveRobotRelative(double y, double x, double turn) {
        double theta = Math.atan2(y, x);
        double r = Math.hypot(x, y);

        this.drive(r, theta, turn);
    }
}