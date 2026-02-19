package org.firstinspires.ftc.teamcode.Nero.Auton;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Nero.Mecanisms.file;

@Autonomous(name="Drive Forward")
public class DriveForward extends OpMode {

    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    private final ElapsedTime timer = new ElapsedTime();
    private Follower follower;
    private Pose startPose = new Pose (55, 7.75 , Math.toRadians(180));

    private boolean movementStarted = false;
    private boolean movementCompleted = false;
    file fileManager = new file();

    @Override
    public void init() {
        frontLeftMotor = hardwareMap.get(DcMotor.class, "leftFront");
        backLeftMotor = hardwareMap.get(DcMotor.class, "leftBack");
        frontRightMotor = hardwareMap.get(DcMotor.class, "rightFront");
        backRightMotor = hardwareMap.get(DcMotor.class, "rightBack");
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        follower.setPose(startPose);
        fileManager.init();
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void start() {
        timer.reset();
    }

    @Override
    public void loop() {
        if (!movementStarted) {
            frontLeftMotor.setPower(0.5);
            backLeftMotor.setPower(0.5);
            frontRightMotor.setPower(0.5);
            backRightMotor.setPower(0.5);
            movementStarted = true;
        }
        else if (timer.seconds() >= 0.5 && !movementCompleted) {
            frontLeftMotor.setPower(0);
            backLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            backRightMotor.setPower(0);
            movementCompleted = true;
        }
    }

    @Override
    public void stop() {
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
        fileManager.FileWrite(follower.getPose().getX(),follower.getPose().getY(),follower.getHeading());
    }
}
