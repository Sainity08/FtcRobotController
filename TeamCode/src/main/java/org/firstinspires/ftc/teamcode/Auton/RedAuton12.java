package org.firstinspires.ftc.teamcode.Auton;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Mechanisms.Intake;
import org.firstinspires.ftc.teamcode.PID.FlywheelPID;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Red 12 Ball Auton")
public class RedAuton12 extends OpMode {

    private Follower follower;
    private boolean flywheelOn = false;
    private ElapsedTime ShooterTimer = new ElapsedTime();
    private ElapsedTime opmodeTimer = new ElapsedTime();
    Intake intake = new Intake();
    public DcMotorEx leftFlywheel = null;
    public DcMotorEx rightFlywheel = null;
    private Limelight3A limelight;
    private double previousPid = 0.0;   // initialize PID contribution
    private double previousPower = 0.0;
    private FlywheelPID pid;
    boolean intakeOn = false;

    public DcMotorEx SIntake;

    public enum PathState {
        STARTPOSE_SHOOT1POSE,
        SHOOT1,
        INTAKE1ALIGN,
        INTAKE1ALIGN_INTAKE1POSE,
        INTAKE1POSE_LEVERPOSE,
        LEVERPOSE_SHOOT2POSE,
        SHOOT2,
        INTAKE2ALIGN,
        INTAKE2ALIGN_INTAKE2POSE,
        INTAKE2POSE_SHOOT3POSE,
        SHOOT3,
        INTAKE3ALIGN,
        INTAKE3ALIGN_INTAKE3POSE,
        INTAKE3POSE_SHOOT4,
        SHOOT4,
        SHOOT4_LEVERPOSITION,
    }
    PathState pathState;
    double shootAngle = (228.22);

    private final Pose startPose = new Pose(111.3, 134.1, (Math.toRadians(270)));
    public PathChain Turn;
    public PathChain Path2;
    public PathChain Path3;
    public PathChain Path4;
    public PathChain Path5;
    public PathChain Path6;
    public PathChain Path7;
    public PathChain Path8;
    public PathChain Path9;
    public PathChain Path10;
    public PathChain Path11;
    public PathChain Path12;




    public void buildPaths(){
        Turn = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(111.3, 134.1), new Pose(88, 88.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(shootAngle))
                .build();

        Path2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(88.000, 88.000), new Pose(88, 84))
                )
                .setLinearHeadingInterpolation(Math.toRadians(shootAngle), Math.toRadians(0))
                .build();

        Path3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(88, 84), new Pose(200, 84))
                )
                .setTangentHeadingInterpolation()
                .build();

        Path4 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(120, 84), new Pose(129, 73))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))
                .build();

        Path5 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(129, 73), new Pose(88, 88))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(shootAngle))
                .build();

        Path6 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(88, 88), new Pose(88, 60))
                )
                .setLinearHeadingInterpolation(Math.toRadians(shootAngle), Math.toRadians(0))
                .build();

        Path7 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(88, 60), new Pose(200, 60))
                )
                .setTangentHeadingInterpolation()
                .build();

        Path8 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(120, 59.045), new Pose(88, 88))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(shootAngle))
                .build();
        Path9 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(88, 88), new Pose(88, 37))
                )
                .setLinearHeadingInterpolation(Math.toRadians(shootAngle), Math.toRadians(0))
                .build();
        Path10 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(88, 37), new Pose(200, 37))
                )
                .setTangentHeadingInterpolation()
                .build();
        Path11 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(120, 37), new Pose(88, 88))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(shootAngle))
                .build();
        Path12 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(88, 88), new Pose(125, 70))
                )
                .setLinearHeadingInterpolation(Math.toRadians(shootAngle), Math.toRadians(0))
                .build();
    }
    boolean ShooterDone = false;
    boolean ShooterDone1 = false;
    boolean ShooterDone2 = false;
    boolean ShooterDone3 = false;
    boolean IntakeDone = false;
    boolean IntakeDone1 = false;
    boolean IntakeDone2 = false;
    public void statePathUpdate() {
        switch (pathState) {
            case STARTPOSE_SHOOT1POSE:

                if (!follower.isBusy()) {
                    follower.followPath(Turn, true);
                    if ((((follower.getPose().getX()) > 88)) && ((follower.getPose().getY()) > 88)) {
                        pathState = PathState.SHOOT1;
                        ShooterTimer.reset();
                    }
                }
                break;


            case SHOOT1:
                if (!follower.isBusy()) {
                    if (ShooterTimer.seconds() > 5) {
                        flywheelOn = false;
                        pathState = PathState.INTAKE1ALIGN;
                        ShooterDone = true;
                    } else {
                        flywheelOn = true;
                    }
                }
                break;
            case INTAKE1ALIGN:
                if (!follower.isBusy()) {
                    if (ShooterDone) {
                        follower.followPath(Path2, true);
                        pathState = PathState.INTAKE1ALIGN_INTAKE1POSE;
                        break;
                    }
                }

            case INTAKE1ALIGN_INTAKE1POSE:
                if (!follower.isBusy()) {
                    if (ShooterDone) {
                        follower.followPath(Path3, 0.75, true);
                        if (!follower.isBusy() | (follower.getPose().getX()) > 118) {
                            pathState = PathState.INTAKE1POSE_LEVERPOSE;
                            break;
                        }
                    }
                }
            case INTAKE1POSE_LEVERPOSE:
                if (!follower.isBusy()) {
                    if (ShooterDone) {
                        follower.followPath(Path4, true);
                        if (!follower.isBusy() | (follower.getPose().getX()) > 120) {
                            pathState = PathState.LEVERPOSE_SHOOT2POSE;
                            IntakeDone = true;
                            break;
                        }
                    }
                }
            case LEVERPOSE_SHOOT2POSE:
                if (!follower.isBusy()) {
                    if (IntakeDone) {
                        follower.followPath(Path5, true);
                        pathState = PathState.SHOOT2;
                        ShooterTimer.reset();
                        break;
                    }
                }
            case SHOOT2:
                if (!follower.isBusy()) {
                    if (ShooterTimer.seconds() > 5) {
                        flywheelOn = false;
                        pathState = PathState.INTAKE2ALIGN;
                        ShooterDone1 = true;
                    } else {
                        flywheelOn = true;
                    }
                }
                break;
            case INTAKE2ALIGN:
                if (!follower.isBusy()) {
                    if (ShooterDone1) {
                        follower.followPath(Path6, true);
                        pathState = PathState.INTAKE2ALIGN_INTAKE2POSE;
                        break;
                    }
                }

            case INTAKE2ALIGN_INTAKE2POSE:
                if (!follower.isBusy()) {
                    if (ShooterDone1) {
                        follower.followPath(Path7, 0.75, true);
                        if (!follower.isBusy() | (follower.getPose().getX()) > 120) {
                            pathState = PathState.INTAKE2POSE_SHOOT3POSE;
                            IntakeDone1 = true;
                            break;
                        }
                    }
                }
            case INTAKE2POSE_SHOOT3POSE:
                if (!follower.isBusy()) {
                    if (IntakeDone1) {
                        follower.followPath(Path8, true);
                        pathState = PathState.SHOOT3;
                        ShooterTimer.reset();
                        break;
                    }
                }
            case SHOOT3:
                if (!follower.isBusy()) {
                    if (ShooterTimer.seconds() > 5) {
                        flywheelOn = false;
                        pathState = PathState.INTAKE3ALIGN;
                        ShooterDone2 = true;
                    } else {
                        flywheelOn = true;
                    }
                }
                break;
            case INTAKE3ALIGN:
                if (!follower.isBusy()) {
                    if (ShooterDone2) {
                        follower.followPath(Path9, false);
                        pathState = PathState.INTAKE3ALIGN_INTAKE3POSE;
                        break;
                    }
                }
            case INTAKE3ALIGN_INTAKE3POSE:
                if (!follower.isBusy()) {
                    if (ShooterDone1) {
                        follower.followPath(Path10,0.75, true);
                        if (!follower.isBusy() | (follower.getPose().getX()) > 122) {
                            pathState = PathState.INTAKE3POSE_SHOOT4;
                            IntakeDone2 = true;
                            break;
                        }
                    }
                }
            case INTAKE3POSE_SHOOT4:
                if (!follower.isBusy()) {
                    if (IntakeDone) {
                        follower.followPath(Path11, true);
                        pathState = PathState.SHOOT4;
                        ShooterTimer.reset();
                        break;
                    }
                }

            case SHOOT4:
                if (!follower.isBusy()) {
                    if (ShooterTimer.seconds() > 5) {
                        flywheelOn = false;
                        pathState = PathState.SHOOT4_LEVERPOSITION;
                        ShooterDone3 = true;
                    } else {
                        flywheelOn = true;
                    }
                }
            case SHOOT4_LEVERPOSITION:
                if (!follower.isBusy()) {
                    if (ShooterDone3) {
                        follower.followPath(Path12, true);
                        if (!follower.isBusy() | (follower.getPose().getX()) > 122) {
                            IntakeDone2 = true;
                            break;
                        }
                    }
                }
        }
    }

    public void setPathState(PathState newState){
        pathState = newState;
    }

    @Override
    public void init() {
        pathState = PathState.STARTPOSE_SHOOT1POSE;
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setPose(startPose);
        opmodeTimer = new ElapsedTimer();
        ShooterTimer = new ElapsedTimer();

        intake.init(hardwareMap);
        leftFlywheel = hardwareMap.get(DcMotorEx.class, "flywheelL");
        rightFlywheel = hardwareMap.get(DcMotorEx.class, "flywheelR");
        SIntake = hardwareMap.get(DcMotorEx.class, "intake2");
        leftFlywheel.setDirection(DcMotorEx.Direction.REVERSE);
        rightFlywheel.setDirection(DcMotorEx.Direction.FORWARD);
        leftFlywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightFlywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftFlywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        rightFlywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        pid = new FlywheelPID(0.015, 0.0005, 0.005);
        previousPid = 0.0;
        previousPower = 0.0;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);
    }

    @Override
    public void start() {
        limelight.start();
        setPathState(pathState);
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();
        telemetry.addData("Path State", pathState.toString());
        telemetry.addData("follower is busy", follower.isBusy());
        telemetry.addData("shooter timer", ShooterTimer.seconds());
        telemetry.addData("OpMode Timer", opmodeTimer.seconds());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));


        boolean input1 = true;
        intake.NonStationary(input1);
        double distance = 62.5;
        double targetRPM = -0.000121 * Math.pow(distance, 4)
                    + 0.0339 * Math.pow(distance, 3)
                    - 3.29 * Math.pow(distance, 2)
                    + 147 * distance
                    + 1214;


        double kF = flywheelOn ? 0.7 : 0.0; //flywheelOn triggers flywheel based on true/false
        double leftVel = leftFlywheel.getVelocity();
        double rightVel = rightFlywheel.getVelocity();
        double avgRPM = (leftVel + rightVel) / 2 / 28.0 * 60.0;
        double pidOut = pid.calculate(targetRPM, avgRPM, 0.02);
        double power;
        if (!flywheelOn) {
            power = 0;
            previousPid = 0;
            pid.reset();
        } else {
            double kFAdjusted = kF * Math.max(0, (targetRPM - avgRPM) / targetRPM);
            double targetPower = kFAdjusted + pidOut;
            double rampRate = 0.01;
            power = previousPower + Math.signum(targetPower - previousPower) * rampRate;
            power = Math.max(0, Math.min(power, 1));
            previousPower = power;
        }
        leftFlywheel.setPower(power);
        rightFlywheel.setPower(power);


        double bounds = 300;
        if (!intakeOn) {
            if (avgRPM < (targetRPM + bounds) && avgRPM > (targetRPM - bounds)) {
                intakeOn = true;
            }
        } else {
            intakeOn = false;
        }

        SIntake.setPower(intakeOn? 1 : 0);
    }
}
