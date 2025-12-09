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

@Autonomous(name = "Blue 9 Ball Auton")
public class BlueAuton9 extends OpMode {

    private Follower follower;
    private boolean flywheelOn = false;
    private ElapsedTime ShooterTimer = new ElapsedTime();
    private Timer opmodeTimer;
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
        INTAKE1POSE_SHOOT2POSE,
        SHOOT2,
        INTAKE2ALIGN,
        INTAKE2ALIGN_INTAKE2POSE,
        INTAKE2POSE_SHOOT3POSE,
        SHOOT3,
        INTAKE3ALIGN,
        INTAKE3ALIGN_INTAKE3POSE,
        INTAKE3POSE_LEVERPOSITION,
    }
    PathState pathState;
    double shootAngle = (308.22);

    private final Pose startPose = new Pose(32.7, 134.1, (Math.toRadians(270)));
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



    public void buildPaths(){
        Turn = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(32.7, 134.1), new Pose(56, 88.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(shootAngle))
                .build();

        Path2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(56, 88.000), new Pose(56, 84))
                )
                .setLinearHeadingInterpolation(Math.toRadians(shootAngle), Math.toRadians(180))
                .build();

        Path3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(56, 84), new Pose(0, 84))
                )
                .setTangentHeadingInterpolation()
                .build();

        Path4 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(24, 84), new Pose(56, 88))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(shootAngle))
                .build();

        Path5 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(56, 88), new Pose(56, 60))
                )
                .setLinearHeadingInterpolation(Math.toRadians(shootAngle), Math.toRadians(180))
                .build();

        Path6 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(56, 60), new Pose(0, 60))
                )
                .setTangentHeadingInterpolation()
                .build();

        Path7 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(24, 59.045), new Pose(56, 88))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(shootAngle))
                .build();
        Path8 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(56, 88), new Pose(56, 37))
                )
                .setLinearHeadingInterpolation(Math.toRadians(shootAngle), Math.toRadians(180))
                .build();
        Path9 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(56, 37), new Pose(0, 37))
                )
                .setTangentHeadingInterpolation()
                .build();
        Path10 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(24, 37), new Pose(24, 200))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
    }
    boolean ShooterDone = false;
    boolean ShooterDone1 = false;
    boolean ShooterDone2 = false;
    boolean IntakeDone = false;
    boolean IntakeDone1 = false;
    boolean IntakeDone2 = false;
    public void statePathUpdate() {
        switch (pathState) {
            case STARTPOSE_SHOOT1POSE:

                if (!follower.isBusy()) {
                    follower.followPath(Turn, true);
                    if ((((follower.getPose().getX()) > 56)) && ((follower.getPose().getY()) < 88)) {
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
                        follower.followPath(Path3, true);
                        if (!follower.isBusy() | (follower.getPose().getX()) < 24) {
                            pathState = PathState.INTAKE1POSE_SHOOT2POSE;
                            IntakeDone = true;
                            break;
                        }
                    }
                }
            case INTAKE1POSE_SHOOT2POSE:
                if (!follower.isBusy()) {
                    if (IntakeDone) {
                        follower.followPath(Path4, true);
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
                        follower.followPath(Path5, true);
                        pathState = PathState.INTAKE2ALIGN_INTAKE2POSE;
                        break;
                    }
                }

            case INTAKE2ALIGN_INTAKE2POSE:
                if (!follower.isBusy()) {
                    if (ShooterDone1) {
                        follower.followPath(Path6, true);
                        if (!follower.isBusy() | (follower.getPose().getX()) < 24) {
                            pathState = PathState.INTAKE2POSE_SHOOT3POSE;
                            IntakeDone1 = true;
                            break;
                        }
                    }
                }
            case INTAKE2POSE_SHOOT3POSE:
                if (!follower.isBusy()) {
                    if (IntakeDone1) {
                        follower.followPath(Path7, true);
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
                        follower.followPath(Path8, false);
                        pathState = PathState.INTAKE3ALIGN_INTAKE3POSE;
                        break;
                    }
                }
            case INTAKE3ALIGN_INTAKE3POSE:
                if (!follower.isBusy()) {
                    if (ShooterDone1) {
                        follower.followPath(Path9, true);
                        if (!follower.isBusy() | (follower.getPose().getX()) < 22) {
                            pathState = PathState.INTAKE3POSE_LEVERPOSITION;
                            IntakeDone2 = true;
                            break;
                        }
                    }
                }
            case INTAKE3POSE_LEVERPOSITION:
                if (!follower.isBusy()) {
                    if (IntakeDone2) {
                        follower.followPath(Path10, true);
                        if ((follower.getPose().getY()) > 68) {
                            follower.breakFollowing();
                        }
                        break;
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
        opmodeTimer = new Timer();
        ShooterTimer = new ElapsedTime();

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
