package org.firstinspires.ftc.teamcode.Nero.Auton;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Nero.Mecanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.Nero.Mecanisms.Intake;
import org.firstinspires.ftc.teamcode.Nero.Mecanisms.Shooter;
import org.firstinspires.ftc.teamcode.Nero.PID.NeroFlywheelPIDF;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class Blue12 extends OpMode {
    private double distance = 31.1126983722;

    private Limelight3A limelight;
    Intake intake = new Intake();
    Drivetrain drive = new Drivetrain();
    NeroFlywheelPIDF pid;
    public Servo hood, leftT, rightT;
    boolean intaking = true;
    boolean scoring = false;
    private ElapsedTime ShooterTimer = new ElapsedTime();
    Shooter shooter = new Shooter();
    public DcMotorEx leftFlywheel = null;
    public DcMotorEx rightFlywheel = null;
    public PathChain Path1;
    public PathChain Path2;
    public PathChain Path3;
    public PathChain Path4;
    public PathChain Path5;
    public PathChain Path6;
    public PathChain Path7;
    public PathChain Path8;
    public PathChain Path9;
    private final Pose startPose = new Pose(34, 136.000, Math.toRadians(270));
    private Follower follower;
    double shootAngle = 315;

    public void buildPaths() {
        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(34.000, 136.000),

                                new Pose(37.000, 106.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(shootAngle-6))

                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(37.000, 106.000),
                                new Pose(73.629, 79.998),
                                new Pose(14.753, 83.199)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(13.690, 83.863),
                                new Pose(51.869, 79.729),
                                new Pose(37.022, 106.022)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(shootAngle-3))

                .build();

        Path4 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(37.022, 106.022),
                                new Pose(60.736, 52.349),
                                new Pose(12, 59.672)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();


        Path5 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(14.015, 59.672),
                                new Pose(32.985, 66.875),
                                new Pose(13.948, 75)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        Path6 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(13.948, 75),
                                new Pose(53.980, 66.400),
                                new Pose(37.000, 106.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(shootAngle-3))

                .build();

        Path7 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(37.000, 106.000),
                                new Pose(70.681, 26.138),
                                new Pose(12.554, 35.554)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path8 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(12.554, 35.554),
                                new Pose(59.260, 26.744),
                                new Pose(37.000, 106.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(shootAngle-1))

                .build();

        Path9 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(37.000, 106.000),
                                new Pose(49.845, 67.330),
                                new Pose(24, 69.561)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();
    }

    public enum PathState {
        STARTPOSE_SHOOT1POSE,
        SHOOT1,
        INTAKE1,
        INTAKE1_SHOOT2POSE,
        SHOOT2,
        INTAKE2,
        INTAKE2_LEVER,
        PRESSLEVER,
        LEVER_SHOOT3POSE,
        SHOOT3,
        INTAKE3,
        INTAKE3_SHOOT4POSE,
        SHOOT4,
        SHOOT4_LEVER,
    }

    boolean ShooterDone = false;
    boolean IntakeDone = false;
    boolean ShooterDone1 = false;
    boolean IntakeDone1 = false;
    boolean ShooterDone2 = false;
    boolean ShooterDone3 = false;
    boolean IntakeDone2 = false;
    boolean done = false;
    boolean autoAim;
    PathState pathState;
    double turn = 0;

    public void statePathUpdate() {
        switch (pathState) {
            case STARTPOSE_SHOOT1POSE:
                if (!follower.isBusy()) {
                    intaking = true;
                    follower.followPath(Path1, true);
                    if ((((follower.getPose().getX() < 107) && (follower.getPose().getY() < 106)))) {
                        pathState = PathState.SHOOT1;
                        ShooterTimer.reset();
                    }
                }
                break;
            case SHOOT1:
                if (!follower.isBusy()) {
                    if (ShooterTimer.seconds() > 2) {
                        pathState = PathState.INTAKE1;
                        ShooterDone = true;
                        intaking = true;
                        scoring = false;
                        autoAim = false;
                    } else {
                        scoring = true;
                        intaking = false;
                        autoAim = true;
                    }
                }
            case INTAKE1:
                if (!follower.isBusy()) {
                    if (ShooterDone) {
                        follower.followPath(Path2, true);
                        pathState = PathState.INTAKE1_SHOOT2POSE;
                        break;
                    }
                }
            case INTAKE1_SHOOT2POSE:
                if (!follower.isBusy()) {
                    if (ShooterDone) {
                        follower.followPath(Path3,true);
                        if (!follower.isBusy() | ((((follower.getPose().getX() < 107) && (follower.getPose().getY() < 106))))) {
                            pathState = PathState.SHOOT2;
                            ShooterTimer.reset();
                            IntakeDone = true;
                            break;
                        }
                    }
                }

            case SHOOT2:
                if (!follower.isBusy()) {
                    if (ShooterTimer.seconds() > 3.5){
                        pathState = PathState.INTAKE2;
                        ShooterDone1 = true;
                        intaking = true;
                        scoring = false;
                        autoAim = false;
                    } else {
                        scoring = true;
                        intaking = false;
                        autoAim = true;
                    }
                }

            case INTAKE2:
                if (!follower.isBusy()) {
                    if (ShooterDone1) {
                        follower.followPath(Path4);
                        pathState = PathState.INTAKE2_LEVER;
                        break;
                    }
                }
            case INTAKE2_LEVER:
                if (!follower.isBusy()) {
                    if (ShooterDone1) {
                        follower.followPath(Path5, true);
                        if (!follower.isBusy() | (follower.getPose().getX()) < 20.65) {
                            pathState = PathState.PRESSLEVER;
                            ShooterTimer.reset();
                            IntakeDone1 = true;
                            break;
                        }
                    }
                }
            case PRESSLEVER:
                if (!follower.isBusy() && ShooterDone1) {
                    if(ShooterTimer.seconds() > 3){
                        pathState = PathState.LEVER_SHOOT3POSE;
                        scoring = false;
                        break;
                    } else {
                        scoring = false;
                    }

                }
            case LEVER_SHOOT3POSE:
                if (!follower.isBusy()) {
                    if (IntakeDone1) {
                        follower.followPath(Path6, true);
                        if (!follower.isBusy() | follower.getPose().getY() > 106) {
                            pathState = PathState.SHOOT3;
                            done = true;
                            ShooterTimer.reset();
                            break;
                        }
                    }
                }
            case SHOOT3:
                if (!follower.isBusy() && IntakeDone1 && done) {
                    if (ShooterTimer.seconds() > 2.5) {
                        pathState = PathState.INTAKE3;
                        ShooterDone2 = true;
                        intaking = true;
                        scoring = false;
                        autoAim = false;
                        ShooterTimer.reset();
                    } else {
                        scoring = true;
                        intaking = false;
                        autoAim = true;
                    }
                };
            case INTAKE3:
                if (!follower.isBusy()) {
                    if (ShooterDone2) {
                        follower.followPath(Path7);
                        IntakeDone2 = true;
                        pathState = PathState.INTAKE3_SHOOT4POSE;
                        break;
                    }
                }
            case INTAKE3_SHOOT4POSE:
                if (!follower.isBusy()) {
                    if (IntakeDone2) {
                        follower.followPath(Path8);
                        if (!follower.isBusy() | follower.getPose().getY() > 106) {
                            pathState = PathState.SHOOT4;
                            ShooterTimer.reset();
                            break;
                        }
                    }
                }
            case SHOOT4:
                if (!follower.isBusy() && IntakeDone2) {
                    if (ShooterTimer.seconds() > 2.5) {
                        pathState = PathState.SHOOT4_LEVER;
                        ShooterDone3 = true;
                        intaking = true;
                        scoring = false;
                        autoAim = false;
                        ShooterTimer.reset();
                    } else {
                        scoring = true;
                        intaking = false;
                        autoAim = true;
                    }
                }
            case SHOOT4_LEVER:
                if (!follower.isBusy()) {
                    if (ShooterDone3) {
                        follower.followPath(Path9);
                        break;
                    }
                }
        }
    }

    public void setPathState(PathState newState) {
        pathState = newState;
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

        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("shooter time", ShooterTimer.seconds());
        telemetry.addData("rpm", shooter.avgRPM);

        double targetRPM = shooter.RPM(30);
        intake.autonIntake(intaking, false, scoring);
        double hoodPos = shooter.hood(26, true);
        double flywheel = pid.calculate(3200, shooter.avgRPM);
        leftFlywheel.setPower(flywheel);
        rightFlywheel.setPower(flywheel);
        hood.setPosition(0);
        boolean isValid = limelight.getLatestResult().isValid();
        double tx = limelight.getLatestResult().getTx();
        leftT.setPosition(0.5);
        rightT.setPosition(0.5);

    }

    @Override
    public void init() {
        pathState = PathState.STARTPOSE_SHOOT1POSE;
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setPose(startPose);
        ShooterTimer = new ElapsedTime();
        intake.init(hardwareMap);
        hood = hardwareMap.get(Servo.class, "hood");
        leftFlywheel = hardwareMap.get(DcMotorEx.class, "1");
        rightFlywheel = hardwareMap.get(DcMotorEx.class, "2");
        leftFlywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightFlywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftFlywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        rightFlywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(2);
        pid = new NeroFlywheelPIDF(0.000005, 0, 0, 0.0002);
        shooter.init(hardwareMap);
        leftT = hardwareMap.get(Servo.class, "leftT");
        rightT = hardwareMap.get(Servo.class, "rightT");
    }
}
