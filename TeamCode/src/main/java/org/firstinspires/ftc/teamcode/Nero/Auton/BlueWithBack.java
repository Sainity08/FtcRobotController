package org.firstinspires.ftc.teamcode.Nero.Auton;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
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
import org.java_websocket.SocketChannelIOHelper;

@Autonomous
public class BlueWithBack extends OpMode {
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
    private final Pose startPose = new Pose(34.000, 136.000, Math.toRadians(270));
    private Follower follower;
    double shootAngle = 310;

    public void buildPaths() {
        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(34.000, 136.000),

                                new Pose(37.000, 106.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(shootAngle))

                .build();


        Path2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(37.000, 106.000),
                                new Pose(102.022, 41.181),
                                new Pose(3.315, 65.994),
                                new Pose(16.470, 56.242),
                                new Pose(8.103, 60.177)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();


        Path3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(8.103, 60.177),
                                new Pose(32.906, 68.865),
                                new Pose(18.321, 69.867)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();

        Path4 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(15.000, 70.000),
                                new Pose(68.838, 71.122),
                                new Pose(37.000, 106.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(shootAngle + 3))

                .build();

        Path5 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(37.000, 106.000),
                                new Pose(87.849, 78.437),
                                new Pose(14.500, 84.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path6 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(14.500, 84.000),
                                new Pose(71.580, 81.066),
                                new Pose(37.000, 106.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(shootAngle + 5))

                .build();

        Path7 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(37.000, 106.000),
                                new Pose(58.004, 67.882),
                                new Pose(26, 70.037)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path8 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(15.089, 70.037),
                                new Pose(23.710, 59.240),
                                new Pose(10.539, 59.852)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(160))

                .build();
    }

    public enum PathState {
        STARTPOSE_SHOOT1POSE,
        SHOOT1,
        INTAKE1,
        INTAKE1_LEVER,
        LEVER_SHOOT2POSE,
        SHOOT2,
        INTAKE2,
        INTAKE2_SHOOT3POSE,
        SHOOT3,
        LEVER,
        INTAKE4,
    }

    boolean ShooterDone = false;
    boolean IntakeDone = false;
    boolean ShooterDone1 = false;
    boolean IntakeDone1 = false;
    boolean ShooterDone2 = false;
    boolean IntakeDone2 = false;
    boolean autoAim;
    PathState pathState;
    double turn = 0;

    public void statePathUpdate() {
        switch (pathState) {
            case STARTPOSE_SHOOT1POSE:
                if (!follower.isBusy()) {
                    intaking = true;
                    follower.followPath(Path1, true);
                    if ((((follower.getPose().getX() > 37) && (follower.getPose().getY() < 106)))) {
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
                        follower.breakFollowing();
                        autoAim = true;
                    }
                }
            case INTAKE1:
                if (!follower.isBusy()) {
                    if (ShooterDone) {
                        follower.followPath(Path2, true);
                        pathState = PathState.INTAKE1_LEVER;
                        break;
                    }
                }
            case INTAKE1_LEVER:
                if (!follower.isBusy()) {
                    if (ShooterDone) {
                        follower.followPath(Path3);
                        if (!follower.isBusy() | (follower.getPose().getX() < 18.2)) {
                            pathState = PathState.LEVER_SHOOT2POSE;
                            IntakeDone = true;
                            break;
                        }
                    }
                }
            case LEVER_SHOOT2POSE:
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
                        pathState = PathState.INTAKE2;
                        ShooterDone1 = true;
                        intaking = true;
                        scoring = false;
                        autoAim = false;
                    } else {
                        scoring = true;
                        intaking = false;
                        follower.breakFollowing();
                        autoAim = true;
                    }
                }
            case INTAKE2:
                if (!follower.isBusy()) {
                    if (ShooterDone1) {
                        follower.followPath(Path5);
                        pathState = PathState.INTAKE2_SHOOT3POSE;
                        break;
                    }
                }
            case INTAKE2_SHOOT3POSE:
                if (!follower.isBusy()) {
                    if (ShooterDone1) {
                        follower.followPath(Path6, true);
                        if (!follower.isBusy() | (follower.getPose().getX()) < 24) {
                            pathState = PathState.SHOOT3;
                            ShooterTimer.reset();
                            IntakeDone1 = true;
                            break;
                        }
                    }
                }
            case SHOOT3:
                if (!follower.isBusy()) {
                    if (ShooterTimer.seconds() > 5) {
                        pathState = PathState.LEVER;
                        ShooterDone2 = true;
                        intaking = true;
                        scoring = false;
                        autoAim = false;
                        ShooterTimer.reset();
                    } else {
                        scoring = true;
                        intaking = false;
                        follower.breakFollowing();
                        autoAim = true;
                    }
                }
            case LEVER:
                if (!follower.isBusy()) {
                    if (ShooterDone2) {
                        follower.followPath(Path7, false);
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
