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
import org.firstinspires.ftc.teamcode.Nero.Mecanisms.SavePose;
import org.firstinspires.ftc.teamcode.Nero.Mecanisms.Shooter;
import org.firstinspires.ftc.teamcode.Nero.Mecanisms.Turret;
import org.firstinspires.ftc.teamcode.Nero.PID.NeroFlywheelPIDF;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Red 15 Ball")
public class Red15 extends OpMode {
    private double distance = 31.1126983722;

    private Limelight3A limelight;
    Intake intake = new Intake();
    Drivetrain drive = new Drivetrain();
    Turret turret = new Turret();
    NeroFlywheelPIDF pid;
    public Servo hood, leftT, rightT;
    boolean intaking = true;
    boolean scoring = false;
    private ElapsedTime ShooterTimer = new ElapsedTime();
    private ElapsedTime LeverTimer = new ElapsedTime();
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
    public PathChain Path10;
    private final Pose startPose = new Pose(34, 136.000, Math.toRadians(270));
    private Follower follower;
    double shootAngle = 315;
    SavePose fileManager = new SavePose();

    public void buildPaths() {
        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(110.000, 136.000),

                                new Pose(89.494, 89.117)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))

                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(89.494, 89.117),
                                new Pose(94.370, 81.506),
                                new Pose(125.253, 83.306)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(125.253, 83.306),

                                new Pose(89.604, 89.223)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        Path4 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(89.604, 89.223),
                                new Pose(78.798, 54.794),
                                new Pose(125.064, 59.121)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        Path5 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(125.064, 59.121),
                                new Pose(111.438, 67.587),
                                new Pose(127.970, 70.913)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(-90))

                .build();

        Path6 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(127.970, 70.913),

                                new Pose(89.445, 89.317)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(-90))

                .build();

        Path7 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(89.445, 89.317),
                                new Pose(78.715, 29.358),
                                new Pose(125.525, 35.302)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        Path8 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(125.525, 35.302),

                                new Pose(89.049, 89.396)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        Path9 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(89.049, 89.396),
                                new Pose(131.800, 79.506),
                                new Pose(136.664, 9.374)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(-90))

                .build();

        Path10 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(136.664, 9.374),

                                new Pose(81.475, 103.985)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(-90))

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
        INTAKE4,
        INTAKE4_SHOOT5,
        SHOOT5

    }

    boolean ShooterDone = false;
    boolean IntakeDone = false;
    boolean ShooterDone1 = false;
    boolean IntakeDone1 = false;
    boolean ShooterDone2 = false;
    boolean ShooterDone3 = false;
    boolean IntakeDone2 = false;
    boolean IntakeDone3 = false;
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
                    if (ShooterTimer.seconds() > 1) {
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
                    if (ShooterTimer.seconds() > 2){
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
                        LeverTimer.reset();
                        if (!follower.isBusy() | (follower.getPose().getX()) < 18) {
                            pathState = PathState.PRESSLEVER;
                            ShooterTimer.reset();
                            IntakeDone1 = true;
                            break;
                        }
                    }
                }
            case PRESSLEVER:
                if (!follower.isBusy() && ShooterDone1) {
                    double t = LeverTimer.seconds();
                    if (t < 1.5) {
                        scoring = false;
                        follower.breakFollowing();
                        break;
                    }
                    if (t >= 1.5) {
                        pathState = PathState.LEVER_SHOOT3POSE;
                        scoring = false;
                        break;
                    }
                }
                break;

            case LEVER_SHOOT3POSE:
                if (!follower.isBusy()) {
                    if (IntakeDone1) {
                        follower.followPath(Path6, true);
                        if (!follower.isBusy() | follower.getPose().getY() > 89) {
                            pathState = PathState.SHOOT3;
                            done = true;
                            ShooterTimer.reset();
                            break;
                        }
                    }
                }
            case SHOOT3:
                if (!follower.isBusy() && IntakeDone1 && done) {
                    if (ShooterTimer.seconds() > 2) {
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
                        if (!follower.isBusy() | follower.getPose().getY() > 89) {
                            pathState = PathState.SHOOT4;
                            ShooterTimer.reset();
                            break;
                        }
                    }
                }
            case SHOOT4:
                if (!follower.isBusy() && IntakeDone2) {
                    if (ShooterTimer.seconds() > 2) {
                        pathState = PathState.INTAKE4;
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
            case INTAKE4:
                if (!follower.isBusy()) {
                    if (ShooterDone3) {
                        follower.followPath(Path9);
                        IntakeDone3 = true;
                        pathState = PathState.INTAKE4_SHOOT5;
                        break;
                    }
                }
            case INTAKE4_SHOOT5:
                if (!follower.isBusy()) {
                    if (IntakeDone3) {
                        follower.followPath(Path10);
                        intaking = false;
                        if (!follower.isBusy() | follower.getPose().getY() > 103.5) {
                            pathState = PathState.SHOOT5;
                            intaking = true;
                            ShooterTimer.reset();
                            break;
                        }
                    }
                }
            case SHOOT5:
                if (!follower.isBusy() && IntakeDone2) {
                    if (ShooterTimer.seconds() > 2) {
                        ShooterDone3 = true;
                        scoring = false;
                        autoAim = false;
                        ShooterTimer.reset();

                    } else {
                        scoring = true;
                        intaking = false;
                        autoAim = true;
                    }
                }
        }
    }

    public void setPathState(PathState newState) {
        pathState = newState;
    }

    @Override
    public void start() {
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
        telemetry.addData("nigger sits at lever for", LeverTimer.seconds());
        telemetry.addData("rpm", shooter.avgRPM);
        turret.update(turret.turretpositionX(follower.getPose().getX(), follower.getPose().getY(),follower.getHeading()),turret.turretpositionY(follower.getPose().getX(), follower.getPose().getY(),follower.getPose().getHeading()),Math.toDegrees(follower.getHeading()),turret.redGoalX,turret.redGoalY,follower.getVelocity().getXComponent(),follower.getVelocity().getYComponent(), true, false);

        double distance = Shooter.distance2D(turret.turretpositionX(follower.getPose().getX(), follower.getPose().getY(),follower.getPose().getHeading()),turret.turretpositionY(follower.getPose().getX(), follower.getPose().getY(),follower.getPose().getHeading()), turret.redGoalX,turret.redGoalY);
        shooter.RPM(distance + 5);
        shooter.hood(distance + 5);
        shooter.ShooterAuto();
        intake.autonIntake(intaking, false, scoring);
        follower.update();
    }

    @Override
    public void init() {
        pathState = PathState.STARTPOSE_SHOOT1POSE;
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setPose(startPose);
        ShooterTimer = new ElapsedTime();
        LeverTimer = new ElapsedTime();
        intake.init(hardwareMap);
        shooter.init(hardwareMap);
        turret.init(hardwareMap);
        fileManager.init();
    }
    @Override
    public void stop() {
        fileManager.FileWrite(follower.getPose().getX(),follower.getPose().getY(),follower.getHeading());
    }
}
