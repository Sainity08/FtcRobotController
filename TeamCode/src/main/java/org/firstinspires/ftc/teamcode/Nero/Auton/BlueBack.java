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
import org.firstinspires.ftc.teamcode.Nero.Mecanisms.Turret;
import org.firstinspires.ftc.teamcode.Nero.Mecanisms.file;
import org.firstinspires.ftc.teamcode.Nero.PID.NeroFlywheelPIDF;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Back Blue")
public class BlueBack extends OpMode {
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
    private final Pose startPose = new Pose(55, 7.75, Math.toRadians(270));
    private Follower follower;
    double shootAngle = 315;
    file fileManager = new file();

    public void buildPaths() {
        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(55, 7.75),

                                new Pose(56.000, 14.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(270))

                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(56.000, 14.000),
                                new Pose(59.277, 38.436),
                                new Pose(12.962, 35.725)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(12.962, 35.725),

                                new Pose(56.000, 14.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path4 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(56.000, 14.000),
                                new Pose(6.732, 57.811),
                                new Pose(8.417, 10)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(270))

                .build();

        Path5 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(8.417, 10),

                                new Pose(56.000, 14.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(270))

                .build();

        Path6 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(56.000, 14.000),

                                new Pose(7.653, 13.623)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        Path7 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(7.653, 13.623),

                                new Pose(56.000, 14.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path8 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(56.000, 14.000),
                                new Pose(54.140, 34.100),
                                new Pose(7.162, 36.072)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path9 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(7.162, 36.072),

                                new Pose(56.000, 14.000)
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
        INTAKE2_SHOOT3POSE,
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
    boolean intakedone = false;
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
                    pathState = PathState.SHOOT1;
                    ShooterTimer.reset();
                }
                break;
            case SHOOT1:
                if (!follower.isBusy()) {
                    if (ShooterTimer.seconds() > 5.5) {
                        pathState = PathState.INTAKE1;
                        ShooterDone = true;
                        intaking = true;
                        scoring = false;
                        autoAim = false;
                    } else {
                        if(shooter.avgRPM > 4800) {
                            scoring = true;
                            intaking = false;
                        }
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
                        if (!follower.isBusy() | follower.getPose().getX() > 56) {
                            pathState = PathState.SHOOT2;
                            ShooterTimer.reset();
                            IntakeDone = true;
                            break;
                        }
                    }
                }

            case SHOOT2:
                if (!follower.isBusy() && IntakeDone) {
                    if (ShooterTimer.seconds() > 3){
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
                        pathState = PathState.INTAKE2_SHOOT3POSE;
                        intakedone = true;
                        break;
                    }
                }
            case INTAKE2_SHOOT3POSE:
                if(!follower.isBusy() && intakedone) {
                    if (!follower.isBusy() | follower.getPose().getY() < 10) {
                        follower.followPath(Path5, true);
                        if (!follower.isBusy() | follower.getPose().getX() > 56) {
                            pathState = PathState.SHOOT3;
                            done = true;
                            ShooterTimer.reset();
                            break;
                        }
                    }
                }
            case SHOOT3:
                if (!follower.isBusy() && done) {
                    double t = ShooterTimer.seconds();
                    if (t > 3) {
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
                        follower.followPath(Path6);
                        IntakeDone2 = true;
                        pathState = PathState.INTAKE3_SHOOT4POSE;
                        break;
                    }
                }
            case INTAKE3_SHOOT4POSE:
                if (!follower.isBusy()) {
                    if (IntakeDone2) {
                        follower.followPath(Path7);
                        if (!follower.isBusy() | follower.getPose().getX() > 56) {
                            pathState = PathState.SHOOT4;
                            ShooterTimer.reset();
                            break;
                        }
                    }
                }
            case SHOOT4:
                if (!follower.isBusy() && IntakeDone2) {
                    if (ShooterTimer.seconds() > 3) {
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
                        follower.followPath(Path8);
                        IntakeDone3 = true;
                        pathState = PathState.INTAKE4_SHOOT5;
                        break;
                    }
                }
            case INTAKE4_SHOOT5:
                if (!follower.isBusy()) {
                    if (IntakeDone3) {
                        follower.followPath(Path8);
                        if (!follower.isBusy() | follower.getPose().getX() > 56) {
                            pathState = PathState.SHOOT4;
                            ShooterTimer.reset();
                            break;
                        }
                    }
                }
            case SHOOT5:
                if (!follower.isBusy() && IntakeDone2) {
                    if (ShooterTimer.seconds() > 3) {
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
        telemetry.addData("lever time", LeverTimer.seconds());
        telemetry.addData("rpm", shooter.avgRPM);
        turret.update(turret.turretpositionX(follower.getPose().getX(), follower.getPose().getY(),follower.getHeading()),turret.turretpositionY(follower.getPose().getX(), follower.getPose().getY(),follower.getPose().getHeading()),Math.toDegrees(follower.getHeading()),turret.blueGoalX,turret.blueGoalY,follower.getVelocity().getXComponent(),follower.getVelocity().getYComponent(), false, false , false);

        double distance = Shooter.distance2D(turret.turretpositionX(follower.getPose().getX(), follower.getPose().getY(),follower.getPose().getHeading()),turret.turretpositionY(follower.getPose().getX(), follower.getPose().getY(),follower.getPose().getHeading()), turret.blueGoalX,turret.blueGoalY);
        shooter.newRPM(distance - 20);
        shooter.newHood(distance);
        shooter.ShooterAuto();
        intake.backAutonIntake(intaking, false, scoring);
        follower.update();
        turret.offset = 0.0175;
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
