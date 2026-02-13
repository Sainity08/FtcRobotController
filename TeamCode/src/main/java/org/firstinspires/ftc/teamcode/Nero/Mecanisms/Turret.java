package org.firstinspires.ftc.teamcode.Nero.Mecanisms;

import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
//import com.seattlesolvers.solverslib.hardware.AbsoluteAnalogEncoder;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


public class Turret {

    // Goals
//    public double blueGoalX = -66;
//    public double blueGoalY = 66;
//    public double redGoalX  = 66;
//    public double redGoalY  = 66;
    public double blueGoalX = 6;
    public double blueGoalY = 138;
    public double redGoalX  = 138;
    public double redGoalY  = 138;
    public double angle;
    public double omega;

    public double[] goal;

//    // Goals
//    public double SblueGoalX = -66;
//    public double SblueGoalY = 66;
//    public double SredGoalX  = 66;
//    public double SredGoalY  = 66;


    public boolean isRed = true;
    private double lastAngularHeading = 0.0;

    // Servos
//    public CRServoGroup turretServos;
    public Servo turretServoFront;
    public Servo turretServoBack;

    public AnalogInput turretAnalog;
    private ElapsedTime angularVelocityTime = new ElapsedTime();


//    public AbsoluteAnalogEncoder turretEncoder;




    // Turret limits
    private static final double MAX_ANGLE = 150.0;

    public double turretpos = .5;

    public double turretAngle = 0.0;

    public double analogangle = 0;
    public boolean shooterActivated = true;

    public double fieldAngle;

    public double ballFlightTime = .7;

    public ElapsedTime angularVeloTime = new ElapsedTime();

    private double Oldtime = 0;


    public void init(HardwareMap hwMap) {
//        turretServos = new CRServoGroup(
//                new CRServoEx(hwMap, "ftservo")
//                        .setCachingTolerance(0.01)
//                        .setRunMode(CRServoEx.RunMode.RawPower),
//                new CRServoEx(hwMap, "btservo")
//                        .setCachingTolerance(0.01)
//                        .setRunMode(CRServoEx.RunMode.RawPower));
//        turretEncoder = new AbsoluteAnalogEncoder(hwMap, "banalog")
//                .zero(0)
//                .setReversed(true);
        turretServoFront = hwMap.get(Servo.class,"leftT");
        turretServoBack = hwMap.get(Servo.class,"rightT");



    }

    // Axon configuration
    private static final double ANALOG_MAX_VOLTAGE = 3.3;
    private static final double SERVO_RANGE_DEG = 300.0; // change if 300°

    private static final double SERVO_NEUTRAL = 0.5;
    private static final double SERVO_MIN = 0.3;
    private static final double SERVO_MAX = 0.7;

    // Encoder unwrap state
    private double lastAngle1 = 0, totalAngle1 = 0;
    private double lastAngle2 = 0, totalAngle2 = 0;

    // FF
    public  double TURRET_FF_GAIN = .001;

    public double turretFeedForwardServo = 0;



    private double angleToServo(double angle360) {
        // Map 0–360° → 0–1
        return angle360 / 360;
    }

    private double calculateTurretAngle(
            double robotX,
            double robotY,
            double robotHeading,
            double goalX,
            double goalY
    ) {
        double dx = goalX - robotX;
        double dy = goalY - robotY;

        // Field angle to target
        fieldAngle = Math.toDegrees(Math.atan2(dy, dx));

        angle = fieldAngle - robotHeading; //210

        if (angle < 0) angle += 360;
        if (angle > 360) angle -= 360;                        // [0,360)
// [0,360)

        return angleToServo(angle);
    }

    public void calcflightTime(double distance){
        ballFlightTime = distance * .01;        // need to figure out regression line
    }

//    private double getRawAngle(AnalogInput analog) {
//        return (analog.getVoltage() / ANALOG_MAX_VOLTAGE) * SERVO_RANGE_DEG;
//    }



    private boolean atTarget(double target, double current, double tolerance) {
        return Math.abs(target - current) <= tolerance;
    }


    private double clampTurretTarget(double target) {
        return Math.max(0.0813, Math.min(.917, target));
    }

    public double turretpositionX(double robotX, double robotY, double robotHeading) {
        double offset = 3.1; // inches from back of robot
        return robotX - offset * Math.cos(robotHeading);
    }

    public double turretpositionY(double robotX, double robotY, double robotHeading) {
        double offset = 0; // inches from back of robot
        return robotY - offset * Math.sin(robotHeading);
    }

    public double[] shootOnTheMove(
            double turretX,
            double turretY,
            double robotVectorX,
            double robotVectorY,
            boolean isRed,
            boolean SOTM
    ) {
        // Select correct goal
        double goalX = isRed ? redGoalX : blueGoalX;
        double goalY = isRed ? redGoalY : blueGoalY;

        // Distance from turret to goal
        double dx = goalX - turretX;
        double dy = goalY - turretY;
        double distance = Math.hypot(dx, dy);

        // Lead compensation (move goal opposite robot motion)
        double leadX = robotVectorX * ballFlightTime;
        double leadY = robotVectorY * ballFlightTime;

        double compensatedGoalX = goalX - leadX;
        double compensatedGoalY = goalY - leadY;
        if(SOTM) {
            return new double[]{compensatedGoalX, compensatedGoalY};
        } else
            return new double[]{goalX, goalY};

    }

    public double AngularVelocity(double robotHeading) {
        double currentHeading = AngleUnit.RADIANS.toDegrees(robotHeading);
        double deltaTime = angularVeloTime.seconds();

        double deltaHeading = AngleUnit.normalizeDegrees(currentHeading - lastAngularHeading);


        lastAngularHeading = currentHeading;
        angularVeloTime.reset();


        return deltaHeading/deltaTime;
    }



    public void update(
            double robotX,
            double robotY,
            double robotHeading,
            double goalX,
            double goalY,
            double robotVectX,
            double robotVectY,
            boolean isRed,
            boolean SOTM
    ) {
        goal = shootOnTheMove(
                turretpositionX(robotX, robotY, robotHeading),
                turretpositionY(robotX, robotY, robotHeading),
                robotVectX,
                robotVectY,
                isRed,
                SOTM
        );
        omega = AngularVelocity(robotHeading);
        turretFeedForwardServo =  (-omega * TURRET_FF_GAIN) / 360.0;
        turretAngle = calculateTurretAngle(robotX, robotY, robotHeading, goal[0], goal[1])-turretFeedForwardServo;
//        FFturret(robotHeading);
        turretAngle = MathFunctions.clamp(turretAngle,0.0813,.917);



//        analogangle = (((turretAnalog.getVoltage() / 3.3)* 450)-45);

//        analogangle = MathFunctions.clamp(analogangle,.25,.75);




        if (shooterActivated) {

            turretServoFront.setPosition(clampTurretTarget(turretAngle));
            turretServoBack.setPosition(clampTurretTarget(turretAngle));
//
//            turretServoFront.setPosition(.5);
//            turretServoBack.setPosition(.5);
//            turretServos.set(.1);
        } else {
            turretServoFront.setPosition(0.5);
            turretServoBack.setPosition(.5);
        }
    }
}