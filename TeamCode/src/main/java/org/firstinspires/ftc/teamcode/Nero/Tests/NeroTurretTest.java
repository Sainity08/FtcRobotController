package org.firstinspires.ftc.teamcode.Nero.Tests;


import com.pedropathing.math.MathFunctions;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.PID.FlywheelPID;

@TeleOp
public class NeroTurretTest extends OpMode {
    public Servo LeftTurret, RightTurret;
    private double targetPosition = 0.49998;
    private boolean upPressed = false;
    public GoBildaPinpointDriver imu;
    private boolean downPressed = false;
    private final float posStep = (1/12);

    @Override
    public void init() {
        LeftTurret = hardwareMap.get(Servo.class, "leftT");
        RightTurret = hardwareMap.get(Servo.class, "rightT");
        imu.setPosition(new Pose2D(DistanceUnit.INCH, 72,72, AngleUnit.RADIANS, 0));
    }

    public double position_to_heading(double position) {
        return (2 * Math.PI * position) - Math.PI;
    }

    public double heading_to_position(double heading) {
        return (heading - Math.PI) / (2 * Math.PI);
    }


    @Override
    public void loop() {
        boolean input1 = gamepad1.left_bumper;

//        if (gamepad1.dpad_up && !upPressed) {
//            targetPosition += posStep;
//            upPressed = true;
//        }
//        if (!gamepad1.dpad_up) {
//            upPressed = false;
//        }
//        if (gamepad1.dpad_down && !downPressed) {
//            targetPosition -= posStep;
//            downPressed = true;
//        }
//        if (!gamepad1.dpad_down) {
//            downPressed = false;
//        }
        if (targetPosition>1){
            targetPosition = targetPosition-1;
        }

        if (targetPosition<0){
            targetPosition = 1+targetPosition;
        }

        double targetX = 144;
        double targetY = 144;

        double botX = imu.getEncoderX();
        double botY = imu.getEncoderY();
        double botHeading = imu.getHeading(AngleUnit.RADIANS);

        double turretX = botX + 3.557842126 * Math.cos(botHeading);
        double turretY = botY + 3.557842126 * Math.sin(botHeading);

        double targetGlobalHeading = Math.atan2(targetY - turretY, targetX - turretX);

        double turretRelativeHeading = targetGlobalHeading - botHeading;

        turretRelativeHeading = MathFunctions.clamp(turretRelativeHeading, -Math.toRadians(150), Math.toRadians(150));

        LeftTurret.setPosition(heading_to_position(turretRelativeHeading));
        RightTurret.setPosition(heading_to_position(turretRelativeHeading));




//        double turretHeading = position_to_heading((LeftTurret.getPosition() + RightTurret.getPosition())/2);
//        double targetTurretHeading = Math.atan2((targetY-turretY),(targetX-turretX));

//
//        double globalTurretHeading = (botHeading + targetTurretHeading);
//        globalTurretHeading = MathFunctions.clamp(globalTurretHeading, -150,150);
//        double actualPosition = ((globalTurretHeading + 180)/360);
//        actualPosition = MathFunctions.clamp(targetPosition, 0.08333, 0.91667);


        telemetry.addData("Calculated Servo Position", targetPosition);
//        telemetry.addData("Actual Turret Heading", turretHeading);
//        telemetry.addData("Turret Angle", globalTurretHeading);
        telemetry.update();
    }
}
