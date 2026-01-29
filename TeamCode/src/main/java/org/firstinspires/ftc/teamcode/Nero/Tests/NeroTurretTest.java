package org.firstinspires.ftc.teamcode.Nero.Tests;


import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.PID.FlywheelPID;

@TeleOp
public class NeroTurretTest extends OpMode {
    public Servo LeftTurret, RightTurret;
    private double targetPostion = .5;
    private boolean upPressed = false;
    private boolean downPressed = false;
    private final double posStep = .05;

    @Override
    public void init() {
        LeftTurret = hardwareMap.get(Servo.class, "leftT");
        RightTurret = hardwareMap.get(Servo.class, "rightT");
    }
    @Override
    public void loop() {
        boolean input1 = gamepad1.left_bumper;

        if (gamepad1.dpad_up && !upPressed) {
            targetPostion += posStep;
            upPressed = true;
        }
        if (!gamepad1.dpad_up) {
            upPressed = false;
        }
        if (gamepad1.dpad_down && !downPressed) {
            targetPostion -= posStep;
            downPressed = true;
        }
        if (!gamepad1.dpad_down) {
            downPressed = false;
        }
        if (targetPostion>1){
            targetPostion = targetPostion-1;
        }

        if (targetPostion<0){
            targetPostion = 1+targetPostion;
        }

        LeftTurret.setPosition(targetPostion);
        LeftTurret.setPosition(targetPostion);

        telemetry.addData("Turret Servo Postion", targetPostion);
        telemetry.addData("Turret Angle", ((360*(targetPostion))-180));
        telemetry.update();
    }
}
