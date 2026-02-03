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
    private double targetPosition = 0.49998;
    private boolean upPressed = false;
    private boolean downPressed = false;
    private final float posStep = (1/12);

    @Override
    public void init() {
        LeftTurret = hardwareMap.get(Servo.class, "leftT");
        RightTurret = hardwareMap.get(Servo.class, "rightT");
    }
    @Override
    public void loop() {
        boolean input1 = gamepad1.left_bumper;

        if (gamepad1.dpad_up && !upPressed) {
            targetPosition += posStep;
            upPressed = true;
        }
        if (!gamepad1.dpad_up) {
            upPressed = false;
        }
        if (gamepad1.dpad_down && !downPressed) {
            targetPosition -= posStep;
            downPressed = true;
        }
        if (!gamepad1.dpad_down) {
            downPressed = false;
        }
        if (targetPosition>1){
            targetPosition = targetPosition-1;
        }

        if (targetPosition<0){
            targetPosition = 1+targetPosition;
        }

        double actualPosition = MathFunctions.clamp(targetPosition, 0.08333, 0.91667);

        LeftTurret.setPosition(actualPosition);
        LeftTurret.setPosition(actualPosition);

        telemetry.addData("Calculated Servo Position", targetPosition);
        telemetry.addData("Actual Servo Position", actualPosition);
        telemetry.addData("Turret Angle", ((360*(actualPosition))-180));
        telemetry.update();
    }
}
