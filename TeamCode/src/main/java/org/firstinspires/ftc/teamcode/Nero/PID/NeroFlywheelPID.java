package org.firstinspires.ftc.teamcode.Nero.PID;

import com.pedropathing.math.MathFunctions;

public class NeroFlywheelPID {

    private double kP, kI, kD, kF;
    private double integral = 0;
    private double lastError = 0;
    private double previousPower = (5000/6000);

    public NeroFlywheelPID(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public double calculate(double targetRPM, double currentRPM, double dt) {
        double error = targetRPM - currentRPM;

        integral += error * dt;
        double derivative = (error - lastError) / dt;
        lastError = error;

        return kP * error + kI * integral + kD * derivative;
    }

    public void reset() {
        integral = 0;
        lastError = 0;
    }
}
