package org.firstinspires.ftc.teamcode.Nero.PID;

import com.pedropathing.math.MathFunctions;

public class NeroFlywheelPID {

    private double kP, kI, kD, kF;
    private double integral = 0;
    private double lastError = 0;
    private double previousPower = 0.0;

    public NeroFlywheelPID(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }

    public double calculate(double targetRPM, double currentRPM, double dt) {
        double error = targetRPM - currentRPM;

        integral += error * dt;
        double derivative = (error - lastError) / dt;
        lastError = error;

        return kP * error + kI * integral + kD * derivative;
    }
    public double getPower(double targetRPM, double avgRPM, boolean flywheel){
        double power;
        double pidOut = calculate(targetRPM, avgRPM, 0.02);
        if (!flywheel) {
            power = 0;
        } else {
            double kFAdjusted = kF * Math.max(0, (targetRPM - avgRPM) / targetRPM);
            double targetPower = kFAdjusted + pidOut;
            double rampRate = 0.01;
            power = previousPower + Math.signum(targetPower - previousPower) * rampRate;
            power = MathFunctions.clamp(power,0,1);
            previousPower = power;
        }
        return power;
    }

    public void reset() {
        integral = 0;
        lastError = 0;
    }
}
