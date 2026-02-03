package org.firstinspires.ftc.teamcode.Nero.PID;

public class NeroFlywheelPIDF {



        private double kP, kI, kD, kF;
        private double integralSum = 0.0;
        private double lastError = 0.0;

        public NeroFlywheelPIDF(double kP, double kI, double kD, double kF) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            this.kF = kF;
        }

        public double calculate(double targetVelocity, double currentVelocity) {
            double dt = 0.2;

            double error = targetVelocity - currentVelocity;

            integralSum += error * dt;

            double derivative = (error - lastError) / dt;
            lastError = error;

            return (kP * error)
                    + (kI * integralSum)
                    + (kD * derivative)
                    + (kF * targetVelocity);
        }

        public void reset() {
            integralSum = 0.0;
            lastError = 0.0;
        }
    }
