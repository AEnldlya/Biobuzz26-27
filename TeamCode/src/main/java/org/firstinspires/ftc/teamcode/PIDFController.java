package org.firstinspires.ftc.teamcode;

/**
 * output = kP*error + kI*integral(error) + kD*errorRate + kV*feedforward + kS*sign(output)
 *
 * errorRate is passed in (for a position loop: target velocity - measured velocity), so there is
 * no noisy differentiation of the position and no derivative kick when the target jumps.
 */
public class PIDFController {
    public double kP, kI, kD, kV, kS;
    /** the integral only accumulates while |error| is inside this band; outside it is cleared */
    public double integralZone = Double.POSITIVE_INFINITY;
    /** cap on |kI * integral| */
    public double maxIntegralOutput = 1.0;

    private double integral = 0.0;

    public PIDFController(double kP, double kI, double kD, double kV, double kS) {
        setGains(kP, kI, kD, kV, kS);
    }

    public void setGains(double kP, double kI, double kD, double kV, double kS) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kV = kV;
        this.kS = kS;
    }

    public double calculate(double error, double errorRate, double feedforward, double dt, boolean applyStatic) {
        if (kI != 0.0 && Math.abs(error) <= integralZone) {
            integral += error * dt;
            double limit = maxIntegralOutput / Math.abs(kI);
            integral = Math.max(-limit, Math.min(limit, integral));
        } else {
            integral = 0.0;
        }

        double output = kP * error + kI * integral + kD * errorRate + kV * feedforward;
        if (applyStatic && output != 0.0) {
            output += Math.copySign(kS, output);
        }
        return output;
    }

    public void reset() {
        integral = 0.0;
    }
}
