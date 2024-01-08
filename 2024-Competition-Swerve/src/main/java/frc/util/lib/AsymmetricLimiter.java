// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.util.lib;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;

public class AsymmetricLimiter {
    private double posMagLimit;
    private double negMagLimit;

    private double prevVal;
    private double prevTime;

    /** 
     * Constructs the AsymmetricLimiter class
     * @param posMagLimit Positive rate limit
     * @param negMagLimit Negative rate limit
     */
    public AsymmetricLimiter(double posMagLimit, double  negMagLimit) {
        this.posMagLimit = Math.abs(posMagLimit);
        this.negMagLimit = Math.abs(negMagLimit);
        this.prevVal = 0.0;
        this.prevTime = Timer.getFPGATimestamp();
    }

    /**
     * Calculate method for asymmetric rate limit. This is a dynamic process by a dt of 0.02. Heavily inspired by Team NOMAD's AsymmetricLimiter, with updates due to deprecated methods. Has improved readibility with real calculus terms (dt, dControl).
     * @param input Input to rate limit
     * @return Rate-limited output
     */
    public double calculate(double input) {
        // Current time in seconds
        double currentTime = Timer.getFPGATimestamp();

        // Differential time element (0.02 s)
        double dt = currentTime - prevTime;
        // Differential input element
        double dInput = input - prevVal;

        if (Math.signum(dInput) == -Math.signum(prevVal)) {
            prevVal += MathUtil.clamp(dInput, -negMagLimit * dt, negMagLimit * dt);
        } else {
            prevVal += MathUtil.clamp(dInput, -posMagLimit * dt, posMagLimit * dt);
        }
    
        prevTime = currentTime;
        return prevVal;
    }
}
