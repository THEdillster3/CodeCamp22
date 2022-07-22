package org.firstinspires.ftc.teamcode.Utilities;

import static org.firstinspires.ftc.teamcode.Utilities.PIDWeights.DASH_DERIVATIVE_WEIGHT;
import static org.firstinspires.ftc.teamcode.Utilities.PIDWeights.DASH_INTEGRAL_WEIGHT;
import static org.firstinspires.ftc.teamcode.Utilities.PIDWeights.DASH_PROPORTIONAL_WEIGHT;

public class PID {

    //initializes Proportional and Integral weight variable
    private double proportionalWeight;
    private double integralWeight;
    private double derivativeWeight;

    //initializes the integral sum, the sum of errors
    private double integralSum = 0;

    //initializes the previousError and PreviousTime, which allows the Rate of Change to be calculated
    private double previousError = 0;
    private long previousTime = System.currentTimeMillis();

    //constructor for method, inputs weights and creates a PID object
    public PID(double proportional, double integral, double derivative) {
        this.proportionalWeight = proportional;
        this.integralWeight = integral;
        this.derivativeWeight = derivative;
    }

    //updates error and returns the correction, using weights from Dashboard for tuning.
    public double update(double error, boolean isTuning){

        integralSum += error; //adds the current error to the sum of errors, or the integral

        double deltaTime = (System.currentTimeMillis() - previousTime) / 1000.0; //finds the difference, or delta, in time between updates, in seconds
        double deltaError = error - previousError; //finds the difference, or delta, in time between updates
        double rateOfChange = deltaError/deltaTime;

        previousError = error; //sets previousError for next loop
        previousTime = System.currentTimeMillis(); //sets previousTime for next loop

        double pComponent = error;
        double iComponent = integralSum;
        double dComponent = rateOfChange;

        if (isTuning){
            pComponent *= DASH_PROPORTIONAL_WEIGHT;
            iComponent *= DASH_INTEGRAL_WEIGHT;
            dComponent *= DASH_DERIVATIVE_WEIGHT;
        }
        else {
            pComponent *= proportionalWeight; //sets the pComponent by multiplying the error by the proportionalWeight
            iComponent *= integralWeight; //sets the iComponent by multipling the integral by the integralWeight
            dComponent *= derivativeWeight; //sets the dComponent by multiplying the rate of change by the derivativeWeight
        }


        //returns the correction
        return pComponent + iComponent + dComponent;

    }

}







