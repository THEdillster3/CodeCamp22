package org.firstinspires.ftc.teamcode.Hardware;

import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.hardwareMap;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.multTelemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Mecanum {
    DcMotor fl;
    DcMotor fr;
    DcMotor bl;
    DcMotor br;
    /**
     * Initializes the robot's necessary subsystems and motors
     */

    public Mecanum(){
        initMecanum();
    }
    public void initMecanum(){
        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");

        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.FORWARD);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    /**
     * Sets the same power to all four motors
     * @param power
     */
    public void setAllPower(double power){

        fl.setPower(power);
        fr.setPower(power);
        bl.setPower(power);
        br.setPower(power);

    }

    /**
     * Sets the power to the motors for driving with a mecanum drivetrain
     * @param power
     */
    public void setDrivePower(double drive, double strafe, double turn, double power){

        double frPower = (drive - strafe - turn) * power;
        double flPower = (drive + strafe + turn) * power;
        double brPower = (drive + strafe - turn) * power;
        double blPower = (drive - strafe + turn) * power;

        fr.setPower(frPower);
        fl.setPower(flPower);
        br.setPower(brPower);
        bl.setPower(blPower);
        /*

                Y O U R   C O D E   H E R E

         */
    }


    /**
     * Translates the robot autonomously a certain distance known as ticks
     * @param ticks
     */
    public void strafe(double ticks){


        /*

                Y O U R   C O D E   H E R E

         */
    }

    /**
     * Rotates the robot autonomously a certain number of degrees with a margin of error
     * @param degrees
     * @param moe
     */
    public void turn(double degrees, double moe){
        /*

                Y O U R   C O D E   H E R E

         */
    }


    /**
     * A mathematical function that optimizes the ramping of power to the motors during autonomous
     * strafes.
     * @param position
     * @param distance
     * @param acceleration
     * @return the coefficient [0, 1] of our power
     */
    public static double powerRamp(double position, double distance, double acceleration){
        /*

                Y O U R   C O D E   H E R E

         */
        return 0;
    }
}
