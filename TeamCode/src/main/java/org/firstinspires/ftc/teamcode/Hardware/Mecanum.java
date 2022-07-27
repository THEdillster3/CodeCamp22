package org.firstinspires.ftc.teamcode.Hardware;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.hardwareMap;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.multTelemetry;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware.Sensors.IMU;
import org.firstinspires.ftc.teamcode.Utilities.MathUtils;
import org.firstinspires.ftc.teamcode.Utilities.PID;

public class Mecanum {
    DcMotor fl;
    DcMotor fr;
    DcMotor bl;
    DcMotor br;
    private PID rotationalPID;
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

        rotationalPID = new PID(.065,0,.00001);

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


    }
    public void resetMotors(){
        fr.setMode(STOP_AND_RESET_ENCODER);
        fl.setMode(STOP_AND_RESET_ENCODER);
        br.setMode(STOP_AND_RESET_ENCODER);
        bl.setMode(STOP_AND_RESET_ENCODER);

        fr.setMode(RUN_WITHOUT_ENCODER);
        fl.setMode(RUN_WITHOUT_ENCODER);
        br.setMode(RUN_WITHOUT_ENCODER);
        bl.setMode(RUN_WITHOUT_ENCODER);
    }
    public double getPosition(){
        double frPosition = fr.getCurrentPosition();
        double flPosition = fl.getCurrentPosition();
        double brPosition = br.getCurrentPosition();
        double blPosition = bl.getCurrentPosition();
        double avgPosition =(frPosition + flPosition + brPosition + blPosition) / 4.0;
        return avgPosition;
    }




    /**
     * Translates the robot autonomously a certain distance known as ticks
     * @param ticks
     */
    public void strafe(double ticks){
        resetMotors();
        double current_distance = 0.0;
        double power = 0.5;
        if (ticks < 0){
            power = -power;
        }
        while (abs(current_distance) < abs(ticks)){
            current_distance = getPosition();

            setAllPower(power);
            multTelemetry.addData("current distance", current_distance);
            multTelemetry.addData("Target Distance", ticks);
            multTelemetry.update();


        }
        setAllPower(0.0);


        /*

                Y O U R   C O D E   H E R E

         */
    }

    /**
     * Rotates the robot autonomously a certain number of degrees with a margin of error
     * @param degrees
     * @param gyro
     */
    public void turn(double degrees, IMU gyro){
        ElapsedTime timer = new ElapsedTime();
         double closestAngle = MathUtils.closestAngle(degrees, gyro.getAngle());
         timer.reset();
        while(timer.seconds() < 2){
            double turnValue = rotationalPID.update(closestAngle - gyro.getAngle(), false);
            setDrivePower(0.0,0.0, -turnValue, 0.5);
            multTelemetry.addData("Closest Angle ", degrees);
            multTelemetry.addData("Current Angle ", gyro.getAngle());
            multTelemetry.update();
        }
        setAllPower(0.0);
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
