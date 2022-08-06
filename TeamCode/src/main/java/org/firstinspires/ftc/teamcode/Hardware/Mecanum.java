package org.firstinspires.ftc.teamcode.Hardware;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static org.firstinspires.ftc.teamcode.Utilities.MathUtils.angleMode.DEGREES;
import static org.firstinspires.ftc.teamcode.Utilities.MathUtils.closestAngle;
import static org.firstinspires.ftc.teamcode.Utilities.MathUtils.cos;
import static org.firstinspires.ftc.teamcode.Utilities.MathUtils.shift;
import static org.firstinspires.ftc.teamcode.Utilities.MathUtils.sin;
import static org.firstinspires.ftc.teamcode.Utilities.MathUtils.unShift;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.hardwareMap;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.multTelemetry;

import static java.lang.Math.abs;
import static java.lang.Math.addExact;
import static java.lang.Math.hypot;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware.Sensors.IMU;
import org.firstinspires.ftc.teamcode.Utilities.Odometry;
import org.firstinspires.ftc.teamcode.Utilities.MathUtils;
import org.firstinspires.ftc.teamcode.Utilities.PID;
import org.opencv.core.Point;

public class Mecanum {
    DcMotor drivefl;
    DcMotor drivefr;
    DcMotor drivebl;
    DcMotor drivebr;
    private PID rotationalPID;
    public IMU robotAngle;

    public Odometry odometry;

    /**
     * Initializes the robot's necessary subsystems and motors
     */

    public Mecanum(){
        initMecanum();
    }
    public void initMecanum(){
        drivefl = hardwareMap.get(DcMotor.class, "drivefl");
        drivefr = hardwareMap.get(DcMotor.class, "drivefr");
        drivebl = hardwareMap.get(DcMotor.class, "drivebl");
        drivebr = hardwareMap.get(DcMotor.class, "drivebr");

        drivefl.setDirection(DcMotorSimple.Direction.FORWARD);

        drivefr.setDirection(DcMotorSimple.Direction.REVERSE);
        drivebl.setDirection(DcMotorSimple.Direction.FORWARD);
        drivebr.setDirection(DcMotorSimple.Direction.REVERSE);
        robotAngle = new IMU("gyro");
        Motor.Encoder middleEncoderPos = hardwareMap.get(Motor.Encoder.class, "drivefl");
        Motor.Encoder centerEncoderPos = hardwareMap.get(Motor.Encoder.class, "drivefr");

        rotationalPID = new PID(.054,0,.000015);
        odometry = new Odometry(
            robotAngle::getAngle,
            middleEncoderPos::getDistance,
            centerEncoderPos::getDistance
        );  

    }

    /**
     * Sets the same power to all four motors
     * @param power
     */
    public void setAllPower(double power){

        drivefl.setPower(power);
        drivefr.setPower(power);
        drivebl.setPower(power);
        drivebr.setPower(power);

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

        drivefr.setPower(frPower);
        drivefl.setPower(flPower);
        drivebr.setPower(brPower);
        drivebl.setPower(blPower);


    }



    public void resetMotors(){
        drivefl.setMode(STOP_AND_RESET_ENCODER);
        drivefr.setMode(STOP_AND_RESET_ENCODER);
        drivebl.setMode(STOP_AND_RESET_ENCODER);
        drivebr.setMode(STOP_AND_RESET_ENCODER);

        drivefl.setMode(RUN_WITHOUT_ENCODER);
        drivefr.setMode(RUN_WITHOUT_ENCODER);
        drivebl.setMode(RUN_WITHOUT_ENCODER);
        drivebr.setMode(RUN_WITHOUT_ENCODER);
    }
    public Point getPosition(IMU imu){
        double yDist = (drivefl.getCurrentPosition() + drivefl.getCurrentPosition() + drivefl.getCurrentPosition() + drivefl.getCurrentPosition()) / 4.0;
        double xDist = (drivefl.getCurrentPosition() - drivefl.getCurrentPosition() + drivefl.getCurrentPosition() - drivefl.getCurrentPosition()) / 4.0;
        Point unshiftedDistances = unShift(new Point(xDist, yDist), imu.getAngle());
        return new Point(xDist, yDist);
    }
        /*
        double frPosition = fr.getCurrentPosition();
        double flPosition = fl.getCurrentPosition();
        double brPosition = br.getCurrentPosition();
        double blPosition = bl.getCurrentPosition();
        double avgPosition =(frPosition + flPosition + brPosition + blPosition) / 4.0;
        return avgPosition;
         */






    /**
     * Translates the robot autonomously a certain distance known as ticks
     * @param ticks
     */

    public void strafe(double ticks, double targetAngle, double strafeAngle, IMU imu){

        // Reset our encoders to 0
        resetMotors();

        targetAngle = closestAngle(targetAngle, imu.getAngle());

        // Calculate our x and y powers
        double xPower = cos(strafeAngle, DEGREES);
        double yPower = sin(strafeAngle, DEGREES);

        // Calculate the distances we need to travel
        double xDist = xPower * ticks;
        double yDist = yPower * ticks;

        // Initialize our current position variables
        Point curPos = new Point(0, 0);
        double curHDist = 0;

        while (curHDist < ticks){
            curPos = getPosition(imu);
            curHDist = Math.hypot(curPos.x, curPos.y);

            Point shiftedPowers = shift(new Point(xPower, yPower), imu.getAngle());
            setDrivePower(shiftedPowers.y, shiftedPowers.x, rotationalPID.update(imu.getAngle() - targetAngle, false), 0.3);

            // Log some data out for debugging
            multTelemetry.addData("curPos", "(" + curPos.x + ", " + curPos.y + ")");
            multTelemetry.addData("curHDist", curHDist);
            multTelemetry.update();
        }
        setAllPower(0);
    }

    /**
     * Rotates the robot autonomously a certain number of degrees with a margin of error
     * @param degrees
     * @param gyro
     */
    public void turn(double degrees, IMU gyro){
        ElapsedTime timer = new ElapsedTime();
         double closestAngle = closestAngle(degrees, gyro.getAngle());
         timer.reset();
        while(timer.seconds() < 1){
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
