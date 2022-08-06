package org.firstinspires.ftc.teamcode.Hardware.Sensors;

import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.hardwareMap;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;

import java.util.Base64;

public class Odometry {

    public static double TRACKWIDTH = 13.027;
    public static double CENTER_WHEEL_OFFSET = 3.46;
    public static double WHEEL_DIAMETER = 2.0 * 0.6889764;
    public static double TICKS_PER_REV = 8192;
    public static double DISTANCE_PER_PULSE = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;

    public MotorEx leftEncoder, rightEncoder, perpEncoder;
    public HolonomicOdometry odometry;

    private Pose2d currentLocation;

    public Odometry(){

        leftEncoder = new MotorEx(hardwareMap, "drivefl");
        rightEncoder = new MotorEx(hardwareMap, "drivebl");
        perpEncoder = new MotorEx(hardwareMap, "drivefr");
        leftEncoder.resetEncoder();
        perpEncoder.resetEncoder();

        leftEncoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        rightEncoder.setDistancePerPulse(-DISTANCE_PER_PULSE);
        perpEncoder.setDistancePerPulse(-DISTANCE_PER_PULSE);

        odometry = new HolonomicOdometry(
          leftEncoder::getDistance,
          rightEncoder::getDistance,
          perpEncoder::getDistance,
          TRACKWIDTH,
          CENTER_WHEEL_OFFSET
        );

        updateOdometry();

    }

    public void updateOdometry(){
        odometry.updatePose();
        currentLocation = odometry.getPose();
    }

    public Pose2d getCurrentLocation(){
        return currentLocation;
    }





}
