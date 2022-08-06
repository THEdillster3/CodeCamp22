package org.firstinspires.ftc.teamcode.Utilities;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Hardware.Sensors.IMU;

import java.util.function.DoubleSupplier;

public class Odometry {
    boolean robotIsActive = true;
    DoubleSupplier middle_encoder_pos;
    double prev_middle_encoder_pos;
    DoubleSupplier center_encoder_pos;
    double prev_center_encoder_pos;
    DoubleSupplier forward_offset;
    DoubleSupplier robotAngle;
    IMU imu;
    Robot robot;
    double heading;
    double x_pos;
    double y_pos;

    public Odometry(DoubleSupplier robotAngle, DoubleSupplier middleEncoderPos, DoubleSupplier centerEncoderPos){
        this.robotAngle = robotAngle;
        this.middle_encoder_pos = middleEncoderPos;
        this.center_encoder_pos = centerEncoderPos;
    }

    public void update(){


        double delta_middle_pos = middle_encoder_pos.getAsDouble() - prev_center_encoder_pos;
        double delta_center_encoder_pos = center_encoder_pos.getAsDouble() - prev_center_encoder_pos;


        double heading = robotAngle.getAsDouble();
        double delta_perp_pos = delta_center_encoder_pos - forward_offset.getAsDouble() * heading;

        double delta_x = delta_middle_pos * cos(heading) - delta_perp_pos * sin(heading);
        double delta_y = delta_middle_pos * sin(heading) + delta_perp_pos * cos(heading);

        x_pos += delta_x;
        y_pos += delta_y;

        prev_middle_encoder_pos = middle_encoder_pos.getAsDouble();
        prev_center_encoder_pos = center_encoder_pos.getAsDouble();
    }
}
