package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.wolfpackmachina.bettersensors.Sensors.Gyro;

import org.firstinspires.ftc.teamcode.Hardware.Sensors.IMU;

import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.multTelemetry;

/**
 * A class for containing an FTC Mecanum robot
 */
public class Robot {

   public static ElapsedTime time = new ElapsedTime();
   public Mecanum drivetrain;
   public IMU gyro;

   public Robot(){
      initRobot();

   }


   public void initRobot() {
      gyro = new IMU("imu");


      /*
            I N I T   M O T O R S
       */
      drivetrain = new Mecanum();


      multTelemetry.addData("Status", "Initialized");
      multTelemetry.update();
   }

}