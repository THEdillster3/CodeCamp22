package org.firstinspires.ftc.teamcode.Autonomous;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.multTelemetry;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.setOpMode;

import org.firstinspires.ftc.teamcode.Hardware.Robot;

@Autonomous(name="LinearAuto", group="Autonomous Linear Opmode")
public class LinearAuto extends LinearOpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private Robot zuckerberg;


    public void initialize(){
        setOpMode(this);

        multTelemetry.addData("Status", "Initalized");
        multTelemetry.update();
        zuckerberg = new Robot();
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public void runOpMode()
    {

        initialize();


        multTelemetry.addLine("Waiting for start");
        multTelemetry.update();
        waitForStart();

        if (opModeIsActive()){
            zuckerberg.drivetrain.strafe(175);
            zuckerberg.drivetrain.turn(-90, zuckerberg.gyro);
            //zuckerberg.drivetrain.
        }
   }
}
