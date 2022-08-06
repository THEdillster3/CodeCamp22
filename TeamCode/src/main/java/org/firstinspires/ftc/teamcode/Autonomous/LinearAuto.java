package org.firstinspires.ftc.teamcode.Autonomous;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.Controls.ButtonControls.ButtonState.TAP;
import static org.firstinspires.ftc.teamcode.Controls.ButtonControls.Input.CROSS;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.multTelemetry;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.setOpMode;

import org.firstinspires.ftc.teamcode.Controls.Controller;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Hardware.Sensors.Odometry;

import java.util.function.DoubleSupplier;

@Autonomous(name="LinearAuto", group="Autonomous Linear Opmode")
public class LinearAuto extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private Robot zuckerberg;
    private Controller controller;


    public void initialize() {
        setOpMode(this);

        zuckerberg = new Robot();
        controller = new Controller(gamepad1);


        multTelemetry.addData("Status", "Initalized");
        multTelemetry.update();
    }

    public void BREAKPOINT(){
        Controller.update();
        while (!controller.get(CROSS, TAP)){
            Controller.update();

            multTelemetry.addLine("BREAKPOINT");
            // add your stuff here
            multTelemetry.addData("Angle", zuckerberg.gyro.getAngle());
            multTelemetry.addData("Position", zuckerberg.drivetrain.getPosition(zuckerberg.gyro));
            multTelemetry.update();

        }
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public void runOpMode() {

        initialize();


        multTelemetry.addLine("Waiting for start");
        multTelemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            //zuckerberg.drivetrain.strafe(2000,90,90, zuckerberg.gyro);
            BREAKPOINT();
            zuckerberg.drivetrain.strafe(1000,0,0,zuckerberg.gyro);
            BREAKPOINT();
        }
    }
}
