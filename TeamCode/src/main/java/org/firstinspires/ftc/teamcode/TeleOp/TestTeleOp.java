package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.Controls.ButtonControls.ButtonState.TAP;
import static org.firstinspires.ftc.teamcode.Controls.ButtonControls.Input.CROSS;
import static org.firstinspires.ftc.teamcode.Controls.ButtonControls.Input.TRIANGLE;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.multTelemetry;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Controls.Controller;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Hardware.Sensors.Odometry;
import org.firstinspires.ftc.teamcode.Utilities.MathUtils;
import org.firstinspires.ftc.teamcode.Utilities.OpModeUtils;
import org.firstinspires.ftc.teamcode.Utilities.PID;
import org.opencv.core.Point;

@TeleOp(name = "Test TeleOp", group = "Iterative Opmode")
public class TestTeleOp extends OpMode {

    PID frontBackPID; //side to side pid
    PID translationalPID; //self Explanatory'
    PID rotationalPID;
    double targetAngle = 0;
    double xTarget;
    double yTarget;
    Point targetPoint = new Point(0, 0);


    Odometry odometry;


    Controller controller;
    Robot robot;
    private boolean wasTurning;
    private double setPoint = 0;

    public void init(){
        OpModeUtils.setOpMode(this);

        frontBackPID = new PID(0.07,0.0,0.009);
        translationalPID = new PID(0.1,0.0,0.01);
        rotationalPID = new PID(.026,0,.002);
        robot = new Robot();
        controller = new Controller(gamepad1);
        wasTurning = false;
        
        odometry = new Odometry();

        multTelemetry.addData("Status", "Initialized");
        multTelemetry.update();
    }
    @Override
    public void init_loop() {

        multTelemetry.addData("Status", "InitLoop");
        multTelemetry.update();
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        multTelemetry.addData("Status", "Started");
        multTelemetry.update();

    }
    public void loop(){

        Controller.update();

        odometry.updateOdometry();
        yTarget = setPoint;
        Point shiftedPoint = new Point(
                targetPoint.x - odometry.getCurrentLocation().getX(),
                targetPoint.y - odometry.getCurrentLocation().getY()
        );
        Point rotatedPoint = MathUtils.shift(shiftedPoint, -robot.gyro.getAngle());

        double drive = frontBackPID.update(rotatedPoint.x, false);//odometry.getCurrentLocation().getX() - rotatedPoint.x,false);
        double strafe = translationalPID.update(rotatedPoint.y, false);//odometry.getCurrentLocation().getY() - rotatedPoint.y,false);
        robot.drivetrain.setDrivePower(drive, -strafe, -rotationalPID.update(robot.gyro.getAngle() - targetAngle, false), 1);
        if (controller.get(TRIANGLE,TAP)){
            targetPoint = new Point(20, 30);

        }
        if (controller.get(CROSS,TAP)){
            targetPoint = new Point(0, 0);

        }
        multTelemetry.addData("setPoint" ,setPoint);
        multTelemetry.addData("x", odometry.getCurrentLocation().getX());
        multTelemetry.addData("y", odometry.getCurrentLocation().getY());
        multTelemetry.update();
    }


}
