package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.Controls.ButtonControls.ButtonState.TAP;
import static org.firstinspires.ftc.teamcode.Controls.ButtonControls.Input.CROSS;
import static org.firstinspires.ftc.teamcode.Controls.ButtonControls.Input.TRIANGLE;
import static org.firstinspires.ftc.teamcode.Controls.JoystickControls.Input.LEFT;
import static org.firstinspires.ftc.teamcode.Controls.JoystickControls.Input.RIGHT;
import static org.firstinspires.ftc.teamcode.Controls.JoystickControls.Value.INVERT_SHIFTED_X;
import static org.firstinspires.ftc.teamcode.Controls.JoystickControls.Value.INVERT_SHIFTED_Y;
import static org.firstinspires.ftc.teamcode.Controls.JoystickControls.Value.SHIFTED_X;
import static org.firstinspires.ftc.teamcode.Controls.JoystickControls.Value.X;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.multTelemetry;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Controls.Controller;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Hardware.Sensors.Odometry;
import org.firstinspires.ftc.teamcode.Utilities.MathUtils;
import org.firstinspires.ftc.teamcode.Utilities.OpModeUtils;
import org.firstinspires.ftc.teamcode.Utilities.PID;
import org.opencv.core.Point;

@TeleOp(name = "Intelligent Trans", group = "Iterative Opmode")
public class IntelligentTranslation extends OpMode {

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

        frontBackPID = new PID(0.079,0.0,0.009);
        translationalPID = new PID(0.14,0.0,0.01);
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

        double correction = -rotationalPID.update(robot.gyro.getAngle() - setPoint, false);
        double rotation;
        if(!(controller.get(RIGHT, X) == 0)){
            rotation = controller.get(RIGHT, X);
            wasTurning = true;
        }else{
            if(wasTurning){
                setPoint = robot.gyro.getAngle();
                wasTurning = false;
            }
            rotation = correction;
        }

        odometry.updateOdometry();
        Point shiftedPoint = new Point(
                targetPoint.x - odometry.getCurrentLocation().getX(),
                targetPoint.y - odometry.getCurrentLocation().getY()
        );
        Point rotatedPoint = MathUtils.shift(shiftedPoint, -robot.gyro.getAngle());

        double driveCorrection = frontBackPID.update(rotatedPoint.x, false);//odometry.getCurrentLocation().getX() - rotatedPoint.x,false);
        double strafeCorrection = translationalPID.update(rotatedPoint.y, false);//odometry.getCurrentLocation().getY() - rotatedPoint.y,false);

        double correctionStrength = Range.clip((1 / Math.pow(MathUtils.absDistBetweenCoords(odometry.getCurrentLocation().getX(), odometry.getCurrentLocation().getY(), targetPoint.x, targetPoint.y) / 6, 3)), 0, 1.2);

        controller.setJoystickShift(LEFT, robot.gyro.getAngle());

        double drive = (controller.get(LEFT, INVERT_SHIFTED_Y) / (correctionStrength / 2)) + (driveCorrection * correctionStrength);
        double strafe = (controller.get(LEFT, SHIFTED_X) / (correctionStrength / 2)) + (strafeCorrection * correctionStrength);


        robot.drivetrain.setDrivePower(drive, -strafe, rotation, 1);
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
