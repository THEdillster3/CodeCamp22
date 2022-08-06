package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.Controls.ButtonControls.ButtonState.DOWN;
import static org.firstinspires.ftc.teamcode.Controls.ButtonControls.ButtonState.TAP;
import static org.firstinspires.ftc.teamcode.Controls.ButtonControls.Input.CIRCLE;
import static org.firstinspires.ftc.teamcode.Controls.ButtonControls.Input.LB1;
import static org.firstinspires.ftc.teamcode.Controls.ButtonControls.Input.SQUARE;
import static org.firstinspires.ftc.teamcode.Controls.ButtonControls.Input.TRIANGLE;
import static org.firstinspires.ftc.teamcode.Controls.JoystickControls.Input.LEFT;
import static org.firstinspires.ftc.teamcode.Controls.JoystickControls.Input.RIGHT;
import static org.firstinspires.ftc.teamcode.Controls.JoystickControls.Value.INVERT_SHIFTED_X;
import static org.firstinspires.ftc.teamcode.Controls.JoystickControls.Value.INVERT_SHIFTED_Y;
import static org.firstinspires.ftc.teamcode.Controls.JoystickControls.Value.SHIFTED_X;
import static org.firstinspires.ftc.teamcode.Controls.JoystickControls.Value.SHIFTED_Y;
import static org.firstinspires.ftc.teamcode.Controls.JoystickControls.Value.X;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.multTelemetry;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.setOpMode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Controls.Controller;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Utilities.PID;

@TeleOp(name="Iterative TeleOp", group="Iterative Opmode")
public class IterativeTeleOp extends OpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    Robot zuckerberg;
    boolean speed = false;
    boolean last = false;
    Controller controller;
    private PID pid;
    private double setPoint = 0;
    double rotation;
    private double correction = 0;
    private boolean wasTurning;
    private boolean direction = true;


    /*
     * Code to run ONCE when the driver hits INIT

     */

    @Override
    public void init() {
        setOpMode(this);


        zuckerberg = new Robot();
        controller = new Controller(gamepad1);

        pid = new PID(0.0054, 0, 0.000015);




        multTelemetry.addData("Status", "Initialized");
        multTelemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY

     */

    @Override
    public void init_loop() {





        multTelemetry.addData("Status", "InitLoop");
        multTelemetry.update();
    }



    @Override
    public void start() {
        runtime.reset();
        multTelemetry.addData("Status", "Started");
        multTelemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP

     */

    @Override
    public void loop() {
        Controller.update();


        correction = pid.update(zuckerberg.gyro.getAngle() - setPoint, true);
        double rotation;
        if(!(controller.get(RIGHT, X) == 0)){
            rotation = controller.get(RIGHT, X);
            wasTurning = true;
        }else{
            if(wasTurning){
                setPoint = zuckerberg.gyro.getAngle();
                wasTurning = false;
            }
            rotation = correction;
        }


        if(controller.get(TRIANGLE, TAP)){
            setPoint += 90;
        }


        if(controller.get(CIRCLE, DOWN) && direction){
            //zuckerberg.duck.spin(1);
        }else if(controller.get(CIRCLE, DOWN) && !direction){
            //zuckerberg.duck.spin(-1);
        }else {
            //zuckerberg.duck.spin(0);}
        }

        //zuckerberg.grabber.update(controller.get(SQUARE, TAP), controller.get(TRIANGLE, TAP));





        //This sets the power of the motors
        double power;
        if (!speed){
            power = (0.5);
        }
        else{
            power = (0.1);
        }

        controller.setJoystickShift(LEFT, zuckerberg.gyro.getAngle());


        double drive = controller.get(LEFT, INVERT_SHIFTED_Y);
        double strafe = controller.get(LEFT, SHIFTED_X);


        zuckerberg.drivetrain.setDrivePower(drive,strafe,rotation,power);
        /*
        This sets the speed ensuring that it doesn't
        continuously switch speeds when the button is pressed

         */



        /*

                    Y O U R   C O D E   H E R E



             ----------- L O G G I N G -----------

         */

        multTelemetry.addData("Status", "TeleOp Running");
        multTelemetry.addData("Angle",zuckerberg.gyro.getAngle());
        multTelemetry.update();

    }

    @Override
    public void stop() {

        /*
                    Y O U R   C O D E   H E R E



         */
    }
}
