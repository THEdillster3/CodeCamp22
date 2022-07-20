package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.wolfpackmachina.bettersensors.Sensors.Gyro;

import static org.firstinspires.ftc.teamcode.Controls.ButtonControls.ButtonState.DOWN;
import static org.firstinspires.ftc.teamcode.Controls.ButtonControls.ButtonState.NOT_TOGGLED;
import static org.firstinspires.ftc.teamcode.Controls.ButtonControls.ButtonState.TAP;
import static org.firstinspires.ftc.teamcode.Controls.ButtonControls.ButtonState.TOGGLE;
import static org.firstinspires.ftc.teamcode.Controls.ButtonControls.ButtonState.UP;
import static org.firstinspires.ftc.teamcode.Controls.ButtonControls.Input.CIRCLE;
import static org.firstinspires.ftc.teamcode.Controls.ButtonControls.Input.CROSS;
import static org.firstinspires.ftc.teamcode.Controls.ButtonControls.Input.TRIANGLE;
import static org.firstinspires.ftc.teamcode.Controls.JoystickControls.Input.LEFT;
import static org.firstinspires.ftc.teamcode.Controls.JoystickControls.Input.RIGHT;
import static org.firstinspires.ftc.teamcode.Controls.JoystickControls.Value.INVERT_SHIFTED_Y;
import static org.firstinspires.ftc.teamcode.Controls.JoystickControls.Value.INVERT_Y;
import static org.firstinspires.ftc.teamcode.Controls.JoystickControls.Value.SHIFTED_X;
import static org.firstinspires.ftc.teamcode.Controls.JoystickControls.Value.X;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.multTelemetry;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.setOpMode;

import org.firstinspires.ftc.teamcode.Controls.Controller;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Hardware.Sensors.IMU;

//@Disabled
@TeleOp(name="Iterative TeleOp", group="Iterative Opmode")
public class IterativeTeleOp extends OpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor fl;
    DcMotor fr;
    DcMotor bl;
    DcMotor br;
    Robot zuckerberg;
    boolean speed = false;
    boolean last = false;
    Controller controller;
    private DcMotor demoMotor;
    private org.firstinspires.ftc.utilities.PID pid;
    private double setPoint = 0;
    double rotation;
    private double correction = 0;
    private boolean wasTurning;



    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        setOpMode(this);

        /*
                    Y O U R   C O D E   H E R E
                                                    */

        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");
        zuckerberg = new Robot();
        controller = new Controller(gamepad1);
        controller.add(CROSS, DOWN, () -> zuckerberg.drivetrain.setAllPower(1));
        pid = new org.firstinspires.ftc.utilities.PID(0, 0, 0);
        correction = pid.update(zuckerberg.gyro.getAngle() - setPoint, true);




        multTelemetry.addData("Status", "Initialized");
        multTelemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

        /*
                    Y O U R   C O D E   H E R E
                                                   */


        multTelemetry.addData("Status", "InitLoop");
        multTelemetry.addData("Motor Position", fl.getCurrentPosition());
        multTelemetry.update();
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
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


        correction= pid.update(zuckerberg.gyro.getAngle() - setPoint, true);


        if(controller.get(CIRCLE, TAP)){
            setPoint += 90;
        }


        rotation = controller.get(RIGHT, X);
        if(!(controller.get(RIGHT, X) == 0)){
            wasTurning = true;
        }else{
            if(wasTurning){
                setPoint = zuckerberg.gyro.getAngle();
                wasTurning = false;
            }
            rotation = correction;
        }

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
                                                    */

        /*
             ----------- L O G G I N G -----------
                                                */
        multTelemetry.addData("Status", "TeleOp Running");
        multTelemetry.update();
        multTelemetry.addData("Angle",zuckerberg.gyro.getAngle());

    }

    @Override
    public void stop() {

        /*
                    Y O U R   C O D E   H E R E
                                                   */

    }
}