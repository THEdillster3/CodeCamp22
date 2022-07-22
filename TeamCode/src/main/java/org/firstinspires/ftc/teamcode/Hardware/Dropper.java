package org.firstinspires.ftc.teamcode.Hardware;

import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.hardwareMap;

import com.qualcomm.robotcore.hardware.Servo;

public class Dropper {
    String id;
    Servo servo;

    double DROP = .1;
    double WOBBLEHOLD = .45;
    double RINGHOLD = .3;



    public Dropper(String id){
        servo = hardwareMap.get(Servo.class, id);
    }

    public void ringhold(){
        servo.setPosition(RINGHOLD);
    }
    public void wobblehold(){
        servo.setPosition(WOBBLEHOLD);
    }
    public void drop(){
        servo.setPosition(DROP);
    }

    /** TODO: ATTRIBUTES
     *  - String left_id
     *  - String right_id
     *  - Servo left_servo
     *  - Servo right_servo
     *
     *  - double LEFT_OPEN_POSITION = ...
     *  - double LEFT_CLOSE_POSITION = ...
     *  - double RIGHT_OPEN_POSITION = ...
     *  - double RIGHT_CLOSE_POSITION = ...
      */

    /** TODO: CONSTRUCTOR
     *  - Instantiate servos using the ids
     */

    /**
     *
     */

}
