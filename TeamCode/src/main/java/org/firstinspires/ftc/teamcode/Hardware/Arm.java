package org.firstinspires.ftc.teamcode.Hardware;


import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.hardwareMap;

import com.qualcomm.robotcore.hardware.Servo;

public class Arm {
    private String id;
    private Servo servo;

    double UP = .7;
    double DOWN = .1;
    double MID = .3;

    public Arm(String id){
        servo = hardwareMap.get(Servo.class, id);
    }
    public void position(double position){
        servo.setPosition(position);
    }
    public void up(){
        servo.setPosition(UP);
    }
    public void mid(){
        servo.setPosition(MID);
    }
    public void down(){
        servo.setPosition(DOWN);
    }
}