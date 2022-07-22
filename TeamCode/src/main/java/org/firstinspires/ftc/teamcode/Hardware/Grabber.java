package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.util.ElapsedTime;

public class Grabber {
    Arm arm;
    Dropper dropper;
    Position currentPosition = Position.HOLD;
    ElapsedTime timer = new ElapsedTime();
    public enum Position{
        HOLD, DROP, GRAB
    }

    public Grabber(){
        arm = new Arm("arm");
        dropper = new Dropper ("grabber");
    }
    public void update(boolean wobbleButton, boolean dropperButton){
        switch(currentPosition){
            case HOLD:
                arm.mid();
                dropper.ringhold();
                timer.reset();
                if(wobbleButton){
                    currentPosition = Position.GRAB;
                }
                if(dropperButton){
                    currentPosition = Position.DROP;
                }
                break;
            case GRAB:
                if (timer.seconds() < 1){
                    arm.down();
                    dropper.drop();
                }
                else if (timer.seconds() < 1.5){
                    dropper.wobblehold();
                }
                else{
                    arm.up();
                }
                if(dropperButton){
                    currentPosition = Position.DROP;
                    timer.reset();
                }
                break;
            case DROP:
                if (timer.seconds() < 0.5){
                    arm.mid();
                }
                else if (timer.seconds() < 1){
                    dropper.drop();
                }
                else{
                    currentPosition = Position.HOLD;
                }
                break;
        }


    }

}
