package org.firstinspires.ftc.teamcode.Wrappers;

import com.qualcomm.robotcore.hardware.Servo;

public class ClawWrapper {

    private final Servo front;
    private final Servo back;

    private boolean attached = false;
    private boolean lowered = false;

    public ClawWrapper(Servo front, Servo back) {
        this.front = front;
        this.back = back;
    }

    public void detach() {
        back.setPosition(0.4);
        raiseFlipper();
        attached = false;
    }

    public void attach() {
        lowerFlipper();
        back.setPosition(0.5);
        attached = true;
    }

    public void lowerFlipper() {
        lowered = true;
        front.setPosition(0);

    }

    public void raiseFlipper() {
        lowered = false;
        front.setPosition(1);
    }

    public void initial() {
        back.setPosition(0.6);
        front.setPosition(1);
    }

    public boolean isAttached() {
        return attached;
    }

    public boolean isLowered() {
        return lowered;
    }
}