package org.firstinspires.ftc.teamcode.Wrappers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import kotlin.Experimental;

@Config
public class LifterWrapper {
    public static volatile boolean kill = false;
    private long lastMillis = 0;
    private volatile int currentTicks = 0;
    private volatile int lastTicks = 0;

    public static volatile boolean finished = true;

    private DcMotorEx rightLifter;
    private DcMotorEx leftLifter;
    private TouchSensor button;

    public static double MAX_VEL = 0;
    public static double MAX_ACC = 0;
    public static double MAX_JERK = 0;

    public static double TICKS_PER_INCH = 0;

    public LifterWrapper(DcMotorEx leftLifter, DcMotorEx rightLifter, TouchSensor button) {
        this.leftLifter = leftLifter;
        this.rightLifter = rightLifter;
        this.button = button;
    }

    public void setRunMode(DcMotor.RunMode runMode) {
        this.leftLifter.setMode(runMode);
        this.rightLifter.setMode(runMode);
    }

    public double getLifterPosition() {
        return leftLifter.getCurrentPosition();
    }

    public double getLifterVelocity() {
        return leftLifter.getVelocity(AngleUnit.DEGREES);
    }

    public void setLifterPower(double power) {
        if(power == 0) stop();
        this.rightLifter.setPower(power);
        this.leftLifter.setPower(power);
    }

    public void stop() {
        this.rightLifter.setPower(0.0);
        this.leftLifter.setPower(0.0);
    }
}
