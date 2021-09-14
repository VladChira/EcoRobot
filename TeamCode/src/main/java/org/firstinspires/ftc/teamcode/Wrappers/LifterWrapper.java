package org.firstinspires.ftc.teamcode.Wrappers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
public class LifterWrapper {
    private final DcMotorEx rightLifter;
    private final DcMotorEx leftLifter;
    private final RevTouchSensor button;

    public static double MAX_VEL = ticksToCM(0.0);
    public static double MAX_ACC = ticksToCM(0.0);
    public static double MAX_JERK = ticksToCM(0.0);

    public static double TICKS_PER_INCH = 0;

    public LifterWrapper(DcMotorEx leftLifter, DcMotorEx rightLifter, RevTouchSensor button) {
        this.leftLifter = leftLifter;
        this.rightLifter = rightLifter;
        this.button = button;
    }

    public void setRunMode(DcMotor.RunMode runMode) {
        this.leftLifter.setMode(runMode);
    }

    public void setTargetPosition(int position) {
        leftLifter.setTargetPosition(position);
    }

    //bad idea to use this
    public void setTargetTolerance(int tolerance) {
        leftLifter.setTargetPositionTolerance(tolerance);
    }

    public void setPIDFCoeffs(DcMotor.RunMode runMode, PIDFCoefficients pidfCoeffs) {
        leftLifter.setPIDFCoefficients(runMode, pidfCoeffs);
    }

    public int getLifterPosition() {
        return leftLifter.getCurrentPosition();
    }

    public boolean isBusy() {
        return leftLifter.isBusy();
    }

    public double getLifterVelocity() {
        return leftLifter.getVelocity(AngleUnit.DEGREES);
    }

    public void setLifterPower(double power) {
        if (Math.abs(power) < 0.05) stop();
        this.rightLifter.setPower(power);
        this.leftLifter.setPower(power);
    }

    public void stop() {
        this.rightLifter.setPower(0.0);
        this.leftLifter.setPower(0.0);
    }

    public PIDFCoefficients getPIDFCoeffs(DcMotorEx.RunMode runMode) {
        return leftLifter.getPIDFCoefficients(runMode);
    }

    public static double ticksToCM(double ticks) {
        return ticks; //TODO convert correctly
    }
}
