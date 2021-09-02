package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.jetbrains.annotations.NotNull;

public class Hardware {
    public DcMotorEx leftWheel = null;
    public DcMotorEx rightWheel = null;

    //-------------------LIFTER-------------------------
    public DcMotorEx leftLifter = null;
    public DcMotorEx rightLifter = null;

    //--------------------SLIDER------------------
    public DcMotorEx slider = null;

    //---------------------SENSORS-----------------
    public TouchSensor button = null;
    public BNO055IMU imu;

    public void init(@NotNull HardwareMap hwMap) {

        leftWheel = hwMap.get(DcMotorEx.class, "leftWheel");
        rightWheel = hwMap.get(DcMotorEx.class, "rightWheel");

        leftWheel.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightWheel.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        leftWheel.setDirection(DcMotorEx.Direction.FORWARD);
        rightWheel.setDirection(DcMotorEx.Direction.REVERSE);

        leftWheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightWheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        leftWheel.setPower(0);
        rightWheel.setPower(0);

        //-------------------------LIFTER-------------------------------------
        leftLifter = hwMap.get(DcMotorEx.class, "lifterL");
        rightLifter = hwMap.get(DcMotorEx.class, "lifterR");

        leftLifter.setPower(0.0);
        rightLifter.setPower(0.0);

        leftLifter.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightLifter.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        leftLifter.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightLifter.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        leftLifter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightLifter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        //-------------------------------SLIDER-----------------------------
        slider = hwMap.get(DcMotorEx.class, "slide");
        slider.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slider.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        slider.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        slider.setDirection(DcMotorEx.Direction.FORWARD);
        slider.setPower(0.0);

        //-------------------SENSORS-------------------------------(imu is separate from this)
        button = hwMap.get(TouchSensor.class, "touch");
    }

    public void initIMU(@NotNull HardwareMap hwMap) {
        imu = hwMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
    }
}
