package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.jetbrains.annotations.NotNull;

import java.util.List;

public class Hardware {
    public DcMotorEx leftWheel = null;
    public DcMotorEx rightWheel = null;

    //-------------------LIFTER-------------------------
    public DcMotorEx leftLifter = null;
    public DcMotorEx rightLifter = null;

    //--------------------SLIDER------------------
    public DcMotorEx slider = null;
    public Servo gripperFront = null;
    public Servo gripperBack = null;

    //---------------------SENSORS-----------------
    public RevTouchSensor button = null;
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

        gripperFront = hwMap.get(Servo.class, "front");
        gripperBack = hwMap.get(Servo.class, "back");

        //-------------------SENSORS-------------------------------(imu is separate from this)
        button = hwMap.get(RevTouchSensor.class, "touch");

        //Auto caching is the simplest way to work with bulk reads. Small risk of
        //performance loss with duplicate calls per hardware iteration, but overall should be fine.
        List<LynxModule> allHubs = hwMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
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
