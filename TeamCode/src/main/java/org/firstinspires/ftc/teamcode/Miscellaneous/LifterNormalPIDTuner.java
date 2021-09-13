package org.firstinspires.ftc.teamcode.Miscellaneous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.Wrappers.LifterWrapper;

/**
 * Tuner for default SDK PIDF controller.
 */

@TeleOp
@Config
public class LifterNormalPIDTuner extends LinearOpMode {
    enum Mode {
        DRIVER_MODE,
        TUNING_MODE
    }

    private Mode mode;
    private static boolean freeze = false;

    public Hardware robot;
    public LifterWrapper lifter;
    ControllerInput controller;

    public static double kP = 5;
    public static double kI = 0;
    public static double kD = 0;
    public static double f = 0;

    public static int currentPosition;
    public static int targetPosition = 5000;

    private boolean newButtonState = false;
    private boolean oldButtonState = false;

    private double lifterOldPower = 0;
    private double lifterNewPower = 0;

    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot = new Hardware();
        robot.init(hardwareMap);
        lifter = new LifterWrapper(robot.leftLifter, robot.rightLifter, robot.button);
        controller = new ControllerInput(gamepad1);

        PIDFCoefficients defaultPIDF = lifter.getPIDFCoeffs(DcMotor.RunMode.RUN_TO_POSITION);
        telemetry.addLine("Default PIDF values for RUN_TO_POSITION:");
        telemetry.addData("kP", defaultPIDF.p);
        telemetry.addData("kI", defaultPIDF.i);
        telemetry.addData("kD", defaultPIDF.d);
        telemetry.addData("f", defaultPIDF.f);
        telemetry.update();

        mode = Mode.TUNING_MODE;
        freeze = false;
        waitForStart();

        telemetryThread threadObj = new telemetryThread();
        Thread thread1 = new Thread(threadObj);
        thread1.start();

        while (opModeIsActive()) {
            controller.update();

            switch (mode) {
                case TUNING_MODE:
                    if (controller.rightBumperOnce()) {
                        mode = Mode.DRIVER_MODE;
                    }
                    if (controller.AOnce()) {
                        //move with built-in control loop
                        int sign = 1;
                        double power = 1;
                        if (currentPosition > targetPosition) { sign = -1; power = 0.7 ; }
                        lifter.setPIDFCoeffs(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(kP, kI, kD, f, MotorControlAlgorithm.PIDF));
                        lifter.setTargetPosition(targetPosition);
                        lifter.setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
                        lifter.setLifterPower(power * sign);
                        timer.reset();
                        while (lifter.isBusy() && opModeIsActive() && !Thread.currentThread().isInterrupted()) {
                            //move to that position
                        }
                        lifter.stop();
                        freeze = true;
                        telemetry.log().clear();
                        telemetry.addData("Ticks", lifter.getLifterPosition());
                        telemetry.addData("Arrived. Elapsed time: ", timer.seconds());
                        telemetry.addData("Moved", Math.abs(targetPosition - currentPosition));
                        telemetry.update();
                        sleep(5000);
                        freeze = false;
                        lifter.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    }
                    break;
                case DRIVER_MODE:
                    if (controller.leftBumperOnce()) {
                        mode = Mode.TUNING_MODE;
                    }
                    handleLifter();
                    break;
            }
        }
    }

    void handleLifter() {
        newButtonState = robot.button.isPressed();
        if (newButtonState && !oldButtonState) {
            //kill the motors when the button is pressed
            lifter.stop();
            lifter.setRunMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            lifter.setRunMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }
        oldButtonState = newButtonState;

        lifterNewPower = -controller.right_stick_y;
        if (lifterNewPower != lifterOldPower) {
            if (newButtonState && lifterNewPower < 0) {
                //if button is pressed do not allow downwards movement
                lifterNewPower = 0;
            }
            lifter.setLifterPower(lifterNewPower);
        }
        lifterOldPower = lifterNewPower;
    }

    class telemetryThread implements Runnable {
        @Override
        public void run() {
            telemetry.log().clear();
            while (opModeIsActive() && !Thread.currentThread().isInterrupted()) {
                if (freeze) continue;
                currentPosition = lifter.getLifterPosition();
                if (mode == Mode.TUNING_MODE) telemetry.addLine("TUNING MODE ENABLED");
                if (mode == Mode.DRIVER_MODE) telemetry.addLine("DRIVER MODE ENABLED");
                telemetry.addData("Current Position", currentPosition);
                telemetry.addData("Target Position", targetPosition);
                telemetry.update();
                sleep(5);
            }

        }
    }
}


