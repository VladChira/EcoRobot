package org.firstinspires.ftc.teamcode.Miscellaneous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.Wrappers.LifterWrapper;

/**
 * Tuner for Roadrunner PIDF controller.
 */

@TeleOp
@Config
public class LifterRoadrunnerPID extends LinearOpMode {
    enum Mode {
        DRIVER_MODE,
        TUNING_MODE
    }

    private Mode mode;
    private static boolean freeze = false;

    public Hardware robot;
    public LifterWrapper lifter;
    ControllerInput controller;

    public static double kP = 1;
    public static double kI = 0;
    public static double kD = 0;

    public static int currentPosition;
    public static int targetPosition = 5000;

    private boolean newButtonState = false;
    private boolean oldButtonState = false;

    private double lifterOldPower = 0;
    private double lifterNewPower = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot = new Hardware();
        robot.init(hardwareMap);
        lifter = new LifterWrapper(robot.leftLifter, robot.rightLifter, robot.button);
        controller = new ControllerInput(gamepad1);

        mode = Mode.TUNING_MODE;
        freeze = false;
        waitForStart();

        telemetryThread threadObj = new telemetryThread();
        Thread thread1 = new Thread(threadObj);
        //thread1.start();

        while (opModeIsActive()) {
            controller.update();

            switch (mode) {
                case TUNING_MODE:
                    if (controller.rightBumperOnce()) {
                        mode = Mode.DRIVER_MODE;
                    }
                    if (controller.AOnce()) {
                        MotionProfile profile = getMotionProfile(currentPosition, targetPosition);

                        PIDCoefficients coeffs = new PIDCoefficients(kP, kI, kD);
                        PIDFController controller = new PIDFController(coeffs);

                        double power = 1.0, sign = 1.0;
                        if (currentPosition > targetPosition) {
                            //move slower when going down
                            sign = -1.0;
                            power = 0.8;
                        }

                        ElapsedTime elapsedTime = new ElapsedTime();
                        //lifter.setLifterPower(power * sign);
                        do {
                            MotionState state = profile.get(elapsedTime.seconds());

                            controller.setTargetPosition(state.getX());
                            controller.setTargetVelocity(state.getV());
                            controller.setTargetAcceleration(state.getA());

                            double correction = controller.update(currentPosition);
                            //lifter.setLifterPower((power + correction) * sign);

                            telemetry.addData("current position", currentPosition);
                            telemetry.addData("target position", targetPosition);
                            telemetry.addData("correction:", correction);
                            telemetry.update();
                        } while (Math.abs(controller.getLastError()) < 5);
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

    MotionProfile getMotionProfile(double current, double target) {
        return MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(current, 0, 0),
                new MotionState(target, 0, 0),
                LifterWrapper.MAX_VEL,
                LifterWrapper.MAX_ACC,
                LifterWrapper.MAX_JERK
        );
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


