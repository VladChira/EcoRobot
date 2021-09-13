package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.Miscellaneous.*;
import org.firstinspires.ftc.teamcode.Wrappers.ClawWrapper;
import org.firstinspires.ftc.teamcode.Wrappers.LifterWrapper;

@TeleOp(group = "Main")
public class Driving extends LinearOpMode {
    ControllerInput controller1;
    ControllerInput controller2;

    Hardware robot;
    LifterWrapper lifter;
    ClawWrapper claw;

    double drive, turn;

    private boolean newButtonState = false;
    private boolean oldButtonState = false;

    private double lifterOldPower = 0;
    private double lifterNewPower = 0;

    private boolean oldLeftBumper;
    private boolean oldRightBumper;
    private boolean newLeftBumper;
    private boolean newRightBumper;

    @Override
    public void runOpMode() throws InterruptedException {
        initSubsystems();
        controller1 = new ControllerInput(gamepad1);
        controller2 = new ControllerInput(gamepad2);

        waitForStart();

        while (opModeIsActive()) {
            controller1.update();
            controller2.update();

            handleDriving();
            handleLifter();
            handleSlider();
            handleClaw();
        }
    }

    void handleSlider() {
        newLeftBumper = gamepad1.left_bumper;
        newRightBumper = gamepad1.right_bumper;
        if ((newLeftBumper != oldLeftBumper) || (newRightBumper != oldRightBumper)) {
            if (robot.slider.getCurrentPosition() < 40) {
                newLeftBumper = false;
            }
            if (!newLeftBumper && !newRightBumper)
                robot.slider.setPower(0);
            else if (newLeftBumper && !newRightBumper)
                robot.slider.setPower(-1);
            else if (!newLeftBumper)
                robot.slider.setPower(1);
        }
        oldLeftBumper = newLeftBumper;
        oldRightBumper = newRightBumper;
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

        if (gamepad1.left_trigger > 0) {
            lifterNewPower = -gamepad1.left_trigger;
        } else if (gamepad1.right_trigger > 0) {
            lifterNewPower = gamepad1.right_trigger;
        } else {
            lifterNewPower = 0;
        }
        if (lifterNewPower != lifterOldPower) {
            if (newButtonState && lifterNewPower < 0) {
                //if button is pressed do not allow downwards movement
                lifterNewPower = 0;
            }
            lifter.setLifterPower(lifterNewPower);
        }
        lifterOldPower = lifterNewPower;
    }

    void handleClaw() {
        if (controller1.AOnce()) {
            claw.attach();
        }
        if (controller1.BOnce()) {
            claw.detach();
        }
    }

    void handleDriving() {
        drive = controller1.left_stick_y;
        turn = -controller1.right_stick_x;

        double leftPower = Range.clip(drive + turn, -1.0, 1.0);
        double rightPower = Range.clip(drive - turn, -1.0, 1.0);

        robot.leftWheel.setPower(leftPower);
        robot.rightWheel.setPower(rightPower);
    }

    void initSubsystems() {
        robot = new Hardware();
        robot.init(hardwareMap);
        lifter = new LifterWrapper(robot.leftLifter, robot.rightLifter, robot.button);
        claw = new ClawWrapper(robot.gripperFront, robot.gripperBack);
        claw.initial();
    }
}