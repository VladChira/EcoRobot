package org.firstinspires.ftc.teamcode.Miscellaneous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.Wrappers.LifterWrapper;

@Config
@Autonomous()
public class LifterMaxVelocityTuner extends LinearOpMode {
    public static double RUNTIME = 2.0;

    private double maxVelocity = 0.0;

    public Hardware robot;
    public LifterWrapper lifter;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addLine("The lifter will go at full speed for " + RUNTIME + " seconds.");
        telemetry.addLine("Press start when ready.");
        telemetry.update();

        robot = new Hardware();
        robot.init(hardwareMap);
        lifter = new LifterWrapper(robot.leftLifter, robot.rightLifter, robot.button);
        waitForStart();

        telemetry.clearAll();
        telemetry.update();

        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        lifter.setLifterPower(1);
        while (!isStopRequested() && timer.seconds() < RUNTIME) {
            double vel = lifter.getLifterVelocity();
            maxVelocity = Math.max(vel, maxVelocity);
            telemetry.addData("Current Velocity", vel);
            telemetry.update();
        }
        lifter.setLifterPower(0.0);

        telemetry.addData("Maximum Velocity", maxVelocity);
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) idle();
    }
}
