package org.firstinspires.ftc.teamcode.JoystickResponseCurves;

import com.qualcomm.robotcore.util.Range;

public class ResponseCurve {
    public static double normalCurve(double input) {
        double sign = Math.signum(input);
        input = Math.abs(input);
        if (input < 0.3) return 0.3 * sign;
        return input * sign;
    }

    public static double speedCurve(double input) {
        double sign = Math.signum(input);
        input = Math.abs(input);
        double number = 3.78 * input - 12.4889 * input * input + 18.9541 * Math.pow(input, 3) - 9.2703 * Math.pow(input, 4);
        return Range.clip(number, 0, 1) * sign;
    }

    public static double precisionCurve(double input) {
        double sign = Math.signum(input);
        input = Math.abs(input);
        double number = 3.9925 * input - 10.426 * input * input + 10.266 * Math.pow(input, 3) - 2.823 * Math.pow(input, 4);
        return Range.clip(number, 0, 1) * sign;
    }
}
