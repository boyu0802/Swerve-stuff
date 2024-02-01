package frc.lib.math;

import javax.swing.text.Position;

public class Conversion {
    public static double cancoderToDegrees(double cancoderPosition){
        return cancoderPosition* 360;
    };

    public static double falconToMeters(double gearRatio, double circumference, double falconPosition){
        return (falconPosition/gearRatio * 2048.0)*circumference;
    }

    public static double falconToRPM(double falconCounts){ //do driveMotor.getVelocity().getValue() to get the velocity in Rounds per second
        return falconCounts*(600/2048);
    }

    public static double falconToMPS(double falconCounts, double circumference){
        double RPM = falconToRPM(falconCounts);
        return (RPM * circumference)/ 60;
    }

    public static double MPSToRPS(double wheelSpeed, double circumference){
        return wheelSpeed/circumference;
    }

}