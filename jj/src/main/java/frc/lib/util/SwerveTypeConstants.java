package frc.lib.util;

import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.util.Units;

public class SwerveTypeConstants {
    public final double wheelDiameter;
    public final double wheelCircumference;

    public final double angleKp;
    public final double angleKi;
    public final double angleKd;
    public final double angleKf;
    public final double angleGearRatio;
    public final double driveGearRatio;
    public final boolean driveMotorInvert;
    public final boolean angleMotorInvert;
    public final SensorDirectionValue CANCODERInvert;


    public SwerveTypeConstants(double wheelDiameter, double angleKp, double angleKi, double angleKd, double angleKf, double angleGearRatio, double driveGearRatio, boolean driveMotorInvert, boolean angleMotorInvert, SensorDirectionValue CANCODERInvert) {
        this.wheelDiameter = wheelDiameter;
        this.wheelCircumference = wheelDiameter*Math.PI;
        this.angleGearRatio = angleGearRatio;
        this.driveGearRatio = driveGearRatio;
        this.angleKd = angleKd;
        this.angleKi = angleKi;
        this.angleKp = angleKp;
        this.angleKf = angleKf;
        this.driveMotorInvert = driveMotorInvert;
        this.angleMotorInvert = angleMotorInvert;
        this.CANCODERInvert = CANCODERInvert;
    }

    public static SwerveTypeConstants SDSMK4I_L1() {
        double wheelDiameter = Units.inchesToMeters(4.0);
        double angleGearRatio = Angle_MK4I_Gear_Ratio;
        double driveGearRatio = Drive_L1_Gear_Ratio;

        double angleKP = 0.01;
        double angleKI = 0.0;
        double angleKd = 0.0;
        double angleKf = 0.0;

        boolean driveMotorInvert = false;
        boolean angleMotorInvert = true;
        SensorDirectionValue CANCODERInvert = SensorDirectionValue.CounterClockwise_Positive;

        return new SwerveTypeConstants(
                wheelDiameter, angleKP, angleKI, angleKd, angleKf, angleGearRatio,
                driveGearRatio, driveMotorInvert, angleMotorInvert,CANCODERInvert
        );
    }

    final static double Angle_MK4I_Gear_Ratio = (150.0/7.0);
    final static double Drive_L1_Gear_Ratio = 8.14;
}
