// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;


/** An example command that uses an example subsystem. */
public class SwerveCommand extends Command
{
    private final SwerveSubsystem swerveSubsystem;
    private final DoubleSupplier xSpeed;
    private final DoubleSupplier ySpeed;
    private final DoubleSupplier rot;
    private final BooleanSupplier fieldRelative;
    private final BooleanSupplier openloop;


    public SwerveCommand(SwerveSubsystem swerveSubsystem, DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier rot, BooleanSupplier fieldRelative, BooleanSupplier openloop)
    {
        this.swerveSubsystem = swerveSubsystem;
        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        this.rot = rot;
        this.fieldRelative = fieldRelative;
        this.openloop = openloop;
        addRequirements(swerveSubsystem);
    }


    @Override
    public void execute() {
        double x = MathUtil.applyDeadband(xSpeed.getAsDouble(), 0.05);
        double y = MathUtil.applyDeadband(ySpeed.getAsDouble(), 0.05);
        double z = MathUtil.applyDeadband(rot.getAsDouble(), 0.05);

        swerveSubsystem.drive(
                new Translation2d(x, y).times(Constants.ChassisConstants.Max_Speed),
                z * Constants.ChassisConstants.Max_Speed,
                !fieldRelative.getAsBoolean(),true);
    }


}
