// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.SwerveCommand;
import frc.robot.subsystems.SwerveSubsystem;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
    public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    public final static XboxController controller = new XboxController(0);
    public RobotContainer()
    {
        swerveSubsystem.setDefaultCommand(
                new SwerveCommand(
                        swerveSubsystem,
                        () ->controller.getRawAxis(XboxController.Axis.kLeftY.value),
                        () -> -controller.getRawAxis(XboxController.Axis.kLeftX.value),
                        () -> -controller.getRawAxis(XboxController.Axis.kRightX.value),
                        () ->controller.getRightBumper(),
                        () ->controller.getAButton()



                )
        );

        configureBindings();
    }

    private void configureBindings() {
        new JoystickButton(controller, XboxController.Button.kLeftBumper.value)
                .onTrue(new InstantCommand(swerveSubsystem::zeroGyro));


//        new JoystickButton(controller, XboxController.Button.kRightBumper.value).onTrue(new InstantCommand(swerveSubsystem::resetModulesToAbsolute));
    }
}
    

