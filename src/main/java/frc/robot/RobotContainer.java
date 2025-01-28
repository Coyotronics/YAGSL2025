// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

//import com.pathplanner.lib.auto.NamedCommands;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */

public class RobotContainer {
    static boolean field_centric = true;
    // The robot's subsystems
    private final DriveSubsystem robot_drive = new DriveSubsystem();

    // The driver's controller
    XboxController driver_controller = new XboxController(OIConstants.kDriverControllerPort);

    // 2nd Drivers's Controller
    XboxController mani_controller = new XboxController(OIConstants.kDriverControllerPort1); // Check ports

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        if (driver_controller.getLeftTriggerAxis() == 1 && driver_controller.getRightTriggerAxis() == 1) {
            field_centric = !field_centric;
        }
        // Configure the button bindings
        configureButtonBindings();

        // Configure default commands
        robot_drive.setDefaultCommand(
                // The left stick controls translation of the robot.
                // Turning is controlled by the X axis of the right stick.
                new RunCommand(

                        () -> robot_drive.drive(
                                -MathUtil.applyDeadband(driver_controller.getLeftY(), OIConstants.kDriveDeadband),
                                -MathUtil.applyDeadband(driver_controller.getLeftX(), OIConstants.kDriveDeadband),
                                -MathUtil.applyDeadband(driver_controller.getRightX(), OIConstants.kDriveDeadband),
                                field_centric, true),

                        robot_drive));
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
     * subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
     * passing it to a
     * {@link JoystickButton}.
     */
    private void configureButtonBindings() {
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return null;
    }

}