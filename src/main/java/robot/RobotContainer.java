
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.buttons.POVButton;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import robot.subsystems.Commands.DriveCommand;
import robot.subsystems.DrivetrainSubsystem;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class RobotContainer {

    public static DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();
    public XboxController xbox = new XboxController(2);

    Button b = new JoystickButton(xbox, 1);
    Button x = new JoystickButton(xbox, 3);
    Button y = new JoystickButton(xbox, 4);

    private void bindButtonsToCommands(){}

    public RobotContainer() {
        bindButtonsToCommands();
    }


}