package robot.subsystems.Commands;

import edu.wpi.first.wpilibj.command.Command;
import robot.Robot;
import robot.RobotContainer;

public class DriveCommand extends Command {

    private double speed;

    public DriveCommand(double speed){
        requires(RobotContainer.drivetrain);
        this.speed = speed;
    }

    @Override
    protected void initialize() {
        RobotContainer.drivetrain.setLeftSpeed(speed);
        RobotContainer.drivetrain.setRightSpeed(speed);
    }

    @Override
    protected void execute() {
        System.out.println(speed);

    }

    @Override
    protected boolean isFinished() {
        return Robot.m_robotContainer.xbox.getYButton();
    }

    @Override
    protected void interrupted() {

    }

    @Override
    protected void end() {
        RobotContainer.drivetrain.setRightSpeed(0);
        RobotContainer.drivetrain.setLeftSpeed(0);
    }
}
