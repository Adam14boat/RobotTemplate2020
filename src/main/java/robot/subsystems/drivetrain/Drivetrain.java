package robot.subsystems.drivetrain;

import static robot.Constants.Drivetrain.*;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * This is a temporary subsystem from last year.
 */
public class Drivetrain extends Subsystem {

    public TalonSRX leftMaster = new TalonSRX(16);
    public TalonSRX rightMaster = new TalonSRX(11);
    public VictorSPX right1 = new VictorSPX(12);
    public VictorSPX left1 = new VictorSPX(14);
    public VictorSPX right2 = new VictorSPX(13);
    public VictorSPX left2 = new VictorSPX(15);

    private Solenoid solenoid = new Solenoid(17);
    private Timer coolDown = new Timer();
    public Drivetrain() {
        leftMaster.setInverted(true);
        left1.setInverted(true);
        left2.setInverted(true);
        rightMaster.setInverted(false);
        right1.setInverted(false);
        right2.setInverted(false);

        right1.follow(rightMaster);
        right2.follow(rightMaster);
        left1.follow(leftMaster);
        left2.follow(leftMaster);

        leftMaster.configPeakCurrentLimit(MAX_CURRENT);
        rightMaster.configPeakCurrentLimit(MAX_CURRENT);
    }

    private boolean isTurning() {
        return getLeftVelocity() - getRightVelocity() < TURN_THRESHOLD;
    }
    private void startCoolDown() {
        coolDown.stop();
        coolDown.reset();
        coolDown.start();
    }
    public void setLeftSpeed(double speed) {
        leftMaster.set(ControlMode.PercentOutput, speed);
    }

    public void setRightSpeed(double speed) {
        rightMaster.set(ControlMode.PercentOutput, speed);
    }

    public double getLeftDistance() {
        return convertTicksToDistance(leftMaster.getSelectedSensorPosition());
    }

    public double getRightDistance() {
        return convertTicksToDistance(rightMaster.getSelectedSensorPosition());
    }

    public double getRightVelocity() {
        return convertTicksToDistance(rightMaster.getSelectedSensorVelocity()) * 10;
    }

    public double getLeftVelocity() {
        return convertTicksToDistance(leftMaster.getSelectedSensorVelocity()) * 10;
    }

    public int convertDistanceToTicks(double distance) {
        return (int) (distance * TICKS_PER_METER);
    }

    /**
     * because the max input from the joystick is 1 , the joystick input * max velocity is
     * function which represent the relation
     *
     * @param joystickInput the y value from the joystick
     * @return joystick value in m/s
     */
    public double convertJoystickInputToVelocity(double joystickInput) {
        return joystickInput * MAX_VEL;
    }


    /**
     * limit the drivetrain's right side acceleration to a certain acceleration
     *
     * @param desiredVelocity the desired velocity
     * @return the desired velocity if possible, if not the current velocity plus the max acceleration
     */
    public double limitRightAcceleration(double desiredVelocity) {

        //Take the attempted acceleration and see if it is too high.
        if (Math.abs(desiredVelocity - getRightVelocity()) / TIME_STEP >= MAX_ACCELERATION) {
            return getRightVelocity() + MAX_ACCELERATION;
        }

        return desiredVelocity;
    }

    /**
     * limit the drivetrain's left side acceleration to a certain acceleration
     *
     * @param desiredVelocity the desired velocity
     * @return the desired velocity if possible, if not the current velocity plus the max acceleration
     */
    public double limitLeftAcceleration(double desiredVelocity) {

        //Take the attempted acceleration and see if it is too high.
        if (Math.abs((desiredVelocity - getLeftVelocity()) / TIME_STEP) >= MAX_ACCELERATION) {
            return getLeftVelocity() + MAX_ACCELERATION;
        }

        return desiredVelocity;
    }

    public double convertTicksToDistance(int tick) {
        return tick / TICKS_PER_METER;
    }

    @Override
    protected void initDefaultCommand() {

    }
}
