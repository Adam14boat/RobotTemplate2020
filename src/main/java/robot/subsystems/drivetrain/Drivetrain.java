package robot.subsystems.drivetrain;

import static robot.Constants.Drivetrain.*;
import static robot.Ports.Drivetrain.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * This is a temporary subsystem from last year.
 */
public class Drivetrain extends SubsystemBase {

    public TalonSRX leftMaster = new TalonSRX(LEFT_MASTER_MOTOR);
    public TalonSRX rightMaster = new TalonSRX(RIGHT_MASTER_MOTOR);
    public VictorSPX right1 = new VictorSPX(RIGHT_SLAVE_MOTOR_1);
    public VictorSPX left1 = new VictorSPX(LEFT_SLAVE_MOTOR_1);
    public VictorSPX right2 = new VictorSPX(RIGHT_SLAVE_MOTOR_2);
    public VictorSPX left2 = new VictorSPX(LEFT_SLAVE_MOTOR_2);

    private Solenoid solenoid = new Solenoid(SOLENOID);
    private Timer coolDown = new Timer();
    PowerDistributionPanel pdp = new PowerDistributionPanel(0);

    private double lastLeftVelocity = 0;
    private double lastRightVelocity = 0;
    private boolean hasShifted = false;

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

    /**
     * runs methods periodically
     */
    public void periodic() {
        autoShift();
        lastLeftVelocity = getLeftVelocity();
        lastRightVelocity = getRightVelocity();
    }

    /**
     * checks if the shifter can shift and chooses to which gear to shift
     */
    public void autoShift() {
        if (canShift()) {
            if ((kickDown() || coastDown()) && !solenoid.get())
                shiftDown();
            else if (canShiftUp() && solenoid.get())
                shiftUp();
        }
    }

    /**
     * if the shifter can shift
     * @return if the robot is not turning and the cool down time is greater than a threshold value
     */
    private boolean canShift() {
        return !isTurning() && (coolDown.get() >= COOLDOWN_TIME || !hasShifted);
    }

    /**
     * @return if the robot is turning
     */
    private boolean isTurning() {
        return Math.abs(getLeftVelocity() - getRightVelocity()) < TURN_THRESHOLD;
    }

    /**
     * if the robot is in kickdown state
     * @return if the velocity is less than a threshold value and the robot's right side is decelerating
     * (the velocity sign is opposite to the acceleration sign) and the current of the motors is correct.
     * We can check only the right velocity and acceleration because we checked in autoShift that the robot is not turning.
     */
    private boolean kickDown() {
        return Math.abs(getRightVelocity()) > 0 && Math.abs(getRightVelocity()) < KICKDOWN_VELOCITY_THRESHOLD
                && getRightVelocitySign() * getRightAcceleration() < KICKDOWN_ACCEL_THRESHOLD
                    && isCorrectCurrent();
    }

    /**
     * if the robot is in coastdown state
     * @return if the robot's velocity is less than a threshold value less than the kickdown threshold
     * and the motor currents are correct. We can check only the velocity of the right side because we checked in autoShift
     * that the robot is not turning.
     */
    private boolean coastDown() {
        return Math.abs(getRightVelocity()) > 0 && Math.abs(getRightVelocity()) < COASTDOWN_THRESHOLD && isCorrectCurrent();
    }

    /**
     * if the shifter can shift up to high gear
     * @return if the velocity and the acceleration (velocity and acceleration with the same sign) are greater than a threshold value.
     * We can check only the right velocity and acceleration because we checked in autoShift that the robot is not turning.
     */
    private boolean canShiftUp() {
        return Math.abs(getRightVelocity()) > UP_SHIFT_VELOCITY_THRESHOLD && getRightVelocitySign() * getRightAcceleration() > UP_SHIFT_ACCEL_THRESHOLD;
    }

    /**
     * shifts down to low gear
     */
    private void shiftDown() {
        solenoid.set(false);
        hasShifted = true;
        startCoolDown();
    }

    /**
     * shifts up to high gear
     */
    private void shiftUp() {
        solenoid.set(true);
        hasShifted = true;
        startCoolDown();
    }

    /**
     * starts the cool down timer
     */
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

    /**
     * @return the acceleration of the right wheel
     */
    public double getRightAcceleration() {
        return (lastRightVelocity - getRightVelocity()) / TIME_STEP;
    }

    /**
     * @return the acceleration of the left wheel
     */
    public double getLeftAcceleration() {
        return (lastLeftVelocity - getLeftVelocity()) / TIME_STEP;
    }

    /**
     * @return the sign of the right velocity
     */
    public int getRightVelocitySign() {
        return (int) Math.signum(getRightVelocity());
    }

    /**
     * @return the sign of the left velocity
     */
    public int getLeftVelocitySign() {
        return (int) Math.signum(getLeftVelocity());
    }


    /**
     * @param channel the pdp port of the motor
     * @return the current of the channel given
     */
    public double getMotorCurrent(int channel) {
        return pdp.getCurrent(channel);
    }

    /**
     * @return if the motor currents are greater than a threshold value
     */
    public boolean isCorrectCurrent() {
        return getMotorCurrent(PDP_PORT_LEFT_MOTOR) > MIN_CURRENT
                && getMotorCurrent(PDP_PORT_RIGHT_MOTOR) > MIN_CURRENT;
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
}
