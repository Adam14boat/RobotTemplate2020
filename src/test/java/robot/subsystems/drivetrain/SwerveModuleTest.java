package robot.subsystems.drivetrain;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;

import static org.junit.Assert.*;

public class SwerveModuleTest {

    private SwerveModule swerveModule;

    @Before
    public void setUp() {
        swerveModule = new SwerveModule(0, new TalonSRX(0), new TalonSRX(1));
    }

    @Test
    public void setTargetAngle() {
        double targetAngle = swerveModule.setTargetAngle(-1.5 * Math.PI, -Math.PI / 4);
        double expectedAngle = Math.PI / 2;

        Assert.assertEquals(expectedAngle, targetAngle, 0.01);
    }
}