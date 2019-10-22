package robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

/**
 * A class holding all of the ports on the robot.
 * Place mechanism-specific ports inside their own sub-class.
 * When accessing a mechanism-specific port, call Ports.[MECHANISM].[PORT_NAME]
 */
public class Ports {

    public static class ExampleSubsystem1 {
        //public static int TALON_PORT = 1;
        //public static boolean TALON_REVERSED = true;
        public static TalonSRXConfiguration TALON_CONFIGURATION = new TalonSRXConfiguration();

        /*
         * In the static field go all of the talon configuration variables. to write the configuration to the talon,
         * call motor.configAllSettings(Ports.SUBSYSTEM.CONFIG);
         */
        static {
            TALON_CONFIGURATION.slot0.kP = 1;
            TALON_CONFIGURATION.slot0.kI = 0;
            TALON_CONFIGURATION.slot0.kD = 0.2;
            TALON_CONFIGURATION.slot0.kF = 0;
            TALON_CONFIGURATION.motionAcceleration = 4 / 100; //Meters per 100 ms
            TALON_CONFIGURATION.motionCruiseVelocity = 2 / 10; //Meters per 100 ms
            TALON_CONFIGURATION.motionCurveStrength = 3; //Curve strength int

        }
    }
}
