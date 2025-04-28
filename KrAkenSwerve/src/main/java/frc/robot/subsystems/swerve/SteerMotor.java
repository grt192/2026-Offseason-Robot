package frc.robot.subsystems.swerve;

//Constants Import 
import static frc.robot.Constants.SwerveSteerConstants.STEER_GEAR_REDUCTION;
import static frc.robot.Constants.SwerveSteerConstants.STEER_PEAK_CURRENT;
import static frc.robot.Constants.SwerveSteerConstants.STEER_RAMP_RATE;

//CTRE imports
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;

public class SteerMotor {

    // Motor instance for controlling the drive motor
    private TalonFX motor;

    // Configuration for Kraken stored in one Object
    private final TalonFXConfiguration motorConfig = new TalonFXConfiguration();

    // Cancoder Creation
    private CANcoder cancoder;

    // Configuration for CANCODER stored in one Object
    private final  CANcoderConfiguration cancoderconfig = new CANcoderConfiguration();

    // For fine control of velocity and torque using FOC (Field-Oriented Control)
    private PositionTorqueCurrentFOC positionRequest = new PositionTorqueCurrentFOC(0).withSlot(0);

    // For making positions wrap from 0-1 and resetting to not stack
    private final ClosedLoopGeneralConfigs closedLoopGeneralConfigs = new ClosedLoopGeneralConfigs();


    public SteerMotor(int motorCAN, int encoderCAN) {
        // Set motor and encoder
        motor = new TalonFX(motorCAN);
        cancoder = new CANcoder(encoderCAN);

        // Configure CANcoder and Kraken
        //configureCancoder(); no configs needed all can be done though phoenix Tunner
        configureMotor();


    }

    /**
     * Applying Configurations to CANCODER
     */
    private void configureCancoder() {
        /**
         * Stuff Needed To DO On Software
         *      Inverse Direction
         *      Turn Range into -180 to 180
         */
        // Apply config with retries (max 5 attempts)
        for (int i = 0; i < 5; i++) {
            if (cancoder.getConfigurator().apply(cancoderconfig, 0.1) == StatusCode.OK) {
                break; // Success
            }
        }
    }

    /**
     * Applying Configurations to Kraken
     */
    private void configureMotor() {

        // Set peak current for torque limiting for stall prevention
        motorConfig.TorqueCurrent.PeakForwardTorqueCurrent = STEER_PEAK_CURRENT;
        motorConfig.TorqueCurrent.PeakReverseTorqueCurrent = - STEER_PEAK_CURRENT;

        // How fast can the code change torque for the motor
        motorConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = STEER_RAMP_RATE;

        // By Default Robot will not move 
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Encoder Being Applied
        motorConfig.Feedback.FeedbackRemoteSensorID = cancoder.getDeviceID(); 
        motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder; 

        // Enable position wrapping (by default values are from 0-1)
        closedLoopGeneralConfigs.ContinuousWrap = true; //basicaly turns stacking off
        motorConfig.ClosedLoopGeneral = closedLoopGeneralConfigs;

        // Apply motor config with retries (max 5 attempts)
        for (int i = 0; i < 5; i++) {
            if (motor.getConfigurator().apply(motorConfig, 0.1) == StatusCode.OK) {
                break; // Success
            }
        }
    }


    /**
     * Gets the motor's position through the absolute encoder
     * 
     * @return position in double from 0 to 1
     */
    public double getPosition() {
        return motor.getPosition().getValueAsDouble();
    }

    /**
     * Configures drive motor's PIDSV
     * @param p :Determines how much the config will react to the error
     * @param i :Corrects recurring errors over time by stacking past errors
     * @param d :Reacting to the rate of change of the error, preventing overshooting and damping oscillations
     * @param s :Helps with overcoming initial friction or resistance in systems
     * @param v :Compensates for the velocity or speed at which the system is moving
     */
    public void configPID(double p, double i, double d, double s, double v) {

        Slot0Configs slot0Configs = new Slot0Configs(); //used to store and update PID values

        /*
         * Think of P as how much we want it to correct, as an example imagine you are parking a car
         *      If you’re too far from the spot, you might turn the steering wheel sharply (a large correction) to get closer.
         *      If you’re very close to the spot, you might make smaller, finer adjustments (a smaller correction).
         * 
         * The proportional control is like how hard you turn the steering wheel based on how far you are from the target spot. If you’re far away, you turn more; if you’re close, you turn less.
         */

        slot0Configs.kP = p;

        /*
         * Integral Control's job is to correct recurring errors over time by stacking past errors.
         * It sums up previous errors, so it looks at how many errors you have had over time.
         * 
         * Imagine the motor is consistently below the target position due to friction.
         *      kI will accumulate this error over time and then apply this correction to bring the motor to its target.
         * 
         * Keep in mind that this will also apply if something is blocking the mechanism from working, e.g., a rock.
         * This will lead to a crazy increase in errors, potentially causing motors to burn out.
         */
        slot0Configs.kI = i;

        /*
         * The Derivitive Controls job is to look at the Rate of Change (slope) of how fast the error is changing (def of derrivitive)
         * If the error is chaning too fast, the kD will slow it down so we do not overshoot
         * 
         * Imagine you’re driving a car toward a stop sign. As you approach the stop sign, you start to apply the brakes. 
         *      The derivative control is like reacting to how quickly you’re approaching the stop sign:
         * 	        If you’re coming in fast, the derivative term would apply more braking force to slow you down before you overshoot the stop sign.
         *          If you’re coming in slowly, the derivative term would apply less braking force, just enough to prevent overshooting but not too much.
         * 
         * kD responds to the rate of change of the error, preventing overshooting and damping oscillations, too high will make mech slower
         */
        slot0Configs.kD = d;

        /*
         * Static is added to make sure the motor can spin properly even without presence of errors,
         *      this includes things like power needed to overcome friction, drag, or the inertia
         * 
         * Imagine trying to turn a gear.
         *      Even if you’re not trying to change its position, you still need to exert some force to overcome static friction — the resistance between the object and the surface, after doing so it will be more easy to turn
         *          kS provides that initial extra force to overcome the friction and start turning the gear.
         *          Once the gear starts moving, we will switch to using kP, kI, and kD terms.
         * 
         * kS helps with overcoming initial friction or resistance in systems
         */
        slot0Configs.kS = s;

        /*
         * Velocity compensates for the velocity or speed at which the system is moving, think if it like correction of speed
         * 
         * The kV term is proportional to the velocity or speed of the system, meaning that as the speed increases:
         *       the control system will apply an appropriate amount of correction based on the speed at which the system is moving.
         * 
         * Imagine driving a car and trying to maintain a  speed. The kV term is like a cruise control system that adjusts the throttle based on how fast you’re going:
         *      	If you’re going too fast, the kV term will reduce the throttle (power) to slow you down.
		 *          If you’re going too slow, it will increase the throttle to speed up.
         * 
         * kV adjusts the system’s response based on its speed, helping to prevent overshooting and stabilize motion with goal of keeping constant speed.
         */
        slot0Configs.kV = v;

        motor.getConfigurator().apply(slot0Configs);
    }


    /**
     * Gets the tempature of the motor
     * @return temperature of the motor in double
     */
    public double getTemperature() {
        return motor.getDeviceTemp().getValueAsDouble();
    }
    

    private double degreesToMotorRotations(double degrees) {
        return (degrees / 360.0) * STEER_GEAR_REDUCTION;
    }

    public static double fasterTurnDirection(double current, double target) {

        // Normalize the angles between 0 and 360
        current = ((current % 360) + 360) % 360;
        target = ((target % 360) + 360) % 360;

        // Calculate clockwise and counterclockwise distances
        double clockwise = (target - current + 360) % 360;
        double counterClockwise = (current - target + 360) % 360;

        // Determine the faster direction
        if (clockwise <= counterClockwise) {
            return clockwise;
        } else {
            return counterClockwise;
        }
    }

    /**
     * Using PID to move to target position
     * 
     * @param targetRads target position in radiants
     */
    public void setPosition(double targetRads) {

        // Convert RADS to Degree
        double targetDegrees = (targetRads * 360) / (2. * Math.PI);

        // Gets Current Position in *
        double currentDegrees = getPosition() * 360.0; 

        // Figure Out position to go To
        double goalTurn = fasterTurnDirection(currentDegrees, targetDegrees);

        // Converts from degrees to rotations
        double desiredMotorRotations = degreesToMotorRotations(goalTurn);

        //Creates a reqest to go to that said position
        positionRequest.withPosition(desiredMotorRotations);
        motor.setControl(positionRequest);
    }
}


