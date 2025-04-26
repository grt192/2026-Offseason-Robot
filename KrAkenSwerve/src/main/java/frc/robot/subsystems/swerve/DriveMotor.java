package frc.robot.subsystems.swerve;

//Constants Import 
import static frc.robot.Constants.SwerveConstants.DRIVE_GEAR_REDUCTION;
import static frc.robot.Constants.SwerveConstants.DRIVE_WHEEL_CIRCUMFERENCE;
import static frc.robot.Constants.SwerveConstants.PEAK_CURRENT;
import static frc.robot.Constants.SwerveConstants.RAMP_RATE;


//CTRE imports
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;




public class DriveMotor {


    // Motor instance for controlling the drive motor
    private TalonFX motor;

    // Configuration for Kraken stored in one Object
    private final TalonFXConfiguration motorConfig = new TalonFXConfiguration();

    // For fine control of velocity and torque using FOC (Field-Oriented Control)
    private VelocityTorqueCurrentFOC torqueCurrentFOC = new VelocityTorqueCurrentFOC(0).withSlot(0);

    // Target speed in rotations per second
    private double targetRotationsPerSec = 0;

    public DriveMotor(int motorID){

        // Set Motor and reset Encoder
        motor = new TalonFX(motorID, "can");
        motor.setPosition(0);

        // Set peak current for torque limiting for stall prevention
        motorConfig.TorqueCurrent.PeakForwardTorqueCurrent = PEAK_CURRENT;
        motorConfig.TorqueCurrent.PeakReverseTorqueCurrent = - PEAK_CURRENT;

        // How fast can the code change torque for the motor
        motorConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = RAMP_RATE;

        // By Default Robot will not move 
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Apply Configurations to Motor, in a loop due to high fail rate
        while(true){
            boolean error = motor.getConfigurator().apply(motorConfig, 0.1) == StatusCode.OK;
            if (!error){
                break;
            } 
        } 

    }

    /**
     * Sets Drive Motor Velocity in Meters/Seconds
     * @param TargetMetersPerSec 
     */
    public void setVelocity(double metersPerSec){

        targetRotationsPerSec = metersPerSec / DRIVE_WHEEL_CIRCUMFERENCE * DRIVE_GEAR_REDUCTION; //turns meters per sec into wheel rotation per sec
        motor.setControl(torqueCurrentFOC.withVelocity(targetRotationsPerSec)); //apply this constant speed 
    }

    /*
     * Reset Encoder Position to 0, resets Distance
     */
    public void resetEncoder(){
        motor.setPosition(0);
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
     * Gets the distance the drive wheel has traveled.
     * @return distance the drive wheel has traveled in meters
     */
    public double getDistance() {
        return DRIVE_WHEEL_CIRCUMFERENCE / DRIVE_GEAR_REDUCTION * (motor.getPosition().getValueAsDouble());
    }

    /**
     * Get swerve wheel's velocity in m/s
     * @return swerve wheel's velocity in m/s
     */
    public double getVelocity() {
        return motor.getVelocity().getValueAsDouble() / DRIVE_GEAR_REDUCTION * DRIVE_WHEEL_CIRCUMFERENCE;
    }

    /**
     * Gets the tempature of the motor
     * @return temperature of the motor in double
     */
    public double getTemperature() {
        return motor.getDeviceTemp().getValueAsDouble();
    }

}
