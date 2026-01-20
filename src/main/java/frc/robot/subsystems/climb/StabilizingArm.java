package frc.robot.subsystems.climb;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTalon;

public class StabilizingArm extends SubsystemBase {

    private TalonFX motor = new TalonFX(0, getName(), null)public StabilizingArm()
    {

    }

}
