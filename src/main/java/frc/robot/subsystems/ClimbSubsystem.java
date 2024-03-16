package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase{
    // Climber Motor
    private final TalonFX m_climbMotor = new TalonFX(3);

    public void releaseClimb()
    {
        m_climbMotor.set(-0.5);
    }

    public void beginClimb()
    {
        m_climbMotor.set(0.5);
    }

    public void stopClimb()
    {
        m_climbMotor.set(0);
    }

}
