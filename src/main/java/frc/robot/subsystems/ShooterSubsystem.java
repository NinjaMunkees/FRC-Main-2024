package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    // Shooter Motor
    private final TalonFX m_shooter = new TalonFX(8);
    // Vars
    private final double SHOOTER_SPEED = 1600;

    public boolean isAtSpeed() {
        return m_shooter.get() == SHOOTER_SPEED;
    }

    public void spoolShooter() {
        m_shooter.set(SHOOTER_SPEED);
    }

    public void stopShooter() {
        m_shooter.set(0);
    }
}
