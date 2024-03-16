package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
    // Main Arm
    private final TalonFX m_Forearm = new TalonFX(4);

    // Grippers
    private final CANSparkMax m_Top_Grippy = new CANSparkMax(16, MotorType.kBrushless);
    private final CANSparkMax m_Bottom_Grippy = new CANSparkMax(17, MotorType.kBrushless);

    private double GRIP_SPEED = -0.2;
    private double GRIP_ALT = -0.4;

    public ArmSubsystem()
    {
        m_Forearm.setNeutralMode(NeutralModeValue.Brake);
    }

    private double ARM_SPEED = 0.3;

    public void requestBrake()
    {
        m_Forearm.setNeutralMode(NeutralModeValue.Brake);
    }

    public void moveArmUp()
    {
        //m_Forearm.setNeutralMode(NeutralModeValue.Brake);
        m_Forearm.set(ARM_SPEED);
    }

    public void moveArmDown()
    {
        //m_Forearm.setNeutralMode(NeutralModeValue.Brake);
        m_Forearm.set(-ARM_SPEED);
    }

    public void stopArm()
    {
        m_Forearm.set(0);
    }

    public void intakeNote()
    {
        m_Top_Grippy.set(GRIP_SPEED);
        m_Bottom_Grippy.set(-GRIP_SPEED);
    }
    public void ejectNote()
    {
        m_Top_Grippy.set(-GRIP_SPEED);
        m_Bottom_Grippy.set(GRIP_SPEED);
    }

    public void stopGrippers()
    {
        m_Top_Grippy.set(0.0);
        m_Bottom_Grippy.set(0.0);
    }

    public void rotateNote()
    {
        m_Top_Grippy.set(GRIP_SPEED);
        m_Bottom_Grippy.set(GRIP_ALT);
    }

    public void rotateNoteOpposite()
    {
        m_Top_Grippy.set(-GRIP_SPEED);
        m_Bottom_Grippy.set(-GRIP_ALT);
    }

    @Override
    public void periodic()
    {
        //m_Forearm.setNeutralMode(NeutralModeValue.Brake);
    }
}
