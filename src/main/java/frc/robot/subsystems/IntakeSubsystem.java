package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    
    private VictorSPX m_intake1;
    private final static int intakeDeviceID1 = 1;
    private VictorSPX m_intake2;
    private final static int intakeDeviceID2 = 2;


     public IntakeSubsystem() {

        m_intake1 = new VictorSPX(intakeDeviceID1);
        m_intake1.configFactoryDefault();
        m_intake2 = new VictorSPX(intakeDeviceID2);
        m_intake2.configFactoryDefault();

        stop();

    }

    public void setSpeed(double speed) {
        m_intake1.set(VictorSPXControlMode.PercentOutput, speed);
        m_intake2.set(VictorSPXControlMode.PercentOutput, speed);
    }    

    public void stop() {
        setSpeed(0);
    }

    public void on(){
        setSpeed(-0.78);
    }

    public void inward() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'inward'");
    }

}
