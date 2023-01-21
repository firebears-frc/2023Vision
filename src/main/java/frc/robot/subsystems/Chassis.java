package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.PowerDistribution;

import static frc.robot.Constants.*;

public class Chassis extends SubsystemBase {
    private CANSparkMax frontLeftMotor;
    private CANSparkMax frontRightMotor;
    private CANSparkMax rearLeftMotor;
    private CANSparkMax rearRightMotor;
    private DifferentialDrive differentialDrive;
    private RelativeEncoder leftEncoder;
    private RelativeEncoder righEncoder;
    private AnalogPotentiometer ultrasonic;
    private double leftOffSet = 0;
    private double rightOffSet = 0;

    private final PowerDistribution m_powerDistribution;

    public Chassis(PowerDistribution powerDistribution) {
        frontLeftMotor = new CANSparkMax(CHASSIS_FRONT_LEFT_MOTOR_CAN_ID, MotorType.kBrushless);
        frontLeftMotor.setInverted(false);
        frontLeftMotor.setIdleMode(IdleMode.kBrake);


        frontRightMotor = new CANSparkMax(CHASSIS_FRONT_RIGHT_MOTOR_CAN_ID, MotorType.kBrushless);
        frontRightMotor.setInverted(true);
        frontRightMotor.setIdleMode(IdleMode.kBrake);

        rearLeftMotor = new CANSparkMax(CHASSIS_REAR_LEFT_MOTOR_CAN_ID, MotorType.kBrushless);
        rearLeftMotor.setInverted(false);
        rearLeftMotor.setIdleMode(IdleMode.kBrake);

        rearLeftMotor.follow(frontLeftMotor);

        rearRightMotor = new CANSparkMax(CHASSIS_REAR_RIGHT_MOTOR_CAN_ID, MotorType.kBrushless);
        rearRightMotor.setInverted(true);
        rearRightMotor.setIdleMode(IdleMode.kBrake);

        rearRightMotor.follow(frontRightMotor);

        differentialDrive = new DifferentialDrive(frontLeftMotor, frontRightMotor);

        m_powerDistribution = powerDistribution;

    }

    @Override
    public void periodic() {
        if (DEBUG) {
            SmartDashboard.putNumber("ultrasonic", getUltrasonicDistanceInches());
            SmartDashboard.putNumber("getDistance", getEncoderDistance());
            SmartDashboard.putBoolean("Voltage", m_powerDistribution.getVoltage() > 11);
         }
    }

    @Override
    public void simulationPeriodic() {
    }

    public void arcadeDrive(double speed, double rotation) {
        differentialDrive.arcadeDrive(speed * -1, rotation * -1);
    }

    /**
     * Reset encoder to zero.
     */
    public void resetEncoder() {
        leftOffSet = leftEncoder.getPosition();
        rightOffSet = righEncoder.getPosition();
    }

    /**
     * @return distance in inches.
     */
    public double getEncoderDistance() {
        double inches = 2.3 * ((leftEncoder.getPosition() - leftOffSet) + (righEncoder.getPosition() - rightOffSet))
                / 2;
        inches = inches * -1;

        return inches;
    }

    public double getUltrasonicDistanceInches() {
        return ultrasonic.get() / 0.193;
    }

    public void driveToPosition(double inches) {
       
    }

    public void setBrake(Boolean brake) {
        if (brake) {
            frontLeftMotor.setIdleMode(IdleMode.kBrake);
            frontRightMotor.setIdleMode(IdleMode.kBrake);
            rearLeftMotor.setIdleMode(IdleMode.kBrake);
            rearRightMotor.setIdleMode(IdleMode.kBrake);
        } else {
            frontLeftMotor.setIdleMode(IdleMode.kCoast);
            frontRightMotor.setIdleMode(IdleMode.kCoast);
            rearLeftMotor.setIdleMode(IdleMode.kCoast);
            rearRightMotor.setIdleMode(IdleMode.kCoast);
        }
    }

}