package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule {
    private final DCMotorSim driveMotorSim;
    private final DCMotorSim turnMotorSim;

    private final PIDController driveController;
    private final PIDController turnController;

    private double drivePositionMeters = 0.0;
    private double driveVelocityMPS = 0.0;
    private double turnPositionRad = 0.0;

    private double driveVelocitySetpoint = 0.0;
    private double turnPositionSetpoint = 0.0;

    private final double wheelCircumference;

    public SwerveModule() {
        driveMotorSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getFalcon500(1), 
                0.025,
                SwerveConstants.DRIVE_GEAR_RATIO
            ),
            DCMotor.getFalcon500(1)
        );
        
        turnMotorSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getNEO(1), 
                0.004,
                SwerveConstants.TURN_GEAR_RATIO
            ),
            DCMotor.getNEO(1)
        );
        
        driveController = new PIDController(
            SwerveConstants.DRIVE_KP,
            SwerveConstants.DRIVE_KI, 
            SwerveConstants.DRIVE_KD
        );
        
        turnController = new PIDController(
            SwerveConstants.TURN_KP,
            SwerveConstants.TURN_KI,
            SwerveConstants.TURN_KD
        );
        
        turnController.enableContinuousInput(-Math.PI, Math.PI);
        
        wheelCircumference = Math.PI * SwerveConstants.WHEEL_DIAMETER_METERS;
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        /***
         * Look into SwerveModuleState.optimize. Think about what we are trying to optimize.
         */
        desiredState.optimize(desiredState.angle);

        /***
         * Look at the properties of desireState that might be useful. Also make sure to keep turnPositionSetpoint in radians.
         */
        
        driveVelocitySetpoint = desiredState.speedMetersPerSecond;
		turnPositionSetpoint = desiredState.angle.getRadians();
    }

    public void updateSim(double dt) {
        double driveVoltage = calculateDriveVoltage();
        driveMotorSim.setInputVoltage(driveVoltage);
        driveMotorSim.update(dt);
        
        double driveAngularVelocity = driveMotorSim.getAngularVelocityRadPerSec();

        // Think about physics for a second...Ask for help if you need

        /***
         * How should we convert from angular velocity to translational velocity?
         */
        driveVelocityMPS = driveAngularVelocity / ((2 * Math.PI) * wheelCircumference);

        /***
         * How is position, velocity, and time related?
		 * Distance = Speed * Time
         */
        drivePositionMeters += driveAngularVelocity * dt;
        
        double turnVoltage = calculateTurnVoltage();
        turnMotorSim.setInputVoltage(turnVoltage);
        turnMotorSim.update(dt);
        
        /***
         * Fill these out. Should be quite similar to what you did for the drive motor.
         */
		double turnVelocity = turnMotorSim.getAngularVelocityRadPerSec();
        turnPositionRad += turnVelocity * dt;
        turnPositionRad = normalizeAngle(turnPositionRad);
    }

    private double calculateDriveVoltage() {
        double pidOutput = driveController.calculate(driveVelocityMPS, driveVelocitySetpoint);
        
        double feedforward = (driveVelocitySetpoint / SwerveConstants.MAX_SPEED_MPS) * 12.0;
        
        double voltage = pidOutput + feedforward;
        return clamp(voltage, -12.0, 12.0);
    }

    private double calculateTurnVoltage() {
        double pidOutput = turnController.calculate(turnPositionRad, turnPositionSetpoint);
        return clamp(pidOutput, -12.0, 12.0);
    }

    public Rotation2d getAngle() {
        return new Rotation2d(turnPositionRad);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(driveVelocityMPS, getAngle());
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(drivePositionMeters, getAngle());
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private double normalizeAngle(double angleRad) {
        double normalized = angleRad % (2 * Math.PI);
        if (normalized > Math.PI) {
            normalized -= 2 * Math.PI;
        } else if (normalized < -Math.PI) {
            normalized += 2 * Math.PI;
        }
        return normalized;
    }

    public double getVelocity() {
        return driveVelocityMPS;
    }

    public double getPosition() {
        return drivePositionMeters;
    }

}
