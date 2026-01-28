package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.SwerveDrive;

public class TeleopDrive extends Command {
    private final SwerveDrive swerveDrive;
    private final CommandXboxController controller;
    
    public TeleopDrive(SwerveDrive swerveDrive, CommandXboxController controller) {
        /***
         * Fill out this command constructor
         */
		this.swerveDrive = swerveDrive;
		this.controller = controller;
		addRequirements(swerveDrive);
    }
    
    @Override
    public void execute() {
        /***
         * Look into controller.getRawAxis() method
         * Also look into OIConstants in Constants.java to make sure the controller syncs correctly
         * Might have to negate something 🤔🤔🤔. Do some trial and error.
         */
		double xInput = controller.getRawAxis(OIConstants.LEFT_X_AXIS);
        double yInput = controller.getRawAxis(OIConstants.LEFT_Y_AXIS);
        double rotInput = controller.getRawAxis(OIConstants.RIGHT_X_AXIS);

        /***
         * Apply a deadband to these inputs. Look at OIConstants.
         */
        xInput = MathUtil.applyDeadband(xInput, OIConstants.DEADBAND);
        yInput = MathUtil.applyDeadband(yInput, OIConstants.DEADBAND);
        rotInput = MathUtil.applyDeadband(rotInput, OIConstants.DEADBAND);
        
        // Challenge: square the inputs while preserving the signs

        /***
         * Use the inputs and look at constants for the following: the max speed, the speed multipler. What do we do with these?
         */
        double xSpeed = xInput * SwerveConstants.MAX_SPEED_MPS * OIConstants.SPEED_MULTIPLIER;
        double ySpeed = yInput * SwerveConstants.MAX_SPEED_MPS * OIConstants.SPEED_MULTIPLIER;
        double rotSpeed = rotInput * SwerveConstants.MAX_SPEED_MPS * OIConstants.SPEED_MULTIPLIER;
        
        /***
         * Fix the error here. What are the arguments to the method? Hint: look at the end method below...
         */
        swerveDrive.drive(new Translation2d(xSpeed, ySpeed), rotSpeed);
    }
    
    @Override
    public void end(boolean interrupted) {
        swerveDrive.drive(new Translation2d(0, 0), 0);
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
}