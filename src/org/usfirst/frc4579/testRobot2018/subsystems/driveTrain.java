// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package org.usfirst.frc4579.testRobot2018.subsystems;

import org.usfirst.frc4579.testRobot2018.Robot;
import org.usfirst.frc4579.testRobot2018.commands.drive;
import org.usfirst.frc4579.filters.FirstOrderLPF;
import org.usfirst.frc4579.testRobot2018.RobotMap;
import org.usfirst.frc4579.testRobot2018.commands.*;
import edu.wpi.first.wpilibj.command.Subsystem;
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS


/**
 *
 */
public class driveTrain extends Subsystem {

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    private final SpeedController leftDrive = RobotMap.driveTrainleftDrive;
    private final SpeedController rightDrive = RobotMap.driveTrainrightDrive;
    private final DifferentialDrive robotDrive = RobotMap.driveTrainrobotDrive;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    public int drive_direction = 1;
    public String pidMode = "rotate";
    public final double TURN_SPEED = 0.12;

    final double baseLine = 23.125; // inches
    
    private FirstOrderLPF vLeftLPF = new FirstOrderLPF(0.7);
    private FirstOrderLPF vRiteLPF = new FirstOrderLPF(0.7);

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    /*public DriveTrain() {
    	super("DriveTrain", 2.0, 0.0, 0.0, 0.6);
		setAbsoluteTolerance(1.0);
		setInputRange(0.0, 360.0);
		setOutputRange(0.0,1.0);
		getPIDController().setContinuous(true);
		SmartDashboard.putData("PID Controller",getPIDController());
		SmartDashboard.putNumber("LEFT MOTOR OFFSET", 0.0);
		SmartDashboard.putNumber("RIGHT MOTOR SCALE FACTOR", 0.0);
    }*/
    
    // Return true if the motors are commanded to zero.
    public boolean isNotMoving() {
    	return (leftDrive.get() == 0.0) && (rightDrive.get() == 0.0);
    }
    
    public void joeyStickDrive() { //The finest drive code known to man.
		
    	//Read the gyro and the driveStick.
		double gz = Robot.measurement.getAngleRate();
    	//double gz = 0;
		double frwd = -Robot.oi.driveStick.getY();	//forward-back driveStick, speed control.
		double turn = Robot.oi.driveStick.getX();	    //left-right driveStick, turn control.

		//Lower limits for the driveStick, stop the motors.
		if (Math.abs(turn) < 0.04 && Math.abs(frwd) < 0.04) {
			turn = 0.0;
			frwd = 0.0;
			gz = 0.0;
		}
		
		//Decrease the low speed sensitivities of the driveStick.
		double frwd2 = Math.signum(frwd) * Math.pow(Math.abs(frwd), 1.5);
		double turn2 = Math.signum(turn) * Math.pow(Math.abs(turn), 2.0);
		
		//Limit the control amount at high and low speeds, to avoid spinouts.
		double maxSens = 0.55;
		double minSens = 0.2;
		double sensitivity = maxSens - Math.abs(frwd2) * (maxSens - minSens);
		turn2 = turn2 * sensitivity;
		
		//Low pass filter the speed settings to the drive motors.
		double vLeft = vLeftLPF.filter(frwd2 + turn2 / 2.0);
		double vRite = vRiteLPF.filter(frwd2 - turn2 / 2.0);
		
		//Calculate the expected rotation rate.  93 in/sec (extrapolated full speed) converts the driveStick 
		//numbers to an expected speed value. The final equation is omega = (SpeedRite - SpeedLeft)/baseline.  
		//omega is rotation in deg/sec.
		double omega = Math.toDegrees((vRite - vLeft) * 93.0 / baseLine); 
		
		//Calculate the two wheel correction factor.
		double correction  = (omega - gz) * 0.008 / 2.0;
		double vRite2 = vRite + correction;
		double vLeft2 = vLeft - correction;
		
		//Normalize the wheel speeds to stay within +/-1.0;
		double magMax = Math.max(Math.abs(vRite2), Math.abs(vLeft2));
		if (magMax > 1.0) {
			vRite2 /= magMax;
			vLeft2 /= magMax;
		}
		
		// Sets the speed scaling of the robot
		double speed = (-Robot.oi.driveStick.getThrottle() + 1.0) / 2.0;
		
		//Set the two motor speeds.
		rightDrive.set(vRite2 * speed);
		leftDrive.set(vLeft2 * speed);
		
	}  
    
    public void joeyAutoDrive(double x, double y) { //The second finest drive code known to man.
		
    	//Read the gyro and the driveStick.
		double gz = Robot.measurement.getAngleRate();
		//double gz = 0.0;
//		System.out.println(gz);
		double frwd = x;
		double turn = y;

		//Lower limits for the driveStick, stop the motors.
		if (Math.abs(turn) < 0.04 && Math.abs(frwd) < 0.04) {
			turn = 0.0;
			frwd = 0.0;
			gz = 0.0;
		}
		
		//Decrease the low speed sensitivities of the driveStick.
		double frwd2 = Math.signum(frwd) * Math.pow(Math.abs(frwd), 1.5);
		double turn2 = Math.signum(turn) * Math.pow(Math.abs(turn), 2.0);
		
		//Limit the control amount at high and low speeds, to avoid spinouts.
		double maxSens = 0.55;
		double minSens = 0.2;
		double sensitivity = maxSens - Math.abs(frwd2) * (maxSens - minSens);
		turn2 = turn2 * sensitivity;
		
		//Low pass filter the speed settings to the drive motors.
		double vLeft = vLeftLPF.filter(frwd2 + turn2 / 2.0);
		double vRite = vRiteLPF.filter(frwd2 - turn2 / 2.0);
		
		//Calculate the expected rotation rate.  93 in/sec (extrapolated full speed) converts the driveStick 
		//numbers to an expected speed value. The final equation is omega = (SpeedRite - SpeedLeft)/baseline.  
		//omega is rotation in deg/sec.
		double omega = Math.toDegrees((vRite - vLeft) * 93.0 / baseLine); 
		
		//Calculate the two wheel correction factor.
		double correction  = (omega - gz) * 0.008 / 2.0;
		double vRite2 = vRite + correction;
		double vLeft2 = vLeft - correction;
		
		//Normalize the wheel speeds to stay within +/-1.0;
		double magMax = Math.max(Math.abs(vRite2), Math.abs(vLeft2));
		if (magMax > 1.0) {
			vRite2 /= magMax;
			vLeft2 /= magMax;
		}
		
		//Set the two motor speeds.
		rightDrive.set(vRite2);
		leftDrive.set(vLeft2);
		
	}  
    
    public void driveStraight(double speed) {
	    double halfCorrection = ((Robot.measurement.getAngleRate() * .006) + (Robot.measurement.getAngle() * .028)) /2.0;
	    leftDrive.set(speed + halfCorrection);
	    rightDrive.set(speed - halfCorrection);
    }
    
    public void stop(){
    	leftDrive.stopMotor();
    	rightDrive.stopMotor();
    }
    
    
    
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void initDefaultCommand() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND

        setDefaultCommand(new drive());

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND

        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }
}

