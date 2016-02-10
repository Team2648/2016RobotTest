
package org.usfirst.frc.team2648.robot;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	private Joystick j1;
	private Joystick j2;
	private SpeedController shooter;
	private SpeedController intake;
	private SpeedController intakelift1;
	private SpeedController intakelift2;
	private SpeedController right;
	private SpeedController left;
	private RobotDrive rd;
	private DigitalInput light;
	private Encoder enc;
	private BuiltInAccelerometer accel;
	
	private PIDController controller;
	private PIDOutput out;
    
	private double timerStart;
    private double timeToCancel;
    
    private Compressor comp;
    private DoubleSolenoid inup;
	
    public void robotInit() {
    	j1 = new Joystick(1);
    	j2 = new Joystick(0);
    	shooter = new Victor(2);
    	intake = new Victor(3);
    	intakelift1 = new Victor(4);
    	intakelift2 = new Victor(5);
    	right = new Victor(0);
    	left = new Victor(1);
    	rd = new RobotDrive(left, right);
    	light = new DigitalInput(3);
    	
    	comp = new Compressor();
    	inup = new DoubleSolenoid(0,1);
    	
    	enc = new Encoder(0,1,2);
    	enc.setDistancePerPulse(.01227); //.1256 inches traveled with each pulse
    	enc.setPIDSourceType(PIDSourceType.kDisplacement); 
    	
    	out = new PIDOutput(){
    		@Override
    		public void pidWrite(double out){
    			rd.setLeftRightMotorOutputs(out, out);
    		}
    	};
    	
    	controller = new PIDController(.7,0,0,enc,out);
    	controller.setAbsoluteTolerance(.0125);
    	controller.disable();
    	
    	timerStart = -1;
    	timeToCancel = -1;
    }
       
    public void autonomousInit() {
    	
    }
    	
    public void autonomousPeriodic() {
    	
    }

    public void teleopPeriodic() { 
    	/*SmartDashboard.getNumber("Accel x: ", accel.getX());
    	SmartDashboard.getNumber("Accel Y: ", accel.getY());
    	SmartDashboard.getNumber("Accel Z: ", accel.getZ());
    	if(controller.isEnabled()){
    		if(((Timer.getFPGATimestamp()-timerStart) > timeToCancel) || controller.onTarget()){
    			controller.disable();
    			timerStart = -1;
    			timeToCancel = -1;
    		}
    	}
    	
 /*   	if(j1.getRawButton(2)){
    		enc.reset();
    		controller.setSetpoint(24);; //destination 24 inches
    		
    		timerStart = Timer.getFPGATimestamp();
    		timeToCancel = 5;//timeout after 5 seconds
    		controller.enable();
    	}
    	else if(j1.getRawButton(3)){
    		controller.disable();
    		timerStart = -1;
    		timeToCancel = -1;
    	}*/
    	
/*    	if(!controller.isEnabled()){
    		rd.arcadeDrive(-j1.getY(),j1.getX());
    		if(light.get()){
    			System.out.println("Photoswitch: " + light.get());
    		}
    		else{
    			System.out.println("Photoswitch: " + light.get());
    		}
    	}*/
    	
    	//rd.arcadeDrive(-j2.getY(),-j2.getX());
    	rd.arcadeDrive(-j2.getRawAxis(1),-j2.getRawAxis(0));
    	if(j2.getRawAxis(2) != 0){
    		shooter.set(-j2.getRawAxis(2));
    		SmartDashboard.putNumber("Shooter: ", shooter.get());
    	}
    	else{
    		shooter.set(0);
    	}
    	if(j2.getRawButton(8)){
    		shooter.set(-1);
    		intake.set(.5);
    		Timer.delay(.50);
    		intake.set(0);
    		Timer.delay(1.5);
    		intake.set(-1);
    		Timer.delay(1);
    		shooter.set(0);
    		intake.set(0);
    	}
    	if(j2.getRawAxis(3) != 0){
    		intake.set(-1*j2.getRawAxis(3));
    		SmartDashboard.putNumber("Intake: ", intake.get());
    	}
    	else if(j2.getRawButton(4)){
    		intake.set(-.5);
    		shooter.set(-1);
    	}
    	else if(j2.getRawButton(5)){
    		intake.set(-1);
    	}
    	else if(j2.getRawButton(6)){
    		intake.set(1);
    	}
    	else{
    		intake.set(0);
    	}
    	
    	if(j2.getRawAxis(1) != 0){  
    		intakelift1.set(j2.getRawAxis(1));
    		intakelift2.set(j2.getRawAxis(1));
    	}
    	else{
    		intakelift1.set(0);
    		intakelift2.set(0);
    	}
    	
    	if(j2.getRawButton(2)){
    		inup.set(DoubleSolenoid.Value.kForward);
    	}
    	else if(j2.getRawButton(3)){
    		inup.set(DoubleSolenoid.Value.kReverse);
    	}
    	else{
    		inup.set(DoubleSolenoid.Value.kOff);
    	}
    	
    	if(j2.getRawButton(1))
    	{
    		inup.set(DoubleSolenoid.Value.kOff);
    	}
    	
        
    }
    
    public void testPeriodic() {
    
    }
    
}
