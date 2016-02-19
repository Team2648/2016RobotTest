
package org.usfirst.frc.team2648.robot;

import edu.wpi.first.wpilibj.AnalogGyro;
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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.interfaces.Gyro;
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
	private SpeedController shooter2;
	private SpeedController intake;
	private SpeedController right; //right side of drivetrain
	private SpeedController left; //left side of drivetrain
	private RobotDrive rd;
	private DigitalInput light; //boolean switch tester
	private Encoder enc; //drivetrain encoder
	private BuiltInAccelerometer accel; //accelerometer built into roborio
	private Gyro gyro; //drivetrain gyro
	
	private PIDController controller;
	private PIDOutput out;
    
	private double timerStart;
    private double timeToCancel;
    private double kp = 0.03;
    
    private Compressor comp;
    private DoubleSolenoid inup;
    private DoubleSolenoid inup2;
    
    private double gyroSubAvg;
    private double gyroTimeStart;
    private int totalCount;
    private int count;
	
    public void robotInit() {
    	j1 = new Joystick(1);
    	j2 = new Joystick(0);
    	shooter = new Victor(2);
    	shooter2 = new Victor(4);
    	intake = new Victor(3);
    	right = new Victor(0);
    	left = new Victor(1);
    	rd = new RobotDrive(left, right);
    	light = new DigitalInput(3);
    	gyro = new AnalogGyro(0);
    	count = 0;
    	
    	comp = new Compressor();
    	inup = new DoubleSolenoid(0,1);
    	inup2 = new DoubleSolenoid(2,3);
    	
    	enc = new Encoder(1,2,true, Encoder.EncodingType.k4X);
    	enc.setDistancePerPulse(.11977); //circumference of wheel/200 (PPR)
    	enc.setPIDSourceType(PIDSourceType.kDisplacement); 
    	enc.reset();
    	
    	out = new PIDOutput(){
    		@Override
    		public void pidWrite(double out){
    			rd.setLeftRightMotorOutputs(out, out);
    		}
    	};
    	
    	controller = new PIDController(.007,0,0,enc,out);
    	controller.setAbsoluteTolerance(.25);
    	controller.disable();
    	
    	timerStart = -1;
    	timeToCancel = -1;
    }
    
    public void disabledInit()
    {
    	gyroSubAvg = 0;
    	gyro.reset();
    }
    
    public void disabledPeriodic()
    {
    		gyroSubAvg += gyro.getAngle();
    		gyroSubAvg /= 2;
    		
    		SmartDashboard.putNumber("Gyro", gyro.getAngle());
    		SmartDashboard.putNumber("Gyro Scaling Value", gyroSubAvg);
    }
        
    public void autonomousInit() {
    	
    }
    	
    public void autonomousPeriodic() {

    }
    
    public void teleopInit(){
    	gyro.reset();
    	enc.reset();
    	
    }

    public void teleopPeriodic() { 
    	/*SmartDashboard.getNumber("Accel x: ", accel.getX());
    	SmartDashboard.getNumber("Accel Y: ", accel.getY());
    	SmartDashboard.getNumber("Accel Z: ", accel.getZ());*/
    	if(controller.isEnabled()){
    		if(((Timer.getFPGATimestamp()-timerStart) > timeToCancel) || controller.onTarget()){
    			controller.disable();
    			timerStart = -1;
    			timeToCancel = -1;
    		}
    	}
    	
    	
    	if(j1.getRawButton(2)){
    		enc.reset();
    		controller.setSetpoint(120); //destination 24 inches -> NO!! Trying to figure out this value
    		
    		timerStart = Timer.getFPGATimestamp();
    		timeToCancel = 10;//timeout after 10 seconds
    		controller.enable();
    	}
    	else if(j1.getRawButton(1)){ //this button stops the robot if the button 2 command goes crazy
    		controller.disable();
    		timerStart = -1;
    		timeToCancel = -1;
    	}
    	else{ //if time out or distance, end
    		controller.disable();
    		timerStart = -1;
    		timeToCancel = -1;
    	}
    	
    	if(!controller.isEnabled()){
    		rd.arcadeDrive(-j1.getY(),-j1.getX()); //normal arcade drive
    	}
    	
    	
    	if(j2.getRawAxis(2) != 0){ //set shooter values to the left trigger value
    		shooter.set(-j2.getRawAxis(2));
    		shooter2.set(-j2.getRawAxis(2));
    		SmartDashboard.putNumber("Shooter: ", shooter.get());
    	}
    	else{ //stop shooter
    		shooter.set(0);
    		shooter2.set(0);
    	}
    	/*if(j2.getRawButton(8)){ //runs intake, waits, runs shooter
    		shooter.set(-1);
    		intake.set(.5);
    		Timer.delay(.50);
    		intake.set(0);
    		Timer.delay(1.5);
    		intake.set(-1);
    		Timer.delay(1);
    		shooter.set(0);
    		intake.set(0);
    	}*/
    	
    	//Need to mess around with sensitivity of light sensor 
    	if(j2.getRawAxis(3) != 0 && !light.get()){ //run intake at speed of right trigger
    		intake.set(-1*j2.getRawAxis(3));
    	}
    	else if(j2.getRawButton(5) && !light.get()){ //run intake into robot
    		intake.set(-1);
    	}
    	else if(j2.getRawButton(6)){ //run intake out of robot
    		intake.set(1);
    	}
    	else{ //stop intake
    		intake.set(0);
    	}
    	
    	if(j2.getRawButton(2)){ //lift intake
    		//inup.set(DoubleSolenoid.Value.kForward);
    		inup2.set(DoubleSolenoid.Value.kForward);
    	}
    	else if(j2.getRawButton(3)){ //drop intake
    		//inup.set(DoubleSolenoid.Value.kReverse);
    		inup2.set(DoubleSolenoid.Value.kReverse);
    	}
    	else{ //solenoids off
    		inup.set(DoubleSolenoid.Value.kOff);
    		inup2.set(DoubleSolenoid.Value.kOff);
    	}
    	

    	
    	
    	//reading values
    	SmartDashboard.putNumber("Encoder Dist: ",  enc.getDistance()); //Distance is in inches
    	SmartDashboard.putNumber("Encoder: ", enc.get());
    	SmartDashboard.putNumber("Encoder Rate: ", enc.getRate());
    	SmartDashboard.putNumber("Gyro: ", gyro.getAngle());
    	SmartDashboard.putNumber("Encoder Raw", enc.getRaw());
    	SmartDashboard.putNumber("Controller: ", controller.getSetpoint());
    	SmartDashboard.putBoolean("Light Sensor: ", light.get());
    }
    
    public void testPeriodic(){
    }
    
}
