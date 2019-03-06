import lejos.hardware.BrickFinder;
import lejos.hardware.ev3.EV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorMode;

public class Wanderer {
    //Private Variables
    //Target
    private float r;
    private float theta;
    
    //Ultrasonic Sensors
    private EV3UltrasonicSensor forward;
    private EV3UltrasonicSensor backLeft;
    private EV3UltrasonicSensor backRight;
   

    //Motors
    private BaseRegulatedMotor left;
    private BaseRegulatedMotor right;
    
    public static void main(String[] args) {
	Wanderer pilot = new Wanderer((BaseRegulatedMotor)Motor.C, 
				    (BaseRegulatedMotor)Motor.A,
				    SensorPort.S1, SensorPort.S2,
				    SensorPort.S3);
	pilot.drive(200);
	/*
	final EV3 ev3 = (EV3) BrickFinder.getLocal();
	TextLCD lcd = ev3.getTextLCD();

	// THIS IS THE RELEVANT CODE
	EV3UltrasonicSensor sonicSensor = new EV3UltrasonicSensor( SensorPort.S3 );
	SampleProvider distance = sonicSensor.getDistanceMode();
	float[] distanceSample = new float[ distance.sampleSize() ];

	long startTime = System.currentTimeMillis();
	long duration;

	do {
	    duration = System.currentTimeMillis() - startTime;
	    // TAKING READINGS FROM THE ULTRA_SONIC SENSOR
	    distance.fetchSample(distanceSample, 0); 
	    } while (duration < 60000);*/
    }

    /*
     * Constructor Method
     */
    public Wanderer(BaseRegulatedMotor left,
		    BaseRegulatedMotor right,
		    Port forward, Port backLeft,
		    Port backRight){
	this.left = left;
	this.right = right;
	this.forward = forward;
	this.backLeft = backLeft;
	this.backRight = backRight;
    }

    public void drive(float speed){
	while(Button.ESCAPE.isUp()){
	    if(vectorSum() > 0){
		toTarget();
	    } else{
		wander();
	    }
	}
    }
    
    private void wander(){
	// if vectorSum == 0, then no target
	//generate random target
    }

    private void toTarget(){
	//if r != 0, move toward target
    }

    private int vectorSum(){
	//read sensors, calculate target
	//r and theta with vector sum
	//return r
    }
    
}  
