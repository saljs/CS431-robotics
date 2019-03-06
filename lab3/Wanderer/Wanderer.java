import lejos.hardware.BrickFinder;
import lejos.hardware.ev3.EV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorMode;
import java.lang.*;

public class Wanderer {
    //Private Variables
    //Target
    private float r;
    private float theta;
    
    //Ultrasonic Sensors
    private EV3UltrasonicSensor forward;
    private EV3UltrasonicSensor backLeft;
    private EV3UltrasonicSensor backRight;

    //Ultrasonic Sensor Sample Providers
    private SampleProvider distForward;
    private SampleProvider distBackR;
    private SampleProvider distBackL;
    

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
    public Wanderer(BaseRegulatedMotor l,
		    BaseRegulatedMotor r,
		    Port f, Port bL,
		    Port bR){
	this.left = l;
	this.right = r;
	
	this.forward = new EV3UltrasonicSensor(f);
	this.backLeft = new EV3UltrasonicSensor(bL);
	this.backRight = new EV3UltrasonicSensor(bR);

	this.distForward = forward.getDistanceMode();
	this.distBackR = backRight.getDistanceMode();
	this.distBackL = backLeft.getDistanceMode();    
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

    private float vectorSum(){
	float r = 0;
	float xSum = 0;
	float ySum = 0;
	float[] distanceSample = new float[this.distForward];
	this.distForward.fetchSample(distanceSample,0);
	r = averageDistance(distanceSample);
	xSum= xSum + r * Math.cos(0);
	ySum = ySum + r * Math.sin(0);
	this.distBackR.fetchSample(distanceSample,0);
	r = averageDistance(distanceSample);
	xSum= xSum + r * Math.cos(0);
	ySum = ySum + r * Math.sin(0);
	this.distBackL.fetchSample(distanceSample,0);
	r = averageDistance(distanceSample);
	xSum= xSum + r * Math.cos(0);
	ySum = ySum + r * Math.sin(0);
	this.r = Math.sqrt((ySum*ySum)+(xSum*xSum));
	this.theta = Math.atan(ySum/xSum);
	//r and theta with vector sum
	return this.r;
    }

    private float averageDistance(float[] sample){
	float avg = 0;
	for (int ndx = 0; ndx < sample.length ; ndx++){
	    avg += sample[ndx];
	}
	return avg/(sample.length);
    }
    
}  
