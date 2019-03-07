import lejos.hardware.BrickFinder;
import lejos.hardware.ev3.EV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorMode;
import java.lang.*;
import java.util.Random;

public class Wanderer {

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
	pilot.drive(200,5000);
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

    public void drive(float speed, float time){
	float[] heading = {0,0};
	while(Button.ESCAPE.isUp()){
	    //wander has internal timer
	    //need to implement decay in heading
	    heading = wander(100, time);
	    avoid(heading,time);
	    Delay.msDelay(500);
	}
    }
    /*
     * adjusts actuator output
     * @param vector: heading to wander toward
     */
    private void avoid(float vector[]){
	float[] heading = vectorSum(heading);//feelforce
	turn(heading,50);
	forward(heading);
	
    }
    /*
     * generates random heading
     * @param: constraint for heading magnitude
     */
    private void wander(float distMax, float time){
	if(System.currentTimeMillis()%time == 0){//generate random vector
	    float[] vector = new float[2];
	    vector[1] = Math.Random() * 2 * Math.PI;
	    vector[0] = Math.Random() * distMax;
	}
	return vector;
    }
    
    private void turn(float vector[], float turnMag){
	float turnSpeed = 0;
	float turnScale = (vector[1]/(2*Math.PI));
	
	if(vector[1] > 0 && vector[1] < Math.PI){
	    turnSpeed = turnMag*turnScale;
	} else {
	    turnSpeed = -1*turnMag*turnScale;
	}
	
	left.setSpeed(-1*turnSpeed);
	right.setSpeed(turnSpeed);
    }
    private void forward(float speed){
	left.setSpeed(left.getSpeed() + speed);
	right.setSpeed(right.getSpeed() + speed);
    }
    
    /*
     * Performs vector sum to generate heading consider
     *  sensed data
     * @param vector: current heading
     */
    private float vectorSum(float vector[]){
	float r = 0;
	float xSum = 0;
	float ySum = 0;
	float[] heading = {0,0};
	float[] distanceSample = new float[this.distForward];
	//forward
	this.distForward.fetchSample(distanceSample,0);
	r = averageDistance(distanceSample);
	xSum= xSum - r * Math.cos(0);
	ySum = ySum - r * Math.sin(0);
	//back right
	this.distBackR.fetchSample(distanceSample,0);
	r = averageDistance(distanceSample);
	xSum= xSum - r * Math.cos(4*Math.PI/3);
	ySum = ySum - r * Math.sin(4*Math.PI/3);
	//back left
	this.distBackL.fetchSample(distanceSample,0);
	r = averageDistance(distanceSample);
	xSum= xSum - r * Math.cos(2*Math.PI/3);
	ySum = ySum - r * Math.sin(2*Math.PI/3);
	//wander heading
	xSum = xSum + vector[0]*Math.cos(vector[1]);
	ySum = ySum + vector[0]*Math.sin(vector[1]);
	
	heading[0] = Math.sqrt((ySum*ySum)+(xSum*xSum));
	heading[1] = Math.atan(ySum/xSum);

	return heading;
    }
    /*
     * calculates average distance
     * @param sample: sample of distance from sensor
     */
    private float averageDistance(float[] sample){
	float avg = 0;
	for (int ndx = 0; ndx < sample.length ; ndx++){
	    avg += sample[ndx];
	}
	return avg/(sample.length);
    }
    
}  
