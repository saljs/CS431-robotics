import lejos.hardware.motor.Motor;
import lejos.hardware.motor.BaseRegulatedMotor;
import lejos.hardware.Button;
import lejos.hardware.port.Port;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.NXTUltrasonicSensor;
import lejos.hardware.sensor.NXTLightSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.robotics.SampleProvider;

public class SumoBot {
    //LED colors
    private static final int GREEN = 1;
    private static final int AMBER = 3;
    private static final int RED = 2;
    
    //Behavior modifiers
    private static final int DECAY = 2000;
    private static final float LEVEL = (float)0.15;
    private static final float DRIVE = (float)800;
    private static final float TURN = (float)1000;

    //Ultrasonic Sensor Sample Providers
    private SampleProvider frontDist;
    private SampleProvider backDist;

    //Photosensors
    private SensorMode frontLight;
    private SensorMode backLight;
    
    //Motors
    private BaseRegulatedMotor frontLeft;
    private BaseRegulatedMotor frontRight;
    private BaseRegulatedMotor backLeft;
    private BaseRegulatedMotor backRight;
    
    //internal state variables
    private float frontBackground;
    private float backBackground;
    private int avoiding;

    private float lastSpeed;

    public static void main(String[] args) {
        SumoBot pilot = new SumoBot();
        Button.LEDPattern(RED);
        while(Button.ENTER.isUp());
        long startTime = System.currentTimeMillis();
        pilot.callibrate();
        while(System.currentTimeMillis() - startTime < 5000);
        pilot.drive();
    }

    /*
     * Constructor Method
     */
    public SumoBot() {
        this.frontLeft = (BaseRegulatedMotor)Motor.D;
        this.frontRight = (BaseRegulatedMotor)Motor.A;
        this.backLeft = (BaseRegulatedMotor)Motor.C;
        this.backRight = (BaseRegulatedMotor)Motor.B;
        
        NXTUltrasonicSensor sensorFront = new NXTUltrasonicSensor(SensorPort.S2);
        NXTUltrasonicSensor sensorBack = new NXTUltrasonicSensor(SensorPort.S1);
        sensorFront.enable();
        sensorBack.enable();
        this.frontDist = sensorFront.getDistanceMode();
        this.backDist = sensorBack.getDistanceMode();
        
        NXTLightSensor colorFront = new NXTLightSensor(SensorPort.S4);
        NXTLightSensor colorBack = new NXTLightSensor(SensorPort.S3);
        this.frontLight = colorFront.getRedMode();
        this.backLight = colorBack.getRedMode();

        this.avoiding = 0;
        this.lastSpeed = DRIVE;
    }

    /*
     * Main robot loop
     */
    public void drive() {
        while(Button.ESCAPE.isUp()){
            //main drive loop
            if(avoid());
            else if(charge());
            else { find(); }
        }
        this.frontLeft.stop();
        this.frontRight.stop();
        this.backLeft.stop();
        this.backRight.stop();
        Button.LEDPattern(0);
    }

    /*
     * Calibrates the light sensors
     */
    public void callibrate() {
        this.frontBackground = readSensor(this.frontLight);
        this.backBackground = readSensor(this.backLight);
        Button.LEDPattern(AMBER);
    }

    /*
     * Avoids the line surrounding the arena
     * @return true if active
     */
    private boolean avoid() {
        if(readSensor(this.frontLight) - this.frontBackground > LEVEL) {
            backward(DRIVE);
            this.avoiding = DECAY;
        }
        else if(readSensor(this.backLight) - this.backBackground > LEVEL) {
            forward(DRIVE);
            this.avoiding = DECAY;
        }
        else if(this.avoiding > 0) {
            this.avoiding--;
        }
        else {
            return false;
        }
        this.lastSpeed = DRIVE;
        Button.LEDPattern(RED);
        return true;
    }

    /*
     * Charges at other robot
     * @return true if active
     */
    private boolean charge() {
        if(readSensor(this.frontDist) <= 1.0) {
            forward(this.lastSpeed);
        }
        else if(readSensor(this.backDist) <= 1.0) {
            backward(this.lastSpeed);
        }
        else {
            this.lastSpeed = DRIVE;
            return false;
        }
        this.lastSpeed += 10;
        Button.LEDPattern(GREEN);
        return true;
    }
    
    /*
     * Turns the robot
     */
    private void find() {
        turn(TURN);
        Button.LEDPattern(AMBER);
    }

    /*
     * Reads average value from a sensor
     * @param sensor: The sensor
     * @return The average sensor value
     */
    private float readSensor(SampleProvider sensor) {
        float[] sample = new float[sensor.sampleSize()];
        float avg = 0;
        sensor.fetchSample(sample, 0);
        for(int i = 0; i < sample.length; i++) {
            avg += sample[i];
        }
        return avg / sample.length;
    }  

    private void forward(float speed) {
        this.frontLeft.setSpeed(speed);
        this.frontRight.setSpeed(speed);
        this.backLeft.setSpeed(speed);
        this.backRight.setSpeed(speed);
        this.frontLeft.forward();
        this.frontRight.forward();
        this.backLeft.backward();
        this.backRight.backward();
    }
    private void backward(float speed) {
        this.frontLeft.setSpeed(speed);
        this.frontRight.setSpeed(speed);
        this.backLeft.setSpeed(speed);
        this.backRight.setSpeed(speed);
        this.frontLeft.backward();
        this.frontRight.backward();
        this.backLeft.forward();
        this.backRight.forward();
    }
    private void turn(float speed) {
        this.frontLeft.setSpeed(speed);
        this.frontRight.setSpeed(speed);
        this.backLeft.setSpeed(speed);
        this.backRight.setSpeed(speed);
        this.frontLeft.forward();
        this.frontRight.backward();
        this.backLeft.backward();
        this.backRight.forward();
    }
   
}  
