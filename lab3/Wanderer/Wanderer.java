import lejos.hardware.BrickFinder;
import lejos.hardware.ev3.EV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorMode;

public class Wanderer {
  public static void main(String[] args) {
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
    } while (duration < 60000);
  }
	
}  
