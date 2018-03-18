package flowSensor;

import java.nio.ByteBuffer;

import org.usfirst.frc4579.filters.AverageFilter;
import org.usfirst.frc4579.testRobot2018.Robot;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SensorBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

public class FlowMotion extends SensorBase  {
	// Variables for the SPI interface and sensor data.
	private SPI flow = new SPI(SPI.Port.kOnboardCS0);
	private ByteBuffer regBuffer = ByteBuffer.allocate(2);    //SPI transaction buffer.
	private ByteBuffer longBuffer = ByteBuffer.allocate(5);   //SPI transaction buffer.
	private byte[] counts = new byte[5];					//Byte buffer for received data.
	private int oldDeltaX, oldDeltaY;						//Previous readings.
	public int deltaX, deltaY;								//Current readings.
	public int accumDeltaX, accumDeltaY = 0;				//Accumulated readings.
	public double rateX, rateY = 0.0;						//Calculated instantaneous motion rate, counts/sec.
	public AverageFilter filteredDeltaX = new AverageFilter(5);
	public AverageFilter filteredDeltaY = new AverageFilter(5);
	
	public void reset(){
		accumDeltaX = 0;
		accumDeltaY = 0;
		}
	
	public void getCounts() {
		double newTime, deltaTime, oldTime = 0.0;		//delta time variables.
		newTime = Timer.getFPGATimestamp();	
		deltaTime = newTime - oldTime;
		oldTime = newTime;
		
		regBuffer.put(0, (byte)(2));              //Start with Reg No. 0x02.
		flow.transaction(regBuffer, longBuffer, 6);  //Do the SPI transaction.
		for (int i=0; i<5; i++) counts[i] = longBuffer.get(i);  //Get the returned bytes into the array.

		/*Alternate diagnostic code segment to print out all the sensor registers, in hex.
		regBuffer.put(0, (byte)(2));
		flow.transaction(regBuffer,  testBuffer, 12);
		for (int i=0; i<11; i++) System.out.printf(" %02X", (byte)testBuffer.get(i));
		System.out.println();
		*/
		
		//Diagnostic: print the byte at Reg 02.
		//Convert the returned bytes to signed int's.
		deltaX = (counts[2] << 8) | (counts[1] & 0x000000FF);
		deltaY = (counts[4] << 8) | (counts[3] & 0x000000FF);
		System.out.printf(" %02X  %02X%02X  %02X%02X  %d\n", (byte)counts[0], (byte)counts[2], (byte)counts[1], (byte)counts[4], (byte)counts[3], deltaX);

		//Test for motion, and zero the data if none.
 		if ((byte)(counts[0] & 0x80) != (byte)0x80) {
 			deltaX = 0;
 			deltaY = 0;
 		}
// 		deltaX = (int)((.5*deltaX)+(.5*oldDeltaX));
// 		deltaY = (int)((.5*deltaY)+(.5*oldDeltaY));
 		

		//		if (Math.abs(deltaX) > 250)  {
//			deltaX = oldDeltaX;	//Reject spurious readings.
//			System.out.printf("*** Max deltaX!  %02x%02x%02x%02x%02x\n", (byte)counts[0], (byte)counts[1], (byte)counts[2], (byte)counts[3], (byte)counts[4]);
//		}
//		if (Math.abs(deltaY) > 124)  {
//			deltaY = oldDeltaY;
//			System.out.printf("*** Max deltaX!  %02x%02x%02x%02x%02x\n", (byte)counts[0], (byte)counts[1], (byte)counts[2], (byte)counts[3], (byte)counts[4]);
//		}
		oldDeltaX = deltaX;									//Save the newest readings.
		oldDeltaY = deltaY;
		accumDeltaX += filteredDeltaX.filter(deltaX);		//Accumulate the latest readings.
		accumDeltaY += filteredDeltaY.filter(deltaY);
		rateX = deltaX /  deltaTime;						//Calculate the rates.
		rateY = deltaY /  deltaTime;
	//End of getCounts().
	}
	
	  private void registerWrite(byte reg, byte value)  {
		  regBuffer.put(0, (byte)(reg | 0x80));				//The high bit required by the sensor reg address.
		  regBuffer.put(1, (byte)value);
		  //System.out.printf("Reg Write: %02x%02x\n", writeBuffer.get(0), writeBuffer.get(1));
		  flow.transaction(regBuffer, regBuffer, 2);		//Simple SPI transaction.
	  }
	  
	  public boolean init() {
	    //Initialize the SPI interface.
		flow.setClockRate(800000);					//SPI settings.
		flow.setChipSelectActiveLow();
		flow.setClockActiveHigh();
		flow.setSampleDataOnRising();
		flow.setMSBFirst();
		flow.resetAccumulator();
		flow.freeAuto();
		System.out.println("End of SPI set up.");
		// Power on reset the sensor.
		regBuffer.put(0, (byte)0x3A);				//Send these two addresses to the Flow Breakout to POR.
		regBuffer.put(1, (byte)0x5A);
		flow.write(regBuffer, 2);
	    //System.out.println("End of Sensor Power On Reset.");
		
	    Timer.delay(.005);  // 5 millisecond delay

	    // Test the SPI communication, checking chipId and inverse chipId
		regBuffer.put(0, (byte)0x00);
		flow.transaction(regBuffer, regBuffer, 2);
		System.out.printf("chipId:  %02x    %02x\n", (byte)0x49, regBuffer.get(1));
		byte chipId = regBuffer.get(1);
		
		regBuffer.put(0, (byte)0x5F);
		flow.transaction(regBuffer, regBuffer, 2); 
		System.out.printf("dIpihc:  %02x    %02x\n", (byte)0xB6, regBuffer.get(1));
		byte dIpihc = regBuffer.get(1);
		
		if (chipId == (byte)0x49 && dIpihc == (byte)0xB6) {
			System.out.println("Motion Sensor is on line!");
			//Read one set of counts to initialize (per Arduino code).
			getCounts();
		
			Timer.delay(0.001);
			registerWrite((byte)0x7F, (byte)0x00);
		    registerWrite((byte)0x61, (byte)0xAD);
		    registerWrite((byte)0x7F, (byte)0x03);
		    registerWrite((byte)0x40, (byte)0x00);
		    registerWrite((byte)0x7F, (byte)0x05);
		    registerWrite((byte)0x41, (byte)0xB3);
		    registerWrite((byte)0x43, (byte)0xF1);
		    registerWrite((byte)0x45, (byte)0x14);
		    registerWrite((byte)0x5B, (byte)0x32);
		    registerWrite((byte)0x5F, (byte)0x34);
		    registerWrite((byte)0x7B, (byte)0x08);
		    registerWrite((byte)0x7F, (byte)0x06);
		    registerWrite((byte)0x44, (byte)0x1B);
		    registerWrite((byte)0x40, (byte)0xBF);
		    registerWrite((byte)0x4E, (byte)0x3F);
		    registerWrite((byte)0x7F, (byte)0x08);
		    registerWrite((byte)0x65, (byte)0x20);
		    registerWrite((byte)0x6A, (byte)0x18);
		    registerWrite((byte)0x7F, (byte)0x09);
		    registerWrite((byte)0x4F, (byte)0xAF);
		    registerWrite((byte)0x5F, (byte)0x40);
		    registerWrite((byte)0x48, (byte)0x80);
		    registerWrite((byte)0x49, (byte)0x80);
		    registerWrite((byte)0x57, (byte)0x77);
		    registerWrite((byte)0x60, (byte)0x78);
		    registerWrite((byte)0x61, (byte)0x78);
		    registerWrite((byte)0x62, (byte)0x08);
		    registerWrite((byte)0x63, (byte)0x50);
		    registerWrite((byte)0x7F, (byte)0x0A);
		    registerWrite((byte)0x45, (byte)0x60);
		    registerWrite((byte)0x7F, (byte)0x00);
		    registerWrite((byte)0x4D, (byte)0x11);
		    registerWrite((byte)0x55, (byte)0x80);
		    registerWrite((byte)0x74, (byte)0x1F);
		    registerWrite((byte)0x75, (byte)0x1F);
		    registerWrite((byte)0x4A, (byte)0x78);
		    registerWrite((byte)0x4B, (byte)0x78);
		    registerWrite((byte)0x44, (byte)0x08);
		    registerWrite((byte)0x45, (byte)0x50);
		    registerWrite((byte)0x64, (byte)0xFF);
		    registerWrite((byte)0x65, (byte)0x1F);
		    registerWrite((byte)0x7F, (byte)0x14);
		    registerWrite((byte)0x65, (byte)0x67);  //was 0x60. JGH
		    registerWrite((byte)0x66, (byte)0x08);
		    registerWrite((byte)0x63, (byte)0x70);  //was 0x78. JGH
		    registerWrite((byte)0x7F, (byte)0x15);
		    registerWrite((byte)0x48, (byte)0x48);  //was 0x58. JGH
		    registerWrite((byte)0x7F, (byte)0x07);
		    registerWrite((byte)0x41, (byte)0x0D);
		    registerWrite((byte)0x43, (byte)0x14);
		    registerWrite((byte)0x4B, (byte)0x0E);
		    registerWrite((byte)0x45, (byte)0x0F);
		    registerWrite((byte)0x44, (byte)0x42);
		    registerWrite((byte)0x4C, (byte)0x80);
		    registerWrite((byte)0x7F, (byte)0x10);
		    registerWrite((byte)0x5B, (byte)0x02);
		    registerWrite((byte)0x7F, (byte)0x07);
		    registerWrite((byte)0x40, (byte)0x41);
		    registerWrite((byte)0x70, (byte)0x00);
		    Timer.delay(0.01);  //Delay 10 milliseconds.  was 100msec. JGH
		    registerWrite((byte)0x32, (byte)0x44);
		    registerWrite((byte)0x7F, (byte)0x07);
		    registerWrite((byte)0x40, (byte)0x40);
		    registerWrite((byte)0x7F, (byte)0x06);
		    registerWrite((byte)0x62, (byte)0xF0);
		    registerWrite((byte)0x63, (byte)0x00);
		    registerWrite((byte)0x7F, (byte)0x0D);
		    registerWrite((byte)0x48, (byte)0xC0);
		    registerWrite((byte)0x6F, (byte)0xD5);
		    registerWrite((byte)0x7F, (byte)0x00);
		    registerWrite((byte)0x5B, (byte)0xA0);
		    registerWrite((byte)0x4E, (byte)0xA8);
		    registerWrite((byte)0x5A, (byte)0x50);
		    registerWrite((byte)0x40, (byte)0x80);
		    registerWrite((byte)0x7F, (byte)0x00);  //new command.  JGH
		    registerWrite((byte)0x5A, (byte)0x10);  //new command.  JGH
		    registerWrite((byte)0x54, (byte)0x00);  //new command.  JGH		    System.out.println("End of sensor performance settings.");
			return true;
		}
		return false;
	// End of Init().
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		// TODO Auto-generated method stub
	}
//End of FlowMotion Class definition.
}
