package es.uma.lcc.caesium.problem.aircontrol;

/**
 * Class for keeping the information of the airport, namely the 
 * last time a flight landed in each runway, and the last type of plane 
 * to land.
 * @author ccottap
 * @version 1.0
 */
public class AirportInformation {
	/**
	 * number of runways
	 */
	private int numRunways;
	/**
	 * time at which the last flight landed in each runway
	 */
	private long[] available;
	/**
	 * type of the last plane to land in each runway
	 */
	private AirplaneType last[];
	
	/**
	 * Creates the airport information. Initially, 
	 * no plane has landed (last time is less than 0).
	 * @param numRunways the number of runways
	 */
	public AirportInformation (int numRunways) {
		this.numRunways = numRunways;
		available = new long[numRunways];	 // last time each runway was used 
		last = new AirplaneType[numRunways]; // last type of plane to use each runway
		reset();
	}
	
	/**
	 * Creates a copy of the airport information
	 * @param ai airport information
	 */
	public AirportInformation (AirportInformation ai) {
		this(ai.numRunways);
		for (int j=0; j<numRunways; j++) {
			available[j] = ai.available[j];
			last[j] = ai.last[j];
		}

	}
	
	/**
	 * Resets the information and makes all runways available from t=0
	 */
	public void reset() {
		for (int j=0; j<numRunways; j++)
			available[j] = -1;				 // (-1 if they have not been used yet)
	}
	
	/**
	 * Returns the time at which a runway is available
	 * @param r the runway
	 * @return the time at which runway {@code r} is available
	 */
	public long getTime(int r) {
		return available[r];
	}
	
	
	/**
	 * Returns the type of the last plane to land in a runway
	 * @param r the runway
	 * @return the type of the last plane to land in runway {@code r}
	 */
	public AirplaneType getType(int r) {
		return last[r];
	}
	
	/**
	 * Updates the airport information after a flight lands on a certain runway at a certain time
	 * @param p the airplane type
	 * @param r the runway
	 * @param t the time
	 */
	public void land(AirplaneType p, int r, long t) {
		available[r] = t;
		last[r] = p;
	}
	
	@Override
	public String toString() {
		String str = "";
		for (int i=0; i<numRunways; i++) {
			str += i + ": (" + available[i] + ", " + last[i] + ") ";  
		}
			
		return str;
	}
}
