/**
 * 
 */
package es.uma.lcc.caesium.problem.aircontrol;

import java.util.HashMap;
import java.util.Map;

/**
 * Encapsulates airplane information
 * @author ccottap
 * @version 1.0	 
 */
public class Flight {
	/**
	 * flight ID
	 */
	private String flightID;
	/**
	 * type of plane
	 */
	private AirplaneType type;
	/**
	 * expected time of arrival to each runway
	 */
	private Map<Integer,Long> expectedTimeArrival;
	
	/**
	 * Creates the flight. Arrival times are left unspecified
	 * @param flightID the ID of the flight
	 * @param type the type of plane
	 */
	public Flight (String flightID, AirplaneType type) {
		this.flightID = flightID;
		this.type = type;
		expectedTimeArrival = new HashMap<Integer, Long>();
	}

	/**
	 * Returns the flight ID
	 * @return the flight ID
	 */
	public String getFlightID() {
		return flightID;
	}

	/**
	 * Returns the plane type
	 * @return the plane type
	 */
	public AirplaneType getType() {
		return type;
	}
	
	/**
	 * Returns the expected arrival time at a certain runway
	 * @param runway the runway
	 * @return the expected arrival time
	 */
	public long getArrivalTime (int runway) {
		return expectedTimeArrival.get(runway);
	}

	/**
	 * Set the arrival time on a specific runway
	 * @param runway the runway
	 * @param time the time of arrival
	 */
	public void setArrivalTime (int runway, long time) {
		expectedTimeArrival.put(runway, time);
	}

	@Override
	public String toString() {
		String str = "Flight " + flightID + " (" + type + "):\t";
		
		for (int r: expectedTimeArrival.keySet()) {
			str += expectedTimeArrival.get(r) + "\t";
		}
		
		return str;
	}
	
}