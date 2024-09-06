package es.uma.lcc.caesium.problem.aircontrol;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Map;

/**
 * A permutational decoder for the Aircraft Landing Scheduling Problem
 * @author ccottap
 * @version 1.0
 */
public class LandingDecoder {
	/**
	 * an air-control problem instance
	 */
	private AirControlProblem data;
	/**
	 * to control verbosity
	 */
	private int verbosityLevel = 0;

	/**
	 * default constructor
	 */
	public LandingDecoder() {
		data = null;
	}
	
	/**
	 * Creates the decoder given a problem instance
	 * @param data the problem instance
	 */
	public LandingDecoder(AirControlProblem data) {
		setProblemData(data);
	}
	
	/**
	 * Sets the verbosity level (0 = no verbosity)
	 * @param verbosityLevel the verbosity level 
	 */
	public void setVerbositylevel(int verbosityLevel) {
		this.verbosityLevel = verbosityLevel;
	}

	/**
	 * Sets the problem data
	 * @param data an air control problem instance
	 */
	public void setProblemData (AirControlProblem data) {
		this.data = data;
	}
	
	/**
	 * Creates a list of landing assignments given an ordering of flights and their assigned runways.
	 * This landing assignment is guaranteed to be valid.
	 * @param flightOrder a list of flight IDs indicating their ordering
	 * @param runways a list of runways, to be matched by position with the list of flights
	 * @return the landing information, sorted in ascending order by landing time
	 */
	public List<LandingInformation> decode (List<String> flightOrder, List<Integer> runways) {
		assert (flightOrder.size() == runways.size()) && (flightOrder.size() == data.getNumFlights());
		
		List<LandingInformation> info = new ArrayList<LandingInformation>(data.getNumFlights());
		int numRunways = data.getNumRunways();
		int numFlights = data.getNumFlights();
		
		AirportInformation ai = new AirportInformation(numRunways);

		for (int i=0; i<numFlights; i++) {
			Flight f = data.getFlight(flightOrder.get(i));
			int r = runways.get(i);
			long expected = f.getArrivalTime(r);
			
			if (verbosityLevel > 0) {
				System.out.println("Flight " + f.getFlightID() + " to land in runway #" + r + ". Plane type = " + f.getType() + ". Expected time = " + expected);
				if (ai.getTime(r) < 0) {
					System.out.println("Runway available");
				}
				else {
					System.out.println("Last use of runway at t = " + ai.getTime(r) + " by " + ai.getType(r) + " plane");
					System.out.println("it can be used again at t = " + (ai.getTime(r)+ data.getSeparation(ai.getType(r), f.getType())));
				}
					
			}
			long t;
			if (ai.getTime(r) < 0) {
				t = expected;
			}
			else {
				t = Math.max(expected, ai.getTime(r) + data.getSeparation(ai.getType(r), f.getType()));
			}
			ai.land(f.getType(), r, t);
			info.add(new LandingInformation(f.getFlightID(), ai.getTime(r), r));
			if (verbosityLevel > 0) {
				System.out.println("Landing took place at t = " + ai.getTime(r));
			}
		}
		
		return info;
		
	}
	
	
	/**
	 * Creates a list of landing assignments given an ordering of flights. Assigns the runway which is available earlier.
	 * This landing assignment is guaranteed to be valid.
	 * @param flightOrder a list of flight IDs indicating their ordering
	 * @return the landing information, sorted in ascending order by landing time
	 */
	public List<LandingInformation> decode (List<String> flightOrder) {
		assert (flightOrder.size() == data.getNumFlights());
		
		List<LandingInformation> info = new ArrayList<LandingInformation>(data.getNumFlights());
		int numRunways = data.getNumRunways();
		int numFlights = data.getNumFlights();
		
		AirportInformation ai = new AirportInformation(numRunways);

		for (int i=0; i<numFlights; i++) {
			Flight f = data.getFlight(flightOrder.get(i));
			long best = Long.MAX_VALUE;
			int r = -1; // runway selected
			if (verbosityLevel > 0) {
				System.out.println("Flight " + f.getFlightID() + " to land. Plane type = " + f.getType());
			}
			for (int j=0; j<numRunways; j++) {
				long expected = f.getArrivalTime(j);

				if (verbosityLevel > 0) {
					System.out.println("runway #" + j + ". Expected time = " + expected);
					if (ai.getTime(j) < 0) {
						System.out.println("Runway available");
					}
					else {
						System.out.println("Last use of runway at t = " + ai.getTime(j) + " by " + ai.getType(j) + " plane");
						System.out.println("it can be used again at t = " + (ai.getTime(j) + data.getSeparation(ai.getType(j), f.getType())));
					}
				}
				long t;
				if (ai.getTime(j) < 0) {
					t = expected;
				}
				else {
					t = Math.max(expected, ai.getTime(j) + data.getSeparation(ai.getType(j), f.getType()));
				}
				if (t < best) {
					best = t;
					r = j;
				}
			}
			ai.land(f.getType(), r, best);
			info.add(new LandingInformation(f.getFlightID(), best, r));
			if (verbosityLevel > 0) {
				System.out.println("Landing took place at t = " + best + " in runway #" + r);
			}
		}
		
		return info;
		
	}
	
	
	/**
	 * Creates a list of landing assignments given an assignment of runways to flights. Assigns the runway which is available earlier.
	 * This landing assignment is guaranteed to be valid.
	 * @param runways a map assigning runways to flights
	 * @return the landing information, sorted in ascending order by landing time
	 */
	public List<LandingInformation> decode (Map<String, Integer> runways) {
		assert (runways.size() == data.getNumFlights());
		
		List<LandingInformation> info = new ArrayList<LandingInformation>(data.getNumFlights());
		int numRunways = data.getNumRunways();
		int numFlights = data.getNumFlights();
		
		var partition = new ArrayList<List<Flight>>(numRunways);
		for (int j=0; j<numRunways; j++) {
			partition.add(new ArrayList<Flight>(numFlights));
		}
		
		for (String id: data.getFlightIDs()) {
			int r = runways.get(id);
			partition.get(r).add(data.getFlight(id));
		}
		
		for (int j=0; j<numRunways; j++) {
			List<Flight> flights = partition.get(j);
			final int r = j;
			Collections.sort(flights, (a,b) -> Long.compare(a.getArrivalTime(r), b.getArrivalTime(r)));
			long available = -1;
			AirplaneType last = null;
			for (Flight f: flights) {
				if (available < 0) {
					available = f.getArrivalTime(j);
				}
				else {
					available = Math.max(f.getArrivalTime(j), available + data.getSeparation(last, f.getType()));
				}
				last = f.getType();
				info.add(new LandingInformation(f.getFlightID(), available, j));
			}
		}
		
		return info;
		
	}


}
